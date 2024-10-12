/*------------------------------------------------------------------------------
 * cors.c  : CORS main functions
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

static void close_cb(uv_async_t* handle)
{
    if (uv_loop_alive(handle->loop)) {
        uv_stop(handle->loop);
    }
}

static void on_timer_stat_cb(uv_timer_t* handle)
{
    NULL;
}

static void start_cors(cors_t *cors)
{
    cors_initssat(&cors->ssats);
    cors_initobs(&cors->obs);
    cors_initnav(&cors->nav);
    cors_initsta(&cors->stas);

    strcpy(cors->monitor.bstas_info_file,cors->opt.bstas_info_file);
    cors->monitor.port=cors->opt.monitor_port;

    cors_ntrip_start(&cors->ntrip,cors,cors->opt.ntrip_sources_file);
    cors_ntrip_agent_start(&cors->agent,&cors->ntrip,cors->opt.agent_user_file);
    cors_rtcm_decoder_start(&cors->rtcm_decoder,cors);
    cors_pnt_start(&cors->pnt,cors);
    cors_srtk_start(&cors->srtk,cors,NULL,cors->opt.baselines_file);
    cors_nrtk_start(&cors->nrtk,cors);
    cors_vrs_start(&cors->vrs,cors,&cors->nrtk,cors->opt.vstas_file);
    cors_monitor_start(&cors->monitor,cors);
}

static void cors_thread(void *cors_arg)
{
    cors_t *cors=(cors_t*)cors_arg;

    uv_loop_t *loop=uv_loop_new();

    cors->close=calloc(1,sizeof(uv_async_t));
    cors->close->data=cors;
    uv_async_init(loop,cors->close,close_cb);

    cors->timer_stat=calloc(1,sizeof(uv_timer_t));
    uv_timer_init(loop,cors->timer_stat);
    uv_timer_start(cors->timer_stat,on_timer_stat_cb,0,10000);

    start_cors(cors);

    uv_run(loop,UV_RUN_DEFAULT);
    close_uv_loop(loop);
    free(loop);
}

extern int cors_start(cors_t* cors, const cors_opt_t *opt)
{
    cors->opt=*opt;

    if (uv_thread_create(&cors->thread,cors_thread,cors)) {
        log_trace(1,"cors thread create error\n");
        return 0;
    }
#if WIN32
    SetPriorityClass(GetCurrentProcess(),REALTIME_PRIORITY_CLASS);
#else
    struct sched_param param;
    param.sched_priority=sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(getpid(),SCHED_RR,&param);
#endif
    cors->state=1;
    log_trace(1,"cors thread create ok\n");
    return 1;
}

static void free_cors(cors_t *cors)
{
    cors_freenav(&cors->nav);
    cors_freeobs(&cors->obs);
    cors_freeblsol(&cors->blsols);
    cors_freessat(&cors->ssats);
    cors_freesta(&cors->stas);
}

extern void cors_close(cors_t *cors)
{
    cors_ntrip_close(&cors->ntrip);
    cors_rtcm_decoder_close(&cors->rtcm_decoder);
    cors_pnt_close(&cors->pnt);
    cors_monitor_close(&cors->monitor);
    cors_srtk_close(&cors->srtk);
    cors_nrtk_close(&cors->nrtk);
    cors_vrs_close(&cors->vrs);
    cors_ntrip_agent_close(&cors->agent);

    cors->state=0;
    uv_async_send(cors->close);
    uv_thread_join(&cors->thread);
    free_cors(cors);
}

extern void cors_updnav(cors_nav_t *cors_nav, const nav_t *nav, int ephsat, int ephset)
{
    geph_t *geph1,*geph2,*geph3;
    eph_t *eph1,*eph2,*eph3;
    int prn;

    if (satsys(ephsat,&prn)!=SYS_GLO) {
        eph1=nav->eph+ephsat-1+MAXSAT*ephset;
        eph2=cors_nav->data.eph+ephsat-1+MAXSAT*ephset;
        eph3=cors_nav->data.eph+ephsat-1+MAXSAT*(2+ephset);

        if (eph2->ttr.time==0||
            (eph1->iode!=eph3->iode&&eph1->iode!=eph2->iode)||
            (timediff(eph1->toe,eph3->toe)!=0.0&&
             timediff(eph1->toe,eph2->toe)!=0.0)||
            (timediff(eph1->toc,eph3->toc)!=0.0&&
             timediff(eph1->toc,eph2->toc)!=0.0)) {
            *eph3=*eph2;
            *eph2=*eph1;
        }
    }
    else {
        cors_nav->data.glo_fcn[prn-1]=nav->glo_fcn[prn-1];
        geph1=nav->geph+prn-1;
        geph2=cors_nav->data.geph+prn-1;
        geph3=cors_nav->data.geph+prn-1+MAXPRNGLO;
        if (geph2->tof.time==0||
            (geph1->iode!=geph3->iode&&geph1->iode!=geph2->iode)) {
            *geph3=*geph2;
            *geph2=*geph1;
        }
    }
}

static cors_obsd_t* new_cors_obsd(cors_obsd_t **tbl, int srcid)
{
    cors_obsd_t *s=calloc(1,sizeof(cors_obsd_t));
    s->obs.data=malloc(MAXOBS*sizeof(obsd_t));
    s->srcid=srcid;
    HASH_ADD_INT(*tbl,srcid,s);
    return s;
}

extern void cors_updobs(cors_obs_t *cors_obs, const obsd_t *obsd, int n, int srcid)
{
    if (n<=0) return;

    cors_obsd_t *s;
    HASH_FIND_INT(cors_obs->data,&srcid,s);
    if (!s&&!(s=new_cors_obsd(&cors_obs->data,srcid))) {
        return;
    }
    memcpy(s->obs.data,obsd,sizeof(obsd_t)*n);
    s->obs.n=n;
}

static cors_ssat_t* new_cors_ssat(cors_ssat_t **tbl, int srcid)
{
    cors_ssat_t *s=calloc(1,sizeof(cors_ssat_t));
    s->srcid=srcid;
    HASH_ADD_INT(*tbl,srcid,s);
    return s;
}

extern void cors_updssat(cors_ssats_t *cors_ssat, ssat_t *ssat, int srcid, int upd_flag, gtime_t time)
{
    cors_ssat_t *s;
    HASH_FIND_INT(cors_ssat->data,&srcid,s);
    if (!s&&!(s=new_cors_ssat(&cors_ssat->data,srcid))) {
        return;
    }
    if (upd_flag==1) {
        if (fabs(timediff(time,s->time))<1E-2) return;
        memcpy(s->ssat,ssat,sizeof(ssat_t)*MAXSAT);
        s->time=time;
    }
    else if (upd_flag==2) {
        memcpy(s->ssat,ssat,sizeof(ssat_t)*MAXSAT);
        s->time=time;
    }
}

static cors_sta_t* new_cors_sta(cors_sta_t **tbl, int srcid)
{
    cors_sta_t *s=calloc(1,sizeof(cors_ssat_t));
    s->srcid=srcid;
    HASH_ADD_INT(*tbl,srcid,s);
    return s;
}

extern void cors_updsta(cors_stas_t *stas, const sta_t *sta, int srcid)
{
    cors_sta_t *s;
    HASH_FIND_INT(stas->data,&srcid,s);
    if (!s&&!(s=new_cors_sta(&stas->data,srcid))) {
        return;
    }
    s->sta=*sta;
}

extern void cors_freenav(cors_nav_t *nav)
{
    freenav(&nav->data,0xFF);
}

extern void cors_freeobs(cors_obs_t *obs)
{
    cors_obsd_t *o,*t;
    HASH_ITER(hh,obs->data,o,t) {
        HASH_DEL(obs->data,o);
        freeobs(&o->obs);
    }
}

extern void cors_freessat(cors_ssats_t *ssat)
{
    cors_ssat_t *s,*t;
    HASH_ITER(hh,ssat->data,s,t) HASH_DEL(ssat->data,s);
}

extern void cors_freesta(cors_stas_t *stas)
{
    cors_sta_t *s,*t;
    HASH_ITER(hh,stas->data,s,t) HASH_DEL(stas->data,s);
}

extern int cors_initnav(cors_nav_t *nav)
{
    eph_t  eph0 ={0,-1,-1};
    geph_t geph0={0,-1};
    seph_t seph0={0};
    int i;

    memset(&nav->data, 0, sizeof(nav_t));
    if (!(nav->data.eph =(eph_t *)malloc(sizeof(eph_t )*MAXSAT*4))||
        !(nav->data.geph=(geph_t*)malloc(sizeof(geph_t)*NSATGLO*2))||
        !(nav->data.seph=(seph_t*)malloc(sizeof(seph_t)*NSATSBS*2))) {
        log_trace(1,"cors_initnav: malloc error\n");
        return 0;
    }
    for (i=0;i< MAXSAT*4;i++) nav->data.eph [i]=eph0;
    for (i=0;i<NSATGLO*2;i++) nav->data.geph[i]=geph0;
    for (i=0;i<NSATSBS*2;i++) nav->data.seph[i]=seph0;
    nav->data.n =MAXSAT *2;
    nav->data.ng=NSATGLO*2;
    nav->data.ns=NSATSBS*2;
    return 1;
}

extern void cors_initobs(cors_obs_t *obs)
{
    obs->data=NULL;
}

extern void cors_initssat(cors_ssats_t *ssats)
{
    ssats->data=NULL;
}

extern void cors_initsta(cors_stas_t *stas)
{
    stas->data=NULL;
}

extern void cors_add_source(cors_t *cors, const char *name, const char *addr, int port, const char *user, const char *passwd,
                            const char *mntpnt, const double *pos)
{
    cors_ntrip_source_info_t *info=calloc(1,sizeof(*info));
    strcpy(info->passwd,passwd);
    strcpy(info->mntpnt,mntpnt);
    strcpy(info->name,name);
    strcpy(info->user,user);
    strcpy(info->addr,addr);
    info->port=port;
    matcpy(info->pos,pos,1,3);

    cors_ntrip_add_source(&cors->ntrip,info);
    cors_nrtk_add_source(&cors->nrtk,info->ID,
            info->pos);
}

extern void cors_del_source(cors_t *cors, const char *name)
{
    cors_ntrip_source_info_t *s;
    HASH_FIND_STR(cors->ntrip.info_tbl[0],name,s);
    if (!s) return;
    cors_nrtk_del_source(&cors->nrtk,s->ID);
    cors_ntrip_del_source(&cors->ntrip,name);
}

extern void cors_add_baseline(cors_t *cors, const char *base, const char *rover)
{
    cors_ntrip_source_info_t *r,*b;
    cors_srtk_t *srtk=&cors->srtk;
    cors_ntrip_source_info_t *info_tbl=cors->ntrip.info_tbl[0];

    HASH_FIND_STR(info_tbl,rover,r);
    HASH_FIND_STR(info_tbl,base,b);
    if (!r||!b) return;
    cors_srtk_add_baseline(srtk,b->ID,r->ID);
}

extern void cors_del_baseline(cors_t *cors, const char *base, const char *rover)
{
    cors_ntrip_source_info_t *r,*b;
    cors_srtk_t *srtk=&cors->srtk;
    cors_ntrip_source_info_t *info_tbl=cors->ntrip.info_tbl[0];

    HASH_FIND_STR(info_tbl,rover,r);
    HASH_FIND_STR(info_tbl,base,b);
    if (!r||!b) return;
    cors_srtk_del_baseline(srtk,b->ID,r->ID);
}

extern void cors_basenet_del_baseline(cors_t *cors, const char *base, const char *rover)
{
    cors_ntrip_source_info_t *r,*b;
    cors_nrtk_t *nrtk=&cors->nrtk;
    cors_ntrip_source_info_t *info_tbl=cors->ntrip.info_tbl[0];

    if (!nrtk->state) return;

    HASH_FIND_STR(info_tbl,rover,r);
    HASH_FIND_STR(info_tbl,base,b);
    if (!r||!b) return;
    return cors_nrtk_del_baseline(nrtk,b->ID,r->ID);
}

extern void cors_basenet_add_baseline(cors_t *cors, const char *base, const char *rover)
{
    cors_ntrip_source_info_t *r,*b;
    cors_nrtk_t *nrtk=&cors->nrtk;
    cors_ntrip_source_info_t *info_tbl=cors->ntrip.info_tbl[0];

    if (!nrtk->state) return;

    HASH_FIND_STR(info_tbl,rover,r);
    HASH_FIND_STR(info_tbl,base,b);
    if (!r||!b) return;
    return cors_nrtk_add_baseline(nrtk,b->ID,r->ID);
}

static cors_blsol_t* new_cors_blsol(cors_blsol_t **tbl, cors_baseline_t *bl, int base_srcid, int rover_srcid)
{
    char id[16];
    sprintf(id,"%d->%d",base_srcid,rover_srcid);

    cors_blsol_t *blsol=calloc(1,sizeof((*blsol)));
    blsol->base_srcid=base_srcid;
    blsol->rover_srcid=rover_srcid;
    bl->sol=blsol;
    strcpy(blsol->id,id);
    HASH_ADD_STR(*tbl,id,blsol);
    return blsol;
}

extern void cors_updblsol(cors_blsols_t *blsols, cors_baseline_t *bl, const rtk_t *rtk, int base_srcid, int rover_srcid)
{
    cors_blsol_t *blsol;
    char id[16];

    sprintf(id,"%d->%d",base_srcid,rover_srcid);
    HASH_FIND_STR(blsols->data,id,blsol);

    if (!blsol&&!(blsol=new_cors_blsol(&blsols->data,bl,base_srcid,rover_srcid))) {
        return;
    }
    blsol->rtk=*rtk;
}

extern void cors_freeblsol(cors_blsols_t *blsols)
{
    cors_blsol_t *blsol,*tmp;
    HASH_ITER(hh,blsols->data,blsol,tmp) {
        HASH_DEL(blsols->data,blsol);
        free(blsol);
    }
}

