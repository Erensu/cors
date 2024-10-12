/*------------------------------------------------------------------------------
 * nrtk.c  : GNSS network RTK functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

#define NRTK_STRICT_TIME_SYNC     1
#define NRTK_STRICT_BSTA_SYNC     1
#define NRTK_STRICT_TRIG_SYNC     1
#define NRTK_WAIT_SYNC            1
#define NTRK_MAX_BLS    64

extern int  vrs_add_vsta(cors_vrs_t *vrs, const char *name, const double *pos);
extern int  vrs_del_vsta(cors_vrs_t *vrs, const char *name);
extern void vrs_upd_vsta(cors_vrs_t *vrs);

typedef struct nrtk_add_source {
    int srcid;
    double pos[3];
    QUEUE q;
} nrtk_add_source_t;

typedef struct nrtk_del_source {
    int srcid;
    QUEUE q;
} nrtk_del_source_t;

typedef struct nrtk_add_vsta {
    double pos[3];
    char name[32];
    QUEUE q;
} nrtk_add_vsta_t;

typedef struct nrtk_del_vsta {
    char name[32];
    QUEUE q;
} nrtk_del_vsta_t;

typedef struct nrtk_add_bl {
    int base_srcid;
    int rover_srcid;
    QUEUE q;
} nrtk_add_bl_t;

typedef struct nrtk_del_bl {
    int base_srcid;
    int rover_srcid;
    QUEUE q;
} nrtk_del_bl_t;

typedef struct nrtk_upd_bl {
    int base_srcid;
    int rover_srcid;
    cors_baseline_t *bl;
    QUEUE q;
} nrtk_upd_bl_t;

static void set_thread_rt_priority()
{
#if WIN32
    SetThreadPriority(GetCurrentThread(),THREAD_PRIORITY_TIME_CRITICAL);
#else
    struct sched_param param;
    param.sched_priority=sched_get_priority_max(SCHED_FIFO);
    sched_setscheduler(getpid(),SCHED_RR,&param);
    pthread_setschedparam(pthread_self(),SCHED_FIFO,&param);
#endif
}

static void nrtk_del_baseline(cors_nrtk_t *nrtk, int base_srcid, int rover_srcid)
{
    cors_srtk_t *s,*t;
    HASH_ITER(hh,nrtk->srtk,s,t) {
        cors_srtk_del_baseline(s,base_srcid,rover_srcid);
    }
}

static cors_srtk_t* new_srtk(cors_nrtk_t *nrtk)
{
    cors_srtk_t *srtk=calloc(1,sizeof(*srtk));
    srtk->ID=HASH_COUNT(nrtk->srtk);

    if (!cors_srtk_start(srtk,nrtk->cors,nrtk,"")) {
        free(srtk);
        return NULL;
    }
    HASH_ADD_INT(nrtk->srtk,ID,srtk);
    while (!srtk->state) uv_sleep(5);
    return srtk;
}

static void nrtk_add_baseline(cors_nrtk_t *nrtk, int base_srcid, int rover_srcid)
{
    cors_srtk_t *s,*t,*p=NULL;

    HASH_ITER(hh,nrtk->srtk,s,t) {
        if (HASH_COUNT(s->bls.data)>=NTRK_MAX_BLS) continue;
        p=s;
        break;
    }
    if (!p&&!(p=new_srtk(nrtk))) return;
    cors_srtk_add_baseline(p,base_srcid,rover_srcid);
}

static void nrtk_upd_bls(cors_nrtk_t *nrtk, cors_dtrig_edge_t **edge_add, cors_dtrig_edge_t **edge_del)
{
    cors_dtrig_edge_t *e,*t;

    HASH_ITER(hh,*edge_add,e,t) {
        nrtk_add_baseline(nrtk,e->vt[0]->srcid,e->vt[1]->srcid);
        HASH_DEL(*edge_add,e);
        free(e);
    }
    HASH_ITER(hh,*edge_del,e,t) {
        nrtk_del_baseline(nrtk,e->vt[0]->srcid,e->vt[1]->srcid);
        HASH_DEL(*edge_del,e);
        free(e);
    }
}

static int nrtk_init_dtrignet(cors_nrtk_t *nrtk)
{
    cors_t *cors=nrtk->cors;
    cors_ntrip_source_info_t *info_tbl=cors->ntrip.info_tbl[1],*info,*t;
    cors_dtrig_net_t *dtg=&nrtk->dtrig_net;
    cors_dtrig_edge_t **edge,*e,*et;
    int i=-1;

    if (HASH_CNT(ii,info_tbl)<=0) {
        return 0;
    }
    cors_dtrignet_init(&nrtk->dtrig_net);

    HASH_ITER(ii,info_tbl,info,t) {
        cors_dtrignet_add_vertex(dtg,info->pos,info->ID,NULL,NULL);
    }
    cors_srtk_t *srtk;
    HASH_ITER(hh,dtg->edges,e,et) {
        if (i<0||i++>=NTRK_MAX_BLS) {
            srtk=new_srtk(nrtk);
            i=0;
        }
        cors_srtk_add_baseline(srtk,e->vt[0]->srcid,e->vt[1]->srcid);
    }
    return HASH_COUNT(dtg->edges);
}

static void nrtk_add_source(cors_nrtk_t *nrtk, nrtk_add_source_t *data)
{
    cors_dtrig_net_t *dtg=&nrtk->dtrig_net;
    cors_dtrig_edge_t *edge_add,*edge_del;
    cors_vrs_t *vrs=&nrtk->cors->vrs;

    cors_dtrignet_add_vertex(dtg,data->pos,data->srcid,&edge_add,&edge_del);
    nrtk_upd_bls(nrtk,&edge_add,&edge_del);
    vrs_upd_vsta(vrs);
    free(data);
}

static void nrtk_del_source(cors_nrtk_t *nrtk, nrtk_del_source_t *data)
{
    cors_dtrig_net_t *dtg=&nrtk->dtrig_net;
    cors_dtrig_edge_t *edge_add,*edge_del;
    cors_vrs_t *vrs=&nrtk->cors->vrs;

    cors_dtrignet_del_vertex(dtg,data->srcid,&edge_add,&edge_del);
    nrtk_upd_bls(nrtk,&edge_add,&edge_del);
    vrs_upd_vsta(vrs);
    free(data);
}

static void nrtk_add_vsta(cors_nrtk_t *nrtk, nrtk_add_vsta_t *data)
{
    cors_vrs_t *vrs=&nrtk->cors->vrs;
    vrs_add_vsta(vrs,data->name,data->pos);
    free(data);
}

static void nrtk_del_vsta(cors_nrtk_t *nrtk, nrtk_del_vsta_t *data)
{
    cors_vrs_t *vrs=&nrtk->cors->vrs;
    vrs_del_vsta(vrs,data->name);
    free(data);
}

static void nrtk_add_bl(cors_nrtk_t *nrtk, nrtk_add_bl_t *data)
{
    cors_dtrig_net_t *dtrignet=&nrtk->dtrig_net;
    cors_dtrignet_add_edge(dtrignet,data->base_srcid,data->rover_srcid);
    nrtk_add_baseline(nrtk,data->base_srcid,data->rover_srcid);
    free(data);
}

static void nrtk_del_bl(cors_nrtk_t *nrtk, nrtk_del_bl_t *data)
{
    cors_dtrig_net_t *dtrignet=&nrtk->dtrig_net;
    cors_srtk_t *s,*t;

    cors_dtrignet_del_edge(dtrignet,data->base_srcid,data->rover_srcid);
    HASH_ITER(hh,nrtk->srtk,s,t) {
        cors_srtk_del_baseline(s,data->base_srcid,data->rover_srcid);
    }
    free(data);
}

static void nrtk_upd_bl(cors_nrtk_t *nrtk, nrtk_upd_bl_t *data)
{
    cors_dtrignet_upd_edge(&nrtk->dtrig_net,data->bl,data->base_srcid,data->rover_srcid);
    free(data);
}

static void nrtk_init(cors_nrtk_t *nrtk)
{
    nrtk_init_dtrignet(nrtk);
    nrtk->state=1;

    QUEUE_INIT(&nrtk->delsrc_queue);
    QUEUE_INIT(&nrtk->addsrc_queue);
    QUEUE_INIT(&nrtk->addvsta_queue);
    QUEUE_INIT(&nrtk->delvsta_queue);
    QUEUE_INIT(&nrtk->addbl_queue);
    QUEUE_INIT(&nrtk->delbl_queue);
    QUEUE_INIT(&nrtk->updbl_queue);

    uv_mutex_init(&nrtk->delsrc_lock);
    uv_mutex_init(&nrtk->addsrc_lock);
    uv_mutex_init(&nrtk->addvsta_lock);
    uv_mutex_init(&nrtk->delvsta_lock);
    uv_mutex_init(&nrtk->addbl_lock);
    uv_mutex_init(&nrtk->delbl_lock);
    uv_mutex_init(&nrtk->updbl_lock);
}

static void do_add_source_work(cors_nrtk_t *nrtk)
{
    while (!QUEUE_EMPTY(&nrtk->addsrc_queue)) {
        uv_mutex_lock(&nrtk->addsrc_lock);

        QUEUE *q=QUEUE_HEAD(&nrtk->addsrc_queue);
        nrtk_add_source_t *data=QUEUE_DATA(q,nrtk_add_source_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&nrtk->addsrc_lock);
        nrtk_add_source(nrtk,data);
    }
}

static void do_del_source_work(cors_nrtk_t *nrtk)
{
    while (!QUEUE_EMPTY(&nrtk->delsrc_queue)) {
        uv_mutex_lock(&nrtk->delsrc_lock);

        QUEUE *q=QUEUE_HEAD(&nrtk->delsrc_queue);
        nrtk_del_source_t *data=QUEUE_DATA(q,nrtk_del_source_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&nrtk->delsrc_lock);
        nrtk_del_source(nrtk,data);
    }
}

static void do_add_vsta_work(cors_nrtk_t *nrtk)
{
    while (!QUEUE_EMPTY(&nrtk->addvsta_queue)) {
        uv_mutex_lock(&nrtk->addvsta_lock);

        QUEUE *q=QUEUE_HEAD(&nrtk->addvsta_queue);
        nrtk_add_vsta_t *data=QUEUE_DATA(q,nrtk_add_vsta_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&nrtk->addvsta_lock);
        nrtk_add_vsta(nrtk,data);
    }
}

static void do_del_vsta_work(cors_nrtk_t *nrtk)
{
    while (!QUEUE_EMPTY(&nrtk->delvsta_queue)) {
        uv_mutex_lock(&nrtk->delvsta_lock);

        QUEUE *q=QUEUE_HEAD(&nrtk->delvsta_queue);
        nrtk_del_vsta_t *data=QUEUE_DATA(q,nrtk_del_vsta_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&nrtk->delvsta_lock);
        nrtk_del_vsta(nrtk,data);
    }
}

static void do_add_bl_work(cors_nrtk_t *nrtk)
{
    while (!QUEUE_EMPTY(&nrtk->addbl_queue)) {
        uv_mutex_lock(&nrtk->addbl_lock);

        QUEUE *q=QUEUE_HEAD(&nrtk->addbl_queue);
        nrtk_add_bl_t *data=QUEUE_DATA(q,nrtk_add_bl_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&nrtk->addbl_lock);
        nrtk_add_bl(nrtk,data);
    }
}

static void do_del_bl_work(cors_nrtk_t *nrtk)
{
    while (!QUEUE_EMPTY(&nrtk->delbl_queue)) {
        uv_mutex_lock(&nrtk->delbl_lock);

        QUEUE *q=QUEUE_HEAD(&nrtk->delbl_queue);
        nrtk_del_bl_t *data=QUEUE_DATA(q,nrtk_del_bl_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&nrtk->delbl_lock);
        nrtk_del_bl(nrtk,data);
    }
}

static void do_upd_bl_work(cors_nrtk_t *nrtk)
{
    while (!QUEUE_EMPTY(&nrtk->updbl_queue)) {
        uv_mutex_lock(&nrtk->updbl_lock);

        QUEUE *q=QUEUE_HEAD(&nrtk->updbl_queue);
        nrtk_upd_bl_t *data=QUEUE_DATA(q,nrtk_upd_bl_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&nrtk->updbl_lock);
        nrtk_upd_bl(nrtk,data);
    }
}

static int subnet_time_sync(cors_nrtk_t *nrtk, cors_master_sta_t *vt, rtk_t **rtk, obs_t **mobs)
{
    gtime_t time_cur={0};
    cors_dtrig_edge_q_t *eq,*tt;
    cors_blsol_t *blsol;
    int i,j=0,sync=0;

    cors_obsd_t *d;
    HASH_FIND_INT(nrtk->cors->obs.data,&vt->srcid,d);
    if (!d||d->obs.n<=0||fabs(timediff(d->obs.data[0].time,vt->time))<1E-2) {
        return 0;
    }
    time_cur=d->obs.data[0].time;
    *mobs=&d->obs;
    for (i=0;i<HASH_COUNT(vt->edge_list);i++) rtk[i]=NULL;

    HASH_ITER(hh,vt->edge_list,eq,tt) {
        if (!eq->edge->bl||!(blsol=eq->edge->bl->sol)) {j++; continue;}
        rtk[j++]=&blsol->rtk;
    }
#if NRTK_STRICT_TIME_SYNC
    static double age=1E-2;
#else
    static double age=15.0;
#endif
    for (i=0;i<HASH_COUNT(vt->edge_list);i++) {
        if (rtk[i]&&fabs(timediff(time_cur,rtk[i]->time))<age) {
            sync++;
        }
    }
#if NRTK_STRICT_BSTA_SYNC
    if (sync==HASH_COUNT(vt->edge_list)) return 1;
#else
#if NRTK_WAIT_SYNC
    static double mt=100.0;
    static double st=0.2;
    if (sync&&++vt->wt>st*1E9/mt) return 1;
#else
    if (sync) return 1;
#endif
#endif
    return 0;
}

static int upd_dtrig_time(cors_dtrig_t *dtrig, rtk_t **rtk)
{
    int i;
    for (i=0;i<3;i++) {
        if (rtk[i]&&timediff(rtk[i]->time,dtrig->time)>1E-2) dtrig->time=rtk[i]->time;
    }
    return dtrig->time.time>0;
}

static int dtrig_time_sync(cors_nrtk_t *nrtk, cors_dtrig_t *dtrig, rtk_t **rtk)
{
#if NRTK_STRICT_TRIG_SYNC
    static double age=1E-2;
#else
    static double age=15.0;
#endif
    gtime_t time_cur={0};
    cors_blsol_t *blsol;
    int i,sync=0;

    for (i=0;i<3;i++) rtk[i]=NULL;
    for (i=0;i<3;i++) {
        if (!dtrig->edge[i]||!dtrig->edge[i]->bl||!(blsol=dtrig->edge[i]->bl->sol)) continue;
        rtk[i]=&blsol->rtk;
        if (!time_cur.time||timediff(rtk[i]->time,time_cur)>1E-2) {
            time_cur=rtk[i]->time;
        }
    }
    if (fabs(timediff(time_cur,dtrig->time))<1E-2) {
        return 0;
    }
    if (!dtrig->time.time) {
        if (!upd_dtrig_time(dtrig,rtk)) return -1;
    }
    for (i=0;i<3;i++) {
        if (rtk[i]&&fabs(timediff(time_cur,rtk[i]->time))<age) {
            sync++;
        }
    }
#if NRTK_STRICT_BSTA_SYNC
    if (sync==3) return 1;
#else
#if NRTK_WAIT_SYNC
    static double mt=100.0;
    static double st=0.2;
    if (sync&&++dtrig->wt>st*1E9/mt) return 1;
#else
    if (sync) return 1;
#endif
#endif
    return 0;
}

static void upd_dtrig_stat(cors_dtrig_t *dtrig, rtk_t **rtk)
{
    upd_dtrig_time(dtrig,rtk);
    dtrig->wt=0;
}

static void upd_vrs(cors_vrs_t *vrs, cors_master_sta_t *msta, const obs_t *mobs, const rtk_t **rtk, int n)
{
    cors_vrs_sta_q_t *vs,*vq;
    HASH_ITER(hh,msta->vsta_list,vs,vq) {
        cors_vrs_upd(vrs,vs->vsta,msta,mobs,rtk,n);
    }
}

static void upd_subnet_stat(cors_dtrig_vertex_t *vt, rtk_t **rtk, const obs_t *mobs)
{
    vt->time=mobs->data[0].time;
    vt->wt=0;
}

static void upd_amb_closure(cors_dtrig_t *dtrig, rtk_t **rtk, int sat, int f)
{
    if (!rtk[0]||!rtk[0]->ssat[sat].fix[f]) return;
    if (!rtk[1]||!rtk[1]->ssat[sat].fix[f]) return;
    if (!rtk[2]||!rtk[2]->ssat[sat].fix[f]) return;

    if (rtk[0]->ssat[sat].refsat[f]!=rtk[1]->ssat[sat].refsat[f]) return;
    if (rtk[0]->ssat[sat].refsat[f]!=rtk[2]->ssat[sat].refsat[f]) return;
    if (rtk[0]->ssat[sat].refsat[f]==sat+1) return;

    char tbuf[32];
    double dd[3],dire;
    int i;

    for (i=0;i<3;i++) {
        if (strcmp(dtrig->edge[i]->id,dtrig->edge[i]->bl->id)) dire=-1.0;
        else dire=1.0;
        dd[i]=dire*rtk[i]->ssat[sat].dd[f];
    }
    time2str(dtrig->time,tbuf,3);
    log_trace(3,"%12s: sat=%4d f=%d stat=[%d %d %d] time=%s amb=[%8.2lf %8.2lf %8.2lf] closure=%6.2lf\n",dtrig->id,
            sat+1,f,rtk[0]->sol.stat,rtk[1]->sol.stat,rtk[2]->sol.stat,tbuf,
            dd[0],dd[1],dd[2],dd[0]+dd[1]+dd[2]);
}

static void chk_dtrig_amb_closure(cors_dtrig_t *dtrig, rtk_t **rtk)
{
    char tbuf[32];
    int i,f;

    time2str(dtrig->time,tbuf,3);
    log_trace(1,"%12s trig time=%s\n",dtrig->id,tbuf);

    for (i=0;i<MAXSAT;i++) {
        for (f=0;f<NFREQ;f++) upd_amb_closure(dtrig,rtk,i,f);
    }
}

static void out_subnet_stat(cors_dtrig_vertex_t *vts, cors_dtrig_vertex_t *vt, rtk_t **rtk)
{
    cors_dtrig_edge_q_t *e,*q;
    cors_dtrig_vertex_t *eq;
    char tbuf[64];
    double dr[3],dd[3];
    int i,j=0;

    HASH_ITER(hh,vt->edge_list,e,q) {
        HASH_FIND_INT(vts,&e->edge->bl->rover_srcid,eq);
        if (!rtk[j]) {
            j++; continue;
        }
        for (i=0;i<3;i++) {
            dd[i]=rtk[j]->sol.rr[i]?rtk[j]->sol.rr[i]-eq->pos[i]:0.0;
            dr[i]=rtk[j]->rb[i]?rtk[j]->rb[i]-rtk[j]->sol.rr[i]:0.0;
        }
        time2str(rtk[j]->time,tbuf,3);
        log_trace(1,"    %4d->%4d(%8.3lf) stat=%d time=%s [%6.3lf %6.3lf %6.3lf]\n",e->edge->vt[0]->srcid,e->edge->vt[1]->srcid,
                norm(dr,3)/1000.0,rtk[j]->sol.stat,tbuf,dd[0],dd[1],dd[2]);
        j++;
    }
}

static void do_subnet_work(cors_nrtk_t *nrtk)
{
    cors_dtrig_vertex_t *vt,*tt;
    cors_dtrig_t *dg,*dt;
    cors_dtrig_vertex_t *vts=nrtk->dtrig_net.vertexs;
    cors_vrs_t *vrs=&nrtk->cors->vrs;
    obs_t *mobs;
    rtk_t *rtk[64];

    HASH_ITER(hh,vts,vt,tt) {
        if (subnet_time_sync(nrtk,vt,rtk,&mobs)<=0) continue;
        upd_subnet_stat(vt,rtk,mobs);
        upd_vrs(vrs,vt,mobs,rtk,HASH_COUNT(vt->edge_list));
        out_subnet_stat(vts,vt,rtk);
    }
    HASH_ITER(hh,nrtk->dtrig_net.dtrigs,dg,dt) {
        if (dtrig_time_sync(nrtk,dg,rtk)<=0) continue;
        chk_dtrig_amb_closure(dg,rtk);
        upd_dtrig_stat(dg,rtk);
    }
}

static void free_nrtk(cors_nrtk_t *nrtk)
{
    cors_srtk_t *s,*t;
    HASH_ITER(hh,nrtk->srtk,s,t) cors_srtk_close(s);
    cors_dtrignet_free(&nrtk->dtrig_net);
}

static void nrtk_thread(void *nrtk_arg)
{
    cors_nrtk_t *nrtk=(cors_nrtk_t*)nrtk_arg;
    set_thread_rt_priority();

    while (nrtk->state) {
        do_subnet_work    (nrtk);
        do_add_source_work(nrtk);
        do_del_source_work(nrtk);
        do_add_bl_work    (nrtk);
        do_del_bl_work    (nrtk);
        do_upd_bl_work    (nrtk);
        do_add_vsta_work  (nrtk);
        do_del_vsta_work  (nrtk);
    }
    free_nrtk(nrtk);
}

extern int cors_nrtk_start(cors_nrtk_t *nrtk, cors_t *cors)
{
    nrtk->cors=cors;
    nrtk_init(nrtk);

    if (uv_thread_create(&nrtk->thread,nrtk_thread,nrtk)) {
        log_trace(1,"start nrtk thread fail\n");
        return 0;
    }
    log_trace(1,"start nrtk thread ok\n");
    return 1;
}

extern void cors_nrtk_close(cors_nrtk_t *nrtk)
{
    nrtk->state=0;
    uv_thread_join(&nrtk->thread);
}

extern void cors_nrtk_add_source(cors_nrtk_t *nrtk, int srcid, const double *pos)
{
    if (nrtk->state<=0) return;
    if (norm(pos,3)<=0) return;

    uv_mutex_lock(&nrtk->addsrc_lock);
    nrtk_add_source_t *data=calloc(1,sizeof(*data));
    data->srcid=srcid;
    matcpy(data->pos,pos,1,3);
    QUEUE_INSERT_TAIL(&nrtk->addsrc_queue,&data->q);
    uv_mutex_unlock(&nrtk->addsrc_lock);
}

extern void cors_nrtk_del_source(cors_nrtk_t *nrtk, int srcid)
{
    if (nrtk->state<=0) return;

    uv_mutex_lock(&nrtk->delsrc_lock);
    nrtk_del_source_t *data=calloc(1,sizeof(*data));
    data->srcid=srcid;
    QUEUE_INSERT_TAIL(&nrtk->delsrc_queue,&data->q);
    uv_mutex_unlock(&nrtk->delsrc_lock);
}

extern void cors_nrtk_del_baseline(cors_nrtk_t *nrtk, int base_srcid, int rover_srcid)
{
    if (base_srcid<=0||rover_srcid<=0) return;
    if (!nrtk) return;

    uv_mutex_lock(&nrtk->delbl_lock);
    nrtk_del_bl_t *data=calloc(1,sizeof(*data));
    data->base_srcid=base_srcid;
    data->rover_srcid=rover_srcid;

    QUEUE_INSERT_TAIL(&nrtk->delbl_queue,&data->q);
    uv_mutex_unlock(&nrtk->delbl_lock);
}

extern void cors_nrtk_upd_baseline(cors_nrtk_t *nrtk, cors_baseline_t *bl, int base_srcid, int rover_srcid)
{
    if (base_srcid<=0||rover_srcid<=0) return;
    if (!nrtk) return;

    uv_mutex_lock(&nrtk->updbl_lock);
    nrtk_upd_bl_t *data=calloc(1,sizeof(*data));
    data->base_srcid=base_srcid;
    data->rover_srcid=rover_srcid;
    data->bl=bl;

    QUEUE_INSERT_TAIL(&nrtk->updbl_queue,&data->q);
    uv_mutex_unlock(&nrtk->updbl_lock);
}

extern void cors_nrtk_add_baseline(cors_nrtk_t *nrtk, int base_srcid, int rover_srcid)
{
    if (base_srcid<=0||rover_srcid<=0) return;
    if (!nrtk) return;

    uv_mutex_lock(&nrtk->addbl_lock);
    nrtk_add_bl_t *data=calloc(1,sizeof(*data));
    data->base_srcid=base_srcid;
    data->rover_srcid=rover_srcid;

    QUEUE_INSERT_TAIL(&nrtk->addbl_queue,&data->q);
    uv_mutex_unlock(&nrtk->addbl_lock);
}

extern void cors_nrtk_add_vsta(cors_nrtk_t *nrtk, const char *name, const double *pos)
{
    if (nrtk->state<=0) return;
    if (norm(pos,3)<=0.0||!strcmp(name,"")) return;

    uv_mutex_lock(&nrtk->addvsta_lock);
    nrtk_add_vsta_t *data=calloc(1,sizeof(*data));
    strcpy(data->name,name);
    matcpy(data->pos,pos,1,3);

    QUEUE_INSERT_TAIL(&nrtk->addvsta_queue,&data->q);
    uv_mutex_unlock(&nrtk->addvsta_lock);
}

extern void cors_nrtk_del_vsta(cors_nrtk_t *nrtk, const char *name)
{
    if (nrtk->state<=0||!strcmp(name,"")) return;

    uv_mutex_lock(&nrtk->delvsta_lock);
    nrtk_del_vsta_t *data=calloc(1,sizeof(*data));
    strcpy(data->name,name);

    QUEUE_INSERT_TAIL(&nrtk->delvsta_queue,&data->q);
    uv_mutex_unlock(&nrtk->delvsta_lock);
}

