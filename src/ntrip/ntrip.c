/*------------------------------------------------------------------------------
 * ntrip.c : NTRIP functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

#define MAX_SRCS  1024

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

static int generate_source_id()
{
    static int mID=0;
    return ++mID;
}

static int read_sources_file(cors_ntrip_t *ntrip, const char *file)
{
    FILE *fp;
    if (!(fp=fopen(file,"r"))) {
        log_trace(2,"read sourcec file fail: %s\n",file);
        return -1;
    }
    cors_ntrip_source_info_t *info_name,*info_id;
    char buff[256],*p,*q,*val[16];
    double pos[3];
    int n;

    while (fgets(buff,sizeof(buff),fp)) {
        for (n=0,p=buff;*p&&n<16;p=q+1) {
            if ((q=strchr(p,','))||(q=strchr(p,'#'))) {val[n++]=p; *q='\0';}
            else break;
        }
        if (n<6) continue;
        HASH_FIND_STR(ntrip->info_tbl[0],val[0],info_name);
        if (info_name) continue;

        info_name=calloc(1,sizeof(*info_name));
        strcpy(info_name->name,val[0]);
        strcpy(info_name->addr,val[1]);
        strcpy(info_name->user,val[3]);
        strcpy(info_name->passwd,val[4]);
        strcpy(info_name->mntpnt,val[5]);
        info_name->port=atoi(val[2]);
        info_name->ID=generate_source_id();

        if (n>=9) {
            pos[0]=atof(val[6])*D2R;
            pos[1]=atof(val[7])*D2R;
            pos[2]=atof(val[8]);
            pos2ecef(pos,info_name->pos);
        }
        info_id=calloc(1,sizeof(*info_id));
        *info_id=*info_name;

        HASH_ADD(hh,ntrip->info_tbl[0],name,strlen(info_name->name),info_name);
        HASH_ADD(ii,ntrip->info_tbl[1],ID,sizeof(int),info_id);
        kd_insert(ntrip->src_kdtree,info_name->pos,info_name);
    }
    fclose(fp);
    return HASH_COUNT(ntrip->info_tbl[0]);
}

static void start_ntrip(cors_ntrip_t *ntrip)
{
    int nc=ceil((double)HASH_COUNT(ntrip->info_tbl[0])/MAX_SRCS);
    int i=0;

    cors_ntrip_source_info_t **itbl=calloc(nc,sizeof(*itbl));
    cors_ntrip_source_info_t *info,*tmp,*s;

    HASH_ITER(hh,ntrip->info_tbl[0],info,tmp) {
        s=calloc(1,sizeof(*s));
        *s=*info;
        HASH_ADD_STR(itbl[i/MAX_SRCS],name,s);
        i++;
    }
    for (i=0;i<nc;i++) {
        cors_ntrip_caster_t *ctr=calloc(1,sizeof(cors_ntrip_caster_t));
        ctr->ntrip=ntrip;
        ctr->ID=i+1;
        cors_ntrip_caster_start(ctr,itbl[i]);
        HASH_ADD_INT(ntrip->ctr_tbl,ID,ctr);
    }
    ntrip->state=1;
    log_trace(3,"ntrip startup ok\n");
}

static void free_ntrip(cors_ntrip_t *ntrip)
{
    cors_ntrip_caster_t *c,*t;
    HASH_ITER(hh,ntrip->ctr_tbl,c,t) {
        HASH_DEL(ntrip->ctr_tbl,c);
        free(c);
    }
    cors_ntrip_source_info_t *i,*itmp;
    HASH_ITER(hh,ntrip->info_tbl[0],i,itmp) {
        HASH_DELETE(hh,ntrip->info_tbl[0],i);
        free(i);
    }
    HASH_ITER(ii,ntrip->info_tbl[1],i,itmp) HASH_DELETE(ii,ntrip->info_tbl[1],i);
    kd_free(ntrip->src_kdtree);
}

static void on_timer_stat_cb(uv_timer_t* handle)
{
    NULL;
}

static void close_cb(uv_async_t* handle)
{
    if (uv_loop_alive(handle->loop)) {
        uv_stop(handle->loop);
    }
}

static void ntrip_thread(void *ntrip_arg)
{
    cors_ntrip_t *ntrip=ntrip_arg;
    ntrip->timer_stat=calloc(1,sizeof(uv_timer_t));

    uv_loop_t *loop=uv_loop_new();

    uv_timer_init(loop,ntrip->timer_stat);
    uv_timer_start(ntrip->timer_stat,on_timer_stat_cb,0,10000);

    ntrip->close=calloc(1,sizeof(uv_async_t));
    ntrip->close->data=ntrip;
    uv_async_init(loop,ntrip->close,close_cb);

    start_ntrip(ntrip);
    set_thread_rt_priority();

    uv_run(loop,UV_RUN_DEFAULT);
    close_uv_loop(loop);
    free(loop);
}

extern int cors_ntrip_start(cors_ntrip_t *ntrip, cors_t *cors, const char *sources_file)
{
    ntrip->src_kdtree=kd_create(3);
    ntrip->cors=cors;
    read_sources_file(ntrip,sources_file);

    if (uv_thread_create(&ntrip->thread,ntrip_thread,ntrip)) {
        log_trace(1,"ntrip thread create error\n");
        return 0;
    }
    log_trace(1,"ntrip thread create ok\n");
    return 1;
}

extern void cors_ntrip_close(cors_ntrip_t *ntrip)
{
    if (!ntrip->close) return;
    uv_async_send(ntrip->close);

    uv_thread_join(&ntrip->thread);
    cors_ntrip_caster_t *c,*t;
    HASH_ITER(hh,ntrip->ctr_tbl,c,t) cors_ntrip_caster_close(c);
    free_ntrip(ntrip);
}

static int ntrip_add_source_prc(cors_ntrip_t *ntrip, cors_ntrip_source_info_t *info)
{
    cors_ntrip_caster_t *c,*t,*s=NULL;

    HASH_ITER(hh,ntrip->ctr_tbl,c,t) {
        if (HASH_COUNT(c->src_tbl)>=MAX_SRCS) continue;
        s=c;
        break;
    }
    info->ID=generate_source_id();

    if (s) {
        return cors_ntrip_caster_add_source(s,info);
    }
    cors_ntrip_caster_t *ctr=calloc(1,sizeof(*ctr));
    ctr->ntrip=ntrip;
    ctr->ID=HASH_COUNT(ntrip->ctr_tbl);

    if (cors_ntrip_caster_start(ctr,info)) {
        HASH_ADD_INT(ntrip->ctr_tbl,ID,ctr);
    }
    return 1;
}

extern void cors_ntrip_add_source(cors_ntrip_t *ntrip, cors_ntrip_source_info_t *info)
{
    cors_ntrip_source_info_t *s;

    HASH_FIND_STR(ntrip->info_tbl[0],info->name,s);
    if (s) return;
    ntrip_add_source_prc(ntrip,info);
}

extern void cors_ntrip_del_source(cors_ntrip_t *ntrip, const char *name)
{
    cors_ntrip_caster_t *c,*t;
    HASH_ITER(hh,ntrip->ctr_tbl,c,t) {
        cors_ntrip_caster_del_source(c,name);
    }
}

extern void cors_ntrip_source_updpos(cors_ntrip_t *ntrip, const double *pos, int srcid)
{
    cors_ntrip_source_info_t *s;
    HASH_FIND(ii,ntrip->info_tbl[1],&srcid,sizeof(int),s);
    if (!s) return;
    matcpy(s->pos,pos,1,3);
}
