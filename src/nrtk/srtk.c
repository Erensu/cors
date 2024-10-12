/*------------------------------------------------------------------------------
 * srtk.c  : GNSS RTK functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"
#include "options.h"

#define SRTK_STRICT_TIME_SYNC    1
#define SRTK_STRICT_BSTA_SYNC    1
#define SRTK_SYNC_WAIT           1
#define SRTK_SYMMETRY_MODE       1

typedef struct add_baseline {
    int base_srcid,rover_srcid;
    cors_srtk_t *srtk;
    QUEUE q;
} add_baseline_t;

typedef struct del_baseline {
    cors_baseline_t *bl;
    cors_srtk_t *srtk;
    QUEUE q;
} del_baseline_t;

typedef struct rtkpos_task {
    cors_baseline_t *bl;
    cors_srtk_t *srtk;
    obs_t *robs,*bobs;
    QUEUE q;
} rtkpos_task_t;

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

static rtkpos_task_t* new_rtkpos_task(cors_srtk_t *srtk, cors_baseline_t *bl,obs_t *robs, obs_t *bobs)
{
    rtkpos_task_t *task=calloc(1,sizeof(*task));
    task->srtk=srtk;
    task->bl=bl;
    task->bl->on++;
    task->robs=robs;
    task->bobs=bobs;
    return task;
}

static void upd_obs(const cors_obsd_t *obs, obs_t **obs_o)
{
    if (!obs) {
        *obs_o=NULL;
        return;
    }
    const obs_t *obsd=&obs->obs;
    *obs_o=calloc(1,sizeof(obs_t));
    (*obs_o)->data=calloc(MAXOBS,sizeof(obsd_t));
    memcpy((*obs_o)->data,obsd->data,sizeof(obsd_t)*obsd->n);
    (*obs_o)->n=obsd->n;
}

static void upd_rbobs(const cors_obsd_t *obsr, const cors_obsd_t *obsb, obs_t **robs, obs_t **bobs)
{
    upd_obs(obsr,robs);
    upd_obs(obsb,bobs);
}

static gtime_t upd_bl_time(const obs_t *robs, const obs_t *bobs)
{
    gtime_t tr={0},tb={0};

    if (robs&&robs->n) tr=robs->data[0].time;
    if (bobs&&bobs->n) tb=bobs->data[0].time;

    if (timediff(tr,tb)>0.0) return tr;
    else return tb;
}

static int bl_time_sync(cors_baseline_t *bl, cors_obs_t *obs, obs_t **robs, obs_t **bobs)
{
    cors_obsd_t *obsd[2];
    HASH_FIND_INT(obs->data,&bl->rover_srcid,obsd[0]);
    HASH_FIND_INT(obs->data,&bl->base_srcid,obsd[1]);

    if (!obsd[0]&&!obsd[1]) return 0;

    gtime_t time_cur={0};
    int i,sync=0;
    time_cur=upd_bl_time(&obsd[0]->obs,&obsd[1]->obs);

#if SRTK_STRICT_TIME_SYNC
    static double age=1E-2;
#else
    static double age=15.0;
#endif
    if (fabs(timediff(time_cur,bl->time))<1E-2) {
        return 0;
    }
    for (i=0;i<2;i++) {
        if (obsd[i]&&fabs(timediff(time_cur,obsd[i]->obs.data[0].time))<age) {
            sync++;
        }
    }
#if SRTK_STRICT_BSTA_SYNC
    if (sync==2) {
        upd_rbobs(obsd[0],obsd[1],robs,bobs);
        return 1;
    }
#else
#if SRTK_SYNC_WAIT
    static double mt=200.0;
    static double st=0.1;
    if (sync&&++bl->wt>st*1E9/mt) {
        upd_rbobs(obsd[0],obsd[1],robs,bobs);
        return 1;
    }
#else
    if (sync) {
        upd_rbobs(obsd[0],obsd[1],robs,bobs);
        return 1;
    }
#endif
#endif
    return 0;
}

static void add_rtkpos_work(cors_srtk_t *srtk, cors_baseline_t *bl, obs_t *robs, obs_t *bobs)
{
    uv_mutex_lock(&srtk->rtkpos_lock);
    rtkpos_task_t *task=new_rtkpos_task(srtk,bl,robs,bobs);

    QUEUE_INSERT_TAIL(&srtk->rtkpos_queue,&task->q);
    uv_mutex_unlock(&srtk->rtkpos_lock);
}

static void upd_stapos_prc(cors_baseline_t *bl, const sta_t *sta, int type)
{
    double pos[3]={0},del[3]={0},dr[3]={0};
    double *rr=type?bl->rtk.rr:bl->rtk.rb;
    int i;

    matcpy(rr,sta->pos,1,3);
    ecef2pos(rr,pos);

    if (sta->deltype) {
        del[2]=sta->hgt; enu2ecef(pos,del,dr);
        for (i=0;i<3;i++) rr[i]+=sta->del[i]+dr[i];
    }
    else {
        enu2ecef(pos,sta->del,dr);
        for (i=0;i<3;i++) rr[i]+=dr[i];
    }
}

static void upd_rtk_stapos(rtkpos_task_t *data)
{
    cors_t *cors=data->srtk->cors;
    cors_stas_t *stas=&cors->stas;
    cors_sta_t *sta;

    HASH_FIND_INT(stas->data,&data->bl->base_srcid,sta);
    if (sta) upd_stapos_prc(data->bl,&sta->sta,0);
    HASH_FIND_INT(stas->data,&data->bl->rover_srcid,sta);
    if (sta) upd_stapos_prc(data->bl,&sta->sta,1);
}

static void do_rtkpos_work(rtkpos_task_t *data)
{
    cors_baseline_t *bl=data->bl;
    cors_t *cors=data->srtk->cors;
    rtk_t *rtk=&bl->rtk;

    char tbuf_r[32]={0};
    double dr[3];
    int i;

    upd_rtk_stapos(data);
    rtkpos(rtk,data->robs,data->bobs,&cors->nav.data);
    bl->on--;

    for (i=0;i<3;i++) dr[i]=bl->rtk.rb[i]-bl->rtk.sol.rr[i];
    time2str(bl->rtk.time,tbuf_r,2);

    log_trace(1,"%4d->%4d(%8.3lf) delay=%2d age=%5.1lf stat=%d nb=%2d %s\n",bl->base_srcid,bl->rover_srcid,norm(dr,3)/1000.0,
            bl->on,bl->rtk.sol.age,bl->rtk.sol.stat,rtk->nb,tbuf_r);

    cors_updssat(&cors->ssats,rtk->ssat,bl->rover_srcid,2,rtk->sol.time);
    cors_updblsol(&cors->blsols,bl,rtk,bl->base_srcid,bl->rover_srcid);

    freeobs(data->robs);
    freeobs(data->bobs); free(data);
}

static void rtk_process(cors_srtk_t *srtk)
{
    while (!QUEUE_EMPTY(&srtk->rtkpos_queue)) {
        uv_mutex_lock(&srtk->rtkpos_lock);

        QUEUE *q=QUEUE_HEAD(&srtk->rtkpos_queue);
        rtkpos_task_t *data=QUEUE_DATA(q,rtkpos_task_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&srtk->rtkpos_lock);
        do_rtkpos_work(data);
    }
}

static void rtk_work_thread(void *srtk_arg)
{
    cors_srtk_t *srtk=(cors_srtk_t*)srtk_arg;
    set_thread_rt_priority();

    while (srtk->state) {
        if (srtk->state<=1) continue;
        rtk_process(srtk);
    }
}

static void do_add_baseline(add_baseline_t *data);

static void add_baseline_work(cors_srtk_t *srtk)
{
    while (!QUEUE_EMPTY(&srtk->addbl_queue)) {
        uv_mutex_lock(&srtk->addbl_lock);
        QUEUE *q=QUEUE_HEAD(&srtk->addbl_queue);
        add_baseline_t *add=QUEUE_DATA(q,add_baseline_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&srtk->addbl_lock);
        do_add_baseline(add);
    }
}

static void do_del_baseline(del_baseline_t *del)
{
    cors_baseline_t *bl=del->bl;
    while (bl->on) uv_sleep(5);

    rtkfree(&bl->rtk);
    free(bl); free(del);
}

static void del_baseline_prc(cors_srtk_t *srtk, del_baseline_t *del)
{
    HASH_DEL(srtk->bls.data,del->bl);
    do_del_baseline(del);
}

static void del_baseline_work(cors_srtk_t *srtk)
{
    while (!QUEUE_EMPTY(&srtk->delbl_queue)) {

        uv_mutex_lock(&srtk->delbl_lock);
        QUEUE *q=QUEUE_HEAD(&srtk->delbl_queue[0]);
        del_baseline_t *del=QUEUE_DATA(q,del_baseline_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&srtk->delbl_lock);
        del_baseline_prc(srtk,del);
    }
}

static void upd_bl_stat(cors_baseline_t *bl, const obs_t *robs, const obs_t *bobs)
{
    bl->time=upd_bl_time(robs,bobs);
    bl->wt=0;
}

static void baseline_rtk_work(cors_srtk_t *srtk)
{
    cors_baseline_t *bl,*bltmp;
    obs_t *robs,*bobs;
    cors_t *cors=srtk->cors;

    HASH_ITER(hh,srtk->bls.data,bl,bltmp) {
        if (!bl_time_sync(bl,&cors->obs,&robs,&bobs)) continue;
        upd_bl_stat(bl,robs,bobs);
        add_rtkpos_work(srtk,bl,robs,bobs);
    }
}

static void rtk_thread(void *srtk_arg)
{
    cors_srtk_t *srtk=(cors_srtk_t*)srtk_arg;
    srtk->state++;

    set_thread_rt_priority();

    while (srtk->state) {
        if (srtk->state<=1) continue;
        baseline_rtk_work(srtk);
        add_baseline_work(srtk);
        del_baseline_work(srtk);
    }
}

static cors_baseline_t *new_baseline(cors_srtk_t *srtk, const char *id, int base_srcid, int rover_srcid)
{
    cors_baseline_t *bl=calloc(1,sizeof(*bl));
    strcpy(bl->id,id);
    bl->rover_srcid=rover_srcid;
    bl->base_srcid=base_srcid;
    if (bl->rover_srcid<0) srtk->opt.mode=PMODE_KINEMA;
    rtkinit(&bl->rtk,&srtk->opt);
    return bl;
}

static void upd_nrtk_baseline(cors_srtk_t *srtk, cors_baseline_t *bl, int base_srcid, int rover_srcid)
{
    return cors_nrtk_upd_baseline(srtk->nrtk,bl,base_srcid,rover_srcid);
}

static void do_add_baseline(add_baseline_t *data)
{
    cors_srtk_t *srtk=data->srtk;
    cors_baseline_t *bl;
    cors_t *cors=srtk->cors;
    char id[16],id_[16];

    sprintf(id,"%d->%d",data->base_srcid,data->rover_srcid);

    HASH_FIND_STR(srtk->bls.data,id,bl);
    if (bl) {
        free(data); return;
    }
#if SRTK_SYMMETRY_MODE
    sprintf(id_,"%d->%d",data->rover_srcid,data->base_srcid);
    HASH_FIND_STR(srtk->bls.data,id_,bl);
#else
    if (srtk->opt.mode==PMODE_FIXED) {
        sprintf(id_,"%d->%d",data->rover_srcid,data->base_srcid);
        HASH_FIND_STR(srtk->bls.data,id_,bl);
    }
#endif
    if (!bl) {
        bl=new_baseline(srtk,id,data->base_srcid,data->rover_srcid);
        HASH_ADD_STR(srtk->bls.data,id,bl);
    }
    upd_nrtk_baseline(srtk,bl,data->base_srcid,
            data->rover_srcid);
    free(data);
}

static add_baseline_t* new_add_baseline(cors_srtk_t *srtk, int base_srcid, int rover_srcid)
{
    add_baseline_t *data=calloc(1,sizeof(*data));
    data->srtk=srtk;
    data->rover_srcid=rover_srcid;
    data->base_srcid=base_srcid;
    return data;
}

static int read_bls_file(cors_srtk_t *srtk, const char *file)
{
    FILE *fp;

    if (!(fp=fopen(file,"r"))) {
        log_trace(2,"read baselines file fail: %s\n",file);
        return -1;
    }
    cors_t *cors=srtk->cors;
    cors_ntrip_source_info_t *info_tbl=cors->ntrip.info_tbl[0];
    cors_ntrip_source_info_t *b,*r;
    int n;
    char buff[256],*p,*q,*val[16];

    while (fgets(buff,sizeof(buff),fp)) {
        for (n=0,p=buff;*p&&n<16;p=q+1) {
            if ((q=strchr(p,','))||(q=strchr(p,'#'))) {val[n++]=p; *q='\0';}
            else break;
        }
        if (n<2) continue;
        HASH_FIND_STR(info_tbl,val[0],b);
        HASH_FIND_STR(info_tbl,val[1],r);
        if (!b||!r) continue;
        add_baseline_t *data=new_add_baseline(srtk,b->ID,r->ID);
        do_add_baseline(data);
    }
    fclose(fp);
    return HASH_COUNT(srtk->bls.data);
}

static void srtk_init(cors_srtk_t *srtk, cors_nrtk_t *nrtk, cors_t *cors)
{
    uv_mutex_init(&srtk->addbl_lock);
    uv_mutex_init(&srtk->delbl_lock);
    uv_mutex_init(&srtk->rtkpos_lock);

    srtk->cors=cors;
    srtk->nrtk=nrtk;

    QUEUE_INIT(&srtk->rtkpos_queue);
    QUEUE_INIT(&srtk->addbl_queue);
    QUEUE_INIT(&srtk->delbl_queue);
    QUEUE_INIT(&srtk->delbl_queue);
    srtk->state++;
}

static void read_opts(cors_srtk_t *srtk)
{
    cors_opt_t *opt=&srtk->cors->opt;

    reset_opts();
    if (load_opts(opt->rtk_conf_file,pnt_opts)>0) get_opts(&srtk->opt,NULL,NULL);
    else {
        srtk->opt=prcopt_default_rtk;
    }
}

extern int cors_srtk_start(cors_srtk_t *srtk, cors_t *cors, cors_nrtk_t *nrtk, const char *bls_file)
{
    srtk_init(srtk,nrtk,cors);
    read_opts(srtk);
    read_bls_file(srtk,bls_file);

    if (uv_thread_create(&srtk->thread[0],rtk_thread,srtk)) {
        return 0;
    }
    if (uv_thread_create(&srtk->thread[1],rtk_work_thread,srtk)) {
        return 0;
    }
    log_trace(1,"srtk thread create ok\n");
    return 1;
}

extern int cors_srtk_close(cors_srtk_t *srtk)
{
    srtk->state=0;

    uv_thread_join(&srtk->thread[1]);
    uv_thread_join(&srtk->thread[0]);

    cors_baseline_t *bl,*bltmp;
    HASH_ITER(hh,srtk->bls.data,bl,bltmp) {
        HASH_DEL(srtk->bls.data,bl);
        rtkfree(&bl->rtk); free(bl);
    }
}

extern int cors_srtk_add_baseline(cors_srtk_t *srtk, int base_srcid, int rover_srcid)
{
    if (!srtk->state) return 0;

    uv_mutex_lock(&srtk->addbl_lock);
    add_baseline_t *data=new_add_baseline(srtk,base_srcid,rover_srcid);
    QUEUE_INSERT_TAIL(&srtk->addbl_queue,&data->q);
    uv_mutex_unlock(&srtk->addbl_lock);
    return 1;
}

static del_baseline_t* new_del_baseline(cors_srtk_t *srtk, int base_srcid, int rover_srcid)
{
    char id[16];
    sprintf(id,"%d->%d",base_srcid,rover_srcid);

    cors_baseline_t *bl;
    HASH_FIND_STR(srtk->bls.data,id,bl);
    if (!bl) return NULL;

    del_baseline_t *data=calloc(1,sizeof(*data));
    data->bl=bl;
    data->srtk=srtk;
    return data;
}

extern int cors_srtk_del_baseline(cors_srtk_t *srtk, int base_srcid, int rover_srcid)
{
    if (!srtk->state) return 0;

    del_baseline_t *data=new_del_baseline(srtk,base_srcid,rover_srcid);
    if (!data) return 0;

    uv_mutex_lock(&srtk->delbl_lock);
    QUEUE_INSERT_TAIL(&srtk->delbl_queue,&data->q);
    uv_mutex_unlock(&srtk->delbl_lock);
    return 1;
}



