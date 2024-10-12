/*------------------------------------------------------------------------------
 * monitor_src.c: monitor source data functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

extern int monitor_src_str(const cors_monitor_t *monitor, const cors_monitor_src_t *m_src, const ssat_t *ssat,
                           const sol_t *sol, const obs_t *obs, char *buff);

static void on_rsp_cb(uv_write_t* req, int status)
{
    free(req);
}

static int send_monitor_src_data(cors_monitord_t *md, ssat_t *ssat, sol_t *sol, obs_t *obs)
{
    static char buff[32768];
    int ret;
    uv_write_t *wreq;
    uv_buf_t buf;

    buf.len=monitor_src_str(md->monitor,&md->m_src,ssat,sol,obs,buff);
    buf.base=buff;
    wreq=malloc(sizeof(uv_write_t));

    if (uv_is_closing((uv_handle_t*)md->conn)) {
        free(wreq);
        return -1;
    }
    if (!uv_is_writable((uv_stream_t*)md->conn)) {
        free(wreq);
        return -1;
    }
    if ((ret=uv_write(wreq,(uv_stream_t *)md->conn,&buf,1,on_rsp_cb))!=0) {
        log_trace(1,"failed to send monitor data: %s\n",
                uv_strerror(ret));
        free(wreq);
        return -1;
    }
    return ret;
}

static void add_ntrip_source(cors_monitor_t *monitor, char **argv)
{
    cors_ntrip_source_info_t *info=calloc(1,sizeof(*info));
    cors_t *cors=monitor->cors;
    cors_ntrip_t *ntrip=&cors->ntrip;
    double pos[3];

    strcpy(info->name,argv[2]);
    strcpy(info->addr, argv[3]);
    info->port=atoi(argv[4]);
    strcpy(info->mntpnt,argv[5]);
    strcpy(info->user,argv[6]);
    strcpy(info->passwd,argv[7]);
    pos[0]=atof(argv[8])*D2R;
    pos[1]=atof(argv[9])*D2R;
    pos[2]=atof(argv[10]);
    pos2ecef(pos,info->pos);
    cors_ntrip_add_source(ntrip,info);
}

static void do_parse_argv(char *buf, char **argv)
{
    static char default_str[64]={0};
    char *p=strtok(buf," \t\r\n");
    int argc=0,i;

    for (i=0;i<16;i++) argv[i]=default_str;

    while (p!=NULL&&argc<16) {
        argv[argc++]=p;
        p=strtok(NULL," \t\r\n");
    }
}

static void upd_monitor_src_info(cors_monitor_src_t *m_pnt, char **argv)
{
    strcpy(m_pnt->name,argv[2]);
    strcpy(m_pnt->adrr,argv[3]);
    m_pnt->port=atoi(argv[4]);
    strcpy(m_pnt->mntpnt,argv[5]);
    strcpy(m_pnt->user,argv[6]);
    strcpy(m_pnt->passwd,argv[7]);
    m_pnt->pos[0]=atof(argv[8]);
    m_pnt->pos[1]=atof(argv[9]);
    m_pnt->pos[2]=atof(argv[10]);
    strcpy(m_pnt->site,argv[11]);
}

static cors_monitor_src_q_t* monitord_src_q_new(cors_monitor_src_qs_t *qs, char **argv)
{
    cors_monitor_src_q_t *q=calloc(1,sizeof(*q));
    strcpy(q->name,argv[2]);
    HASH_ADD_STR(qs->q_tbl,name,q);
    return q;
}

static cors_monitord_t* monitord_src_new(cors_monitor_t *monitor, uv_stream_t *str, char **argv)
{
    cors_monitord_t *new=calloc(1,sizeof(*new));
    new->conn=(uv_tcp_t*)str;
    new->monitor=monitor;
    return new;
}

static void monitor_src_updmtbl(cors_monitor_t *monitor, cors_monitord_t *m_cur, cors_monitor_src_qs_t *qs,
                                uv_stream_t *str, char **argv)
{
    cors_monitor_src_q_t *q,*q_cur;
    cors_monitord_t *mf;

    uv_mutex_lock(&qs->lock);
    HASH_FIND_STR(qs->q_tbl,argv[2],q_cur);
    if (!q_cur) {
        q_cur=monitord_src_q_new(qs,argv);
    }
    HASH_FIND_PTR(q_cur->md_tbl,&str,mf);
    if (!mf) {
        if (strcmp(m_cur->m_src.name,"")!=0&&strcmp(m_cur->m_src.name,argv[2])!=0) {
            HASH_FIND_STR(qs->q_tbl,m_cur->m_src.name,q);
            HASH_FIND_PTR(q->md_tbl,&str,mf);
            HASH_DEL(q->md_tbl,mf);
            upd_monitor_src_info(&mf->m_src,argv);
            HASH_ADD_PTR(q_cur->md_tbl,conn,mf);
        }
        else {
            mf=monitord_src_new(monitor,str,argv);
            HASH_ADD_PTR(q_cur->md_tbl,conn,mf);
            upd_monitor_src_info(&mf->m_src,argv);
        }
    }
    upd_monitor_src_info(&m_cur->m_src,argv);
    uv_mutex_unlock(&qs->lock);
}

extern void monitor_src_updconn(uv_stream_t *str, char *buf)
{
    cors_monitord_t *md=str->data,*mf;
    cors_monitor_src_qs_t *qs=&md->monitor->src_qs;
    cors_monitor_t *monitor=md->monitor;
    cors_t *cors=monitor->cors;
    cors_ntrip_t *ntrip=&cors->ntrip;

    char *argv[16];
    cors_ntrip_source_info_t *info;

    do_parse_argv(buf,argv);

    log_trace(1,"[0x%08x] [0x%08x] %s %s\n",str,md,argv[2],md->m_src.name);

    if (strcmp(argv[2],"")==0) {
        return;
    }
    HASH_FIND(hh,ntrip->info_tbl[0],argv[2],strlen(argv[2]),info);
    if (!info) {
        add_ntrip_source(monitor,argv);
        return;
    }
    monitor_src_updmtbl(monitor,md,qs,str,argv);
}

typedef struct monitor_src_task {
    ssat_t ssat[MAXSAT];
    sol_t sol;
    obs_t obs;
    int srcid;
    cors_monitor_t *monitor;
    cors_monitor_src_qs_t *qs;
    QUEUE q;
} monitor_src_task_t;

static void do_monitor_pnt_work(monitor_src_task_t *data)
{
    cors_ntrip_source_info_t *info;
    cors_monitor_src_q_t *q;
    cors_t *cors=data->monitor->cors;
    cors_ntrip_t *ntrip=&cors->ntrip;

    HASH_FIND(ii,ntrip->info_tbl[1],&data->srcid,sizeof(int),info);
    HASH_FIND_STR(data->qs->q_tbl,info->name,q);

    if (!q) {
        freeobs(&data->obs);
        free(data);
        return;
    }
    cors_monitord_t *m,*d;
    HASH_ITER(hh,q->md_tbl,m,d) {
        if (send_monitor_src_data(m,data->ssat,&data->sol,&data->obs)<0) {
            cors_monitor_del(m->monitor,m);
        }
    }
    freeobs(&data->obs);
    free(data);
}

extern void monitor_pnt_moni(uv_async_t *handle)
{
    cors_monitor_src_qs_t *qs=handle->data;

    while (!QUEUE_EMPTY(&qs->data_queue)) {
        uv_mutex_lock(&qs->qlock);

        QUEUE *q=QUEUE_HEAD(&qs->data_queue);
        monitor_src_task_t *data=QUEUE_DATA(q,monitor_src_task_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&qs->qlock);
        do_monitor_pnt_work(data);
    }
}

static monitor_src_task_t* new_monitor_src_task(cors_monitor_t *monitor, cors_monitor_src_qs_t *qs, const ssat_t *ssat,
                                                const sol_t *sol, const obs_t *obs, int srcid)
{
    monitor_src_task_t *task=calloc(1,sizeof(*task));
    task->obs.data=calloc(MAXOBS,sizeof(obsd_t));
    task->obs.n=obs->n;
    task->sol=*sol;
    task->monitor=monitor;
    task->qs=qs;
    task->srcid=srcid;
    memcpy(task->ssat,ssat,sizeof(ssat_t)*MAXSAT);
    memcpy(task->obs.data,obs->data,sizeof(obsd_t)*obs->n);
    return task;
}

static void add_monitor_src_task(cors_monitor_t *monitor, cors_monitor_src_qs_t *qs, const ssat_t *ssat,
                                 const sol_t *sol, const obs_t *obs, int srcid)
{
    uv_mutex_lock(&qs->qlock);
    monitor_src_task_t *task=new_monitor_src_task(monitor,qs,ssat,sol,obs,srcid);
    QUEUE_INSERT_TAIL(&qs->data_queue,&task->q);
    uv_mutex_unlock(&qs->qlock);

    uv_async_send(qs->moni);
}

extern void cors_monitor_src(cors_monitor_t *monitor, const ssat_t *ssat, const sol_t *sol,
                             const obs_t *obs, int srcid)
{
    return add_monitor_src_task(monitor,&monitor->src_qs,
            ssat,sol,obs,srcid);
}

extern void monitor_src_init(uv_loop_t *loop, cors_monitor_t *monitor, cors_monitor_src_qs_t *qs)
{
    qs->moni=calloc(1,sizeof(uv_async_t));
    qs->moni->data=&monitor->src_qs;
    uv_async_init(loop,monitor->src_qs.moni,monitor_pnt_moni);

    uv_mutex_init(&monitor->src_qs.qlock);
    uv_mutex_init(&monitor->src_qs.lock);
    QUEUE_INIT(&monitor->src_qs.data_queue);
}

extern void monitor_src_delete_monitor(cors_monitor_src_qs_t *qs, cors_monitord_t *md)
{
    uv_mutex_lock(&qs->lock);

    cors_monitor_src_q_t *q;
    cors_monitord_t *t;
    HASH_FIND_STR(qs->q_tbl,md->m_src.name,q);
    if (q) {
        HASH_FIND_PTR(q->md_tbl,&md->conn,t);
        if (t) {HASH_DEL(q->md_tbl,t); free(t);}
    }
    uv_mutex_unlock(&qs->lock);
}

extern void monitor_src_close(cors_monitor_src_qs_t *qs)
{
    cors_monitor_src_q_t *q,*p;
    cors_monitord_t *m,*d;

    HASH_ITER(hh,qs->q_tbl,q,p) {
        if (q->md_tbl) HASH_ITER(hh,q->md_tbl,m,d) {HASH_DEL(q->md_tbl,m);}
        HASH_DEL(qs->q_tbl,q);
        free(q);
    }
}
