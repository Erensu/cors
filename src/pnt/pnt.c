/*------------------------------------------------------------------------------
 * pnt.c   : PNT engine for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"
#include "options.h"

typedef struct pnt_data {
    int srcid;
    cors_pnt_t *pnt;
    obs_t obs;
} pnt_data_t;

typedef struct cors_pnt_task {
    pnt_data_t data;
    QUEUE q;
} pnt_task_t;

static void on_timer_stat_cb(uv_timer_t* handle)
{
    NULL;
}

static void do_cors_pnt_work(pnt_task_t *task)
{
    pnt_data_t *data=&task->data;
    obs_t *obs=&data->obs;
    cors_t *cors=task->data.pnt->cors;
    cors_ssat_t *s;
    ssat_t ssat[MAXSAT]={{0}};
    sol_t sol={0};
    char msg[256];

    if (!pntpos(obs->data,obs->n,&cors->nav.data,&task->data.pnt->opt,&sol,NULL,ssat,msg)) {
        log_trace(1,"srcid: %4d point pos error (%s)\n",task->data.srcid,msg);
    }
#if CORS_MONITOR
    cors_monitor_src(&cors->monitor,ssat,&sol,obs,data->srcid);
#endif
    cors_updssat(&cors->ssats,ssat,data->srcid,1,sol.time);
    freeobs(obs); free(task);
}

static void pnt_process_cb(uv_async_t* handle)
{
    cors_pnt_t *pnt=(cors_pnt_t*)handle->data;

    while (!QUEUE_EMPTY(&pnt->pnt_queue)) {
        uv_mutex_lock(&pnt->qlock);

        QUEUE *q=QUEUE_HEAD(&pnt->pnt_queue);
        pnt_task_t *task=QUEUE_DATA(q,pnt_task_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&pnt->qlock);
        do_cors_pnt_work(task);
    }
}

static void close_cb(uv_async_t* handle)
{
    if (uv_loop_alive(handle->loop)) {
        uv_stop(handle->loop);
    }
}

static void read_opts(cors_pnt_t *pnt)
{
    cors_opt_t *opt=&pnt->cors->opt;

    reset_opts();
    if (load_opts(opt->pnt_conf_file,pnt_opts)>0) get_opts(&pnt->opt,NULL,NULL);
    else {
        pnt->opt=prcopt_default_pnt;
    }
}

static void pnt_thread(void *arg)
{
    cors_pnt_t *pnt=(cors_pnt_t*)arg;

    uv_loop_t *loop=uv_loop_new();

    pnt->timer_stat=calloc(1,sizeof(uv_timer_t));
    pnt->timer_stat->data=pnt;

    pnt->process=calloc(1,sizeof(uv_async_t));
    pnt->process->data=pnt;
    uv_async_init(loop,pnt->process,pnt_process_cb);

    pnt->close=calloc(1,sizeof(uv_async_t));
    pnt->close->data=pnt;
    uv_async_init(loop,pnt->close,close_cb);

    uv_mutex_init(&pnt->lock_tbl);
    uv_mutex_init(&pnt->qlock);

    QUEUE_INIT(&pnt->pnt_queue);

    uv_timer_init(loop,pnt->timer_stat);
    uv_timer_start(pnt->timer_stat,on_timer_stat_cb,0,10000);

    uv_run(loop,UV_RUN_DEFAULT);

    close_uv_loop(loop);
    free(loop);
}

extern int cors_pnt_start(cors_pnt_t *pnt, cors_t* cors)
{
    pnt->cors=cors;
    read_opts(pnt);
    if (uv_thread_create(&pnt->thread,pnt_thread,pnt)) {
        log_trace(1,"pnt thread create error\n");
        return 0;
    }
    log_trace(1,"pnt thread create ok\n");
    return 1;
}

static pnt_task_t* new_pnt_task(const obsd_t *obs, int n, cors_pnt_t *pnt, int srcid)
{
    pnt_task_t *task=calloc(1,sizeof(*task));
    task->data.obs.data=malloc(MAXOBS*sizeof(obsd_t));
    task->data.srcid=srcid;
    task->data.pnt=pnt;
    task->data.obs.n=n;
    memcpy(task->data.obs.data,obs,sizeof(obsd_t)*n);
    return task;
}

static void add_pnt_task(cors_pnt_t *pnt, const obsd_t *obs, int n, int srcid)
{
    pnt_task_t *task=new_pnt_task(obs,n,pnt,srcid);
    if (!task) return;

    uv_mutex_lock(&pnt->qlock);
    QUEUE_INSERT_TAIL(&pnt->pnt_queue,&task->q);
    uv_mutex_unlock(&pnt->qlock);

    uv_async_send(pnt->process);
}

extern int cors_pnt_pos(cors_pnt_t *pnt, const obsd_t *obs, int n, int srcid)
{
#if CORS_PNT
    if (n<=0) return 0;
    add_pnt_task(pnt,obs,n,srcid);
    return 1;
#else
    return 0;
#endif
}

extern void cors_pnt_close(cors_pnt_t *pnt)
{
    if (!pnt->close) return;
    uv_async_send(pnt->close);

    uv_thread_join(&pnt->thread);
}
