/*------------------------------------------------------------------------------
 * monitor_bsta_distr.c: monitor base station distribution functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

extern int monitor_bsta_distr_str(const cors_monitor_bstas_info_t *bstas, const char *province, int type,
                                  char *buff);

typedef struct moni_bsta_distr_task {
    cors_monitord_t *md;
    uv_stream_t *str;
    int type;
    char province[32];
    QUEUE q;
} moni_bsta_distr_task_t;

static void do_parse_argv(char *buf, char **argv)
{
    char *p=strtok(buf," \t\r\n");
    int argc=0;

    while (p!=NULL&&argc<16) {
        argv[argc++]=p;
        p=strtok(NULL," \t\r\n");
    }
}

static void on_rsp_cb(uv_write_t* req, int status)
{
    char *buff=req->data;
    free(req);
    free(buff);
}

static int do_monitor_bsta_distr_work(moni_bsta_distr_task_t *data)
{
    cors_monitord_t *md=data->str->data;
    cors_monitor_t *monitor=md->monitor;
    cors_t *cors=monitor->cors;
    int nmax=HASH_COUNT(cors->monitor.moni_bstas_info.data);
    int ret,len;

    if (nmax<=0) return 0;

    char *buff=malloc(nmax*1024*sizeof(char));
    if ((len=monitor_bsta_distr_str(&cors->monitor.moni_bstas_info,data->province,data->type,buff))<=0) {
        free(buff);
        return 0;
    }
    if (uv_is_closing((uv_handle_t*)md->conn)) {
        free(buff);
        return -1;
    }
    if (!uv_is_writable((uv_stream_t*)md->conn)) {
        free(buff);
        return -1;
    }
    uv_write_t *wreq;
    uv_buf_t buf;
    wreq=malloc(sizeof(uv_write_t));

    buf.base=buff;
    buf.len=len;
    wreq->data=buff;

    if ((ret=uv_write(wreq,(uv_stream_t *)md->conn,&buf,1,on_rsp_cb))!=0) {
        log_trace(1,"failed to send monitor data: %s\n",
                uv_strerror(ret));
        free(wreq);
        free(buff);
        return -1;
    }
    return 1;
}

extern void monitor_bsta_distr_moni(uv_async_t *handle)
{
    cors_monitor_bsta_distr_t *bd=handle->data;

    while (!QUEUE_EMPTY(&bd->queue)) {
        uv_mutex_lock(&bd->lock);
        QUEUE *q=QUEUE_HEAD(&bd->queue);
        moni_bsta_distr_task_t *data=QUEUE_DATA(q,moni_bsta_distr_task_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&bd->lock);

        if (do_monitor_bsta_distr_work(data)<0) {
            cors_monitor_del(data->md->monitor,data->md);
        }
    }
}

static moni_bsta_distr_task_t* new_moni_bsta_distr_task(const char *province, int type, uv_stream_t *str, cors_monitord_t *md)
{
    moni_bsta_distr_task_t *task=calloc(1,sizeof(*task));
    task->str=str;
    task->md=md;
    task->type=type;
    strcpy(task->province,province);
    return task;
}

extern void monitor_bsta_distr(uv_stream_t *str, char *buff)
{
    char *argv[16],province[32],type=0;
    do_parse_argv(buff,argv);

    strcpy(province,argv[2]);
    if (strcmp(argv[3],"physics-station")==0) type=0;
    if (strcmp(argv[3],"virtual-station")==0) type=1;
    if (strcmp(argv[3],"all")==0) type=2;

    cors_monitord_t *md=str->data;
    cors_monitor_t *monitor=md->monitor;

    uv_mutex_lock(&monitor->moni_bsta_distr.lock);
    moni_bsta_distr_task_t *task=new_moni_bsta_distr_task(province,type,str,md);
    QUEUE_INSERT_TAIL(&monitor->moni_bsta_distr.queue,&task->q);
    uv_mutex_unlock(&monitor->moni_bsta_distr.lock);

    uv_async_send(monitor->moni_bsta_distr.bsta_distr_moni);
}

extern void monitor_bsta_distr_init(uv_loop_t *loop, cors_monitor_bsta_distr_t *m_bsta_distr)
{
    uv_mutex_init(&m_bsta_distr->lock);
    QUEUE_INIT(&m_bsta_distr->queue);

    m_bsta_distr->bsta_distr_moni=calloc(1,sizeof(uv_async_t));
    m_bsta_distr->bsta_distr_moni->data=m_bsta_distr;
    uv_async_init(loop,m_bsta_distr->bsta_distr_moni,monitor_bsta_distr_moni);
}

