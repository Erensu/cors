/*------------------------------------------------------------------------------
 * monitor.c: monitor functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

#define MONITOR_CMD_SOURCE     "MONITOR-SOURCE"
#define MONITOR_CMD_BSTA_DISTR "MONITOR-BSTADISTR"

extern void monitor_src_updconn(uv_stream_t *str, char *buff);
extern void monitor_src_init(uv_loop_t *loop, cors_monitor_t *monitor, cors_monitor_src_qs_t *qs);
extern void monitor_src_delete_monitor(cors_monitor_src_qs_t *qs, cors_monitord_t *md);
extern void monitor_src_close(cors_monitor_src_qs_t *qs);

extern void monitor_bsta_distr(uv_stream_t *str, char *buff);
extern void monitor_bsta_distr_moni(uv_async_t *handle);
extern void monitor_bsta_distr_init(uv_loop_t *loop, cors_monitor_bsta_distr_t *m_bsta_distr);

extern void monitor_init_bstas_info(cors_monitor_bstas_info_t *bstas);
extern void monitor_free_bstas_info(cors_monitor_bstas_info_t *bstas);
extern int monitor_read_bstas_info(const char *file, cors_monitor_bstas_info_t *bstas);

typedef struct delete_monitor_task {
    cors_monitord_t *md;
    cors_monitor_t *monitor;
    QUEUE q;
} delete_monitor_task_t;

static void close_cb(uv_async_t* handle)
{
    if (uv_loop_alive(handle->loop)) {
        uv_stop(handle->loop);
    }
}

static void alloc_buffer(uv_handle_t *handle, size_t suggested_size, uv_buf_t *buf)
{
    buf->base=malloc(suggested_size);
    buf->len=suggested_size;
}

static void on_read_cb(uv_stream_t *str, ssize_t nr, const uv_buf_t *buf)
{
    cors_monitord_t *md=str->data;

    if (nr<0) {
        cors_monitor_del(md->monitor,md);
        free(buf->base);
        return;
    }
    buf->base[nr]='\0';
    char *p;

    if ((p=strrstr(buf->base,MONITOR_CMD_SOURCE))) {
        monitor_src_updconn(str,p);
    }
    else if ((p=strrstr(buf->base,MONITOR_CMD_BSTA_DISTR))) {
        monitor_bsta_distr(str,p);
    }
    free(buf->base);
}

static void on_add_monitor(uv_async_t *handle)
{
    cors_monitor_t *monitor=handle->data;
    cors_monitord_t *md=calloc(1,sizeof(*md));

    md->monitor=monitor;
    md->conn=calloc(1,sizeof(uv_tcp_t));
    uv_tcp_init(handle->loop,md->conn);

    if (uv_accept((uv_stream_t*)monitor->svr,(uv_stream_t*)md->conn)!=0) {
        free(md->conn);
        free(md); return;
    }
    md->conn->data=md;
    uv_read_start((uv_stream_t*)md->conn,alloc_buffer,on_read_cb);
    HASH_ADD_PTR(monitor->moni_tbl,conn,md);
}

static void do_delete_monitor_work(delete_monitor_task_t *data)
{
    cors_monitord_t *m,*t;
    cors_monitor_src_qs_t *qs=&data->monitor->src_qs;

    HASH_FIND_PTR(data->monitor->moni_tbl,&data->md->conn,m);
    if (!m) return;

    monitor_src_delete_monitor(qs,data->md);

    uv_close((uv_handle_t*)data->md->conn,on_close_cb);
    HASH_DEL(data->monitor->moni_tbl,m);
    free(m);
}

static void on_delete_monitor(uv_async_t *handle)
{
    cors_monitor_t *moni=handle->data;

    while (!QUEUE_EMPTY(&moni->del_queue)) {
        uv_mutex_lock(&moni->dlock);
        QUEUE *q=QUEUE_HEAD(&moni->del_queue);
        delete_monitor_task_t *data=QUEUE_DATA(q,delete_monitor_task_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&moni->dlock);
        do_delete_monitor_work(data);
    }
}

static void on_new_connection(uv_stream_t *svr, int status)
{
    if (status<0) {
        log_trace(1,"new connection error %s\n",uv_strerror(status));
        return;
    }
    cors_monitor_add(svr->data);
}

static void monitor_thread(void *monitor_arg)
{
    cors_monitor_t *monitor=monitor_arg;
    uv_loop_t *loop=uv_loop_new();

    monitor->svr=calloc(1,sizeof(uv_tcp_t));
    monitor->svr->data=monitor;
    uv_tcp_init(loop,monitor->svr);
    struct sockaddr_in addr;

    uv_ip4_addr("127.0.0.1",monitor->port,&addr);
    uv_tcp_bind(monitor->svr,(const struct sockaddr*)&addr,0);

    int ret=uv_listen((uv_stream_t*)monitor->svr,SOMAXCONN,on_new_connection);
    if (ret) {
        log_trace(1,"monitor error %s\n",uv_strerror(ret));
        free(loop); return;
    }
    monitor->close=calloc(1,sizeof(uv_async_t));
    monitor->close->data=monitor;
    uv_async_init(loop,monitor->close,close_cb);

    monitor_src_init(loop,monitor,&monitor->src_qs);
    monitor_bsta_distr_init(loop,&monitor->moni_bsta_distr);
    cors_monitor_initrtcm(&monitor->moni_rtcm);
    cors_monitor_initnav(&monitor->moni_nav);

    monitor_init_bstas_info(&monitor->moni_bstas_info);
    monitor_read_bstas_info(monitor->bstas_info_file,&monitor->moni_bstas_info);

    monitor->add_moni=calloc(1,sizeof(uv_async_t));
    monitor->add_moni->data=monitor;
    uv_async_init(loop,monitor->add_moni,on_add_monitor);

    monitor->delete_moni=calloc(1,sizeof(uv_async_t));
    monitor->delete_moni->data=monitor;
    uv_async_init(loop,monitor->delete_moni,on_delete_monitor);

    uv_mutex_init(&monitor->dlock);
    QUEUE_INIT(&monitor->del_queue);

    uv_run(loop,UV_RUN_DEFAULT);
    close_uv_loop(loop);
}

extern int cors_monitor_start(cors_monitor_t *monitor, cors_t *cors)
{
    monitor->cors=cors;
    if (uv_thread_create(&monitor->thread,monitor_thread,monitor)) {
        log_trace(1,"monitor thread create error\n");
        return 0;
    }
    log_trace(1,"monitor thread create ok\n");
    return 1;
}

extern void cors_monitor_close(cors_monitor_t *monitor)
{
    uv_async_send(monitor->close);
    uv_thread_join(&monitor->thread);

    monitor_src_close(&monitor->src_qs);
    monitor_free_bstas_info(&monitor->moni_bstas_info);

    cors_monitord_t *m,*d;
    HASH_ITER(hh,monitor->moni_tbl,m,d) {
        HASH_DEL(monitor->moni_tbl,m);
        free(m);
    }
    cors_monitor_freertcm(&monitor->moni_rtcm);
    cors_monitor_freenav(&monitor->moni_nav);
}

extern int cors_monitor_add(cors_monitor_t *monitor)
{
    uv_async_send(monitor->add_moni);
}

static delete_monitor_task_t* new_delete_task(cors_monitor_t *monitor, cors_monitord_t *md)
{
    delete_monitor_task_t *task=calloc(1,sizeof(*task));
    task->monitor=monitor;
    task->md=md;
    return task;
}

extern void cors_monitor_del(cors_monitor_t *monitor, cors_monitord_t *md)
{
    uv_mutex_lock(&monitor->dlock);
    delete_monitor_task_t *task=new_delete_task(monitor,md);
    QUEUE_INSERT_TAIL(&monitor->del_queue,&task->q);
    uv_mutex_unlock(&monitor->dlock);

    uv_async_send(monitor->delete_moni);
}