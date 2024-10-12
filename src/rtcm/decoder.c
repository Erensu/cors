/*------------------------------------------------------------------------------
 * decoder.c: RTCM decoder functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

#define REFINE_RTCM_DECODER    1

typedef struct decode_rtcm_data {
    uint8_t *buff;
    int nb;
    cors_rtcm_decoder_t *decoder;
    cors_rtcm_t *rtcm;
} decode_rtcm_data_t;

typedef struct decode_rtcm_task {
    decode_rtcm_data_t data;
    QUEUE q;
} decode_rtcm_task_t;

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

static void close_cb(uv_async_t* handle)
{
    if (uv_loop_alive(handle->loop)) {
        uv_stop(handle->loop);
    }
}

static void free_rtcm_decoder(cors_rtcm_decoder_t *decoder)
{
    cors_rtcm_t *s,*tmp;
    HASH_ITER(hh,decoder->data_tbl,s,tmp) {
        HASH_DEL(decoder->data_tbl,s);
        free_rtcm(&s->rtcm);
        free(s);
    }
}

extern void cors_rtcm_decoder_close(cors_rtcm_decoder_t *decoder)
{
    if (!decoder->close) return;

    uv_async_send(decoder->close);
    uv_thread_join(&decoder->thread);
    free_rtcm_decoder(decoder);
}

static void on_timer_stat_cb(uv_timer_t* handle)
{
    NULL;
}

static void upd_rtcm_data(cors_rtcm_decoder_t *decoder, rtcm_t *rtcm, int ret)
{
    cors_t *cors=decoder->cors;
    cors_pnt_t *pnt=&cors->pnt;
    cors_ntrip_t *ntrip=&cors->ntrip;
    cors_stas_t *stas=&cors->stas;
    obs_t *obs=&rtcm->obs;
    nav_t *nav=&rtcm->nav;

    if (ret==1) {
        cors_updobs(&cors->obs,obs->data,obs->n,rtcm->srcid);
        cors_pnt_pos(pnt,obs->data,obs->n,rtcm->srcid);
    }
    else if (ret==2) {
        cors_updnav(&cors->nav,nav,rtcm->ephsat,rtcm->ephset);
#if CORS_MONITOR
        cors_monitor_nav(&cors->monitor.moni_nav,nav,rtcm->ephsat,rtcm->ephset,rtcm->srcid);
#endif
    }
    else if (ret==5) {
        cors_updsta(stas,&rtcm->sta,rtcm->srcid);
        cors_ntrip_source_updpos(ntrip,rtcm->sta.pos,rtcm->srcid);
        cors_dtrignet_upd_vertex(&cors->nrtk.dtrig_net,rtcm->sta.pos,rtcm->srcid);
    }
}

static void do_rtcm_decode_work(decode_rtcm_task_t *task)
{
    cors_rtcm_t *s=task->data.rtcm;
    cors_t *cors=task->data.decoder->cors;
    cors_monitor_rtcm_t *moni_rtcm=&cors->monitor.moni_rtcm;
    int i,ret=0,rlen=task->data.nb;

#if REFINE_RTCM_DECODER
    for (i=0;i<task->data.nb;i++) {
        if ((ret=input_rtcm3x(&s->rtcm,task->data.buff+(task->data.nb-rlen),rlen,&rlen))) {
            upd_rtcm_data(task->data.decoder,&s->rtcm,ret);
#if CORS_MONITOR
            cors_monitor_rtcm(moni_rtcm,&s->rtcm,s->rtcm.srcid);
#endif
        }
        if (rlen<=0) break;
    }
#else
    for (i=0;i<task->data.nb;i++) {
        ret=input_rtcm3(&s->rtcm,task->data.buff[i]);
        if (ret) {
            upd_rtcm_data(task->data.decoder,&s->rtcm,ret);
        }
#if CORS_MONITOR
        cors_monitor_rtcm(moni_rtcm,&s->rtcm,s->rtcm.srcid);
#endif
    }
#endif
    free(task->data.buff);
    free(task);
}

static void rtcm_decode_cb(uv_async_t* handle)
{
    cors_rtcm_decoder_t *decoder=handle->data;
    QUEUE *queue=&decoder->decode_queue;

    while (!QUEUE_EMPTY(queue)) {
        uv_mutex_lock(&decoder->qlock);

        QUEUE *q=QUEUE_HEAD(queue);
        decode_rtcm_task_t *task=QUEUE_DATA(q,decode_rtcm_task_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&decoder->qlock);
        do_rtcm_decode_work(task);
    }
}

static void rtcm_decoder_thread(void *arg)
{
    cors_rtcm_decoder_t *decoder=(cors_rtcm_decoder_t*)arg;

    uv_loop_t *loop=uv_loop_new();
    decoder->state=0;

    set_thread_rt_priority();

    decoder->timer_stat=calloc(1,sizeof(uv_timer_t));
    uv_timer_init(loop,decoder->timer_stat);
    uv_timer_start(decoder->timer_stat,on_timer_stat_cb,0,10000);

    decoder->decode=calloc(1,sizeof(uv_async_t));
    decoder->decode->data=decoder;
    uv_async_init(loop,decoder->decode,rtcm_decode_cb);

    uv_mutex_init(&decoder->qlock);
    uv_mutex_init(&decoder->tbl_lock);

    decoder->close=calloc(1,sizeof(uv_async_t));
    uv_async_init(loop,decoder->close,close_cb);

    QUEUE_INIT(&decoder->decode_queue);
    decoder->state=1;

    uv_run(loop,UV_RUN_DEFAULT);
    close_uv_loop(loop);
    free(loop);
}

extern int cors_rtcm_decoder_start(cors_rtcm_decoder_t *decoder, cors_t* cors)
{
    decoder->cors=cors;
    if (uv_thread_create(&decoder->thread,rtcm_decoder_thread,decoder)) {
        log_trace(1,"rtcm decoder thread create error\n");
        return 0;
    }
    log_trace(1,"rtcm decoder thread create ok\n");
    return 1;
}

static cors_rtcm_t* new_cors_rtcm(cors_rtcm_decoder_t *decoder, int srcid)
{
    cors_rtcm_t *rtcm=(cors_rtcm_t*)calloc(1,sizeof(cors_rtcm_t));
    if (!rtcm) return NULL;
    init_rtcm(&rtcm->rtcm);

    rtcm->rtcm.srcid=srcid;

    uv_mutex_lock(&decoder->tbl_lock);
    HASH_ADD_INT(decoder->data_tbl,rtcm.srcid,rtcm);
    uv_mutex_unlock(&decoder->tbl_lock);
    return rtcm;
}

static decode_rtcm_task_t* new_rtcm_decode_task(cors_rtcm_t *rtcm, cors_rtcm_decoder_t *decoder,
                                                const uint8_t *data, int n)
{
    decode_rtcm_task_t *task=calloc(1,sizeof(*task));
    task->data.rtcm=rtcm;
    task->data.decoder=decoder;
    task->data.buff=malloc(n*sizeof(char));
    task->data.nb=n;
    memcpy(task->data.buff,data,n);
    return task;
}

static void add_rtcm_decode_task(cors_rtcm_t *rtcm, cors_rtcm_decoder_t *decoder,
                                 const uint8_t *data, int n)
{
    uv_mutex_lock(&decoder->qlock);
    decode_rtcm_task_t *task=new_rtcm_decode_task(rtcm,decoder,data,n);
    QUEUE_INSERT_TAIL(&decoder->decode_queue,&task->q);
    uv_mutex_unlock(&decoder->qlock);

    uv_async_send(decoder->decode);
}

extern int cors_rtcm_decode(cors_rtcm_decoder_t *decoder, const uint8_t *data, int n, int srcid)
{
    cors_rtcm_t *rtcm;

    if (!decoder->state) return 0;
    if (uv_is_closing((uv_handle_t*)decoder->decode)) {
        return 0;
    }
    if (n<=0||srcid<0) return 0;

    uv_mutex_lock(&decoder->tbl_lock);
    HASH_FIND_INT(decoder->data_tbl,&srcid,rtcm);
    uv_mutex_unlock(&decoder->tbl_lock);

    if (!rtcm) {
        if (!(rtcm=new_cors_rtcm(decoder,srcid))) {
            return 0;
        }
    }
    add_rtcm_decode_task(rtcm,decoder,
            data,n);
    return 1;
}

