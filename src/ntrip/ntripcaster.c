/*------------------------------------------------------------------------------
 * ntripcaster.c: NTRIP caster functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

static int on_read_cb(void* userdata, const uint8_t *data, int n)
{
    cors_ntrip_client_t *cli=userdata;
    cors_ntrip_source_t *src=container_of(cli,cors_ntrip_source_t,cli);
    cors_t *cors=src->ctr->ntrip->cors;

    log_trace(1,"[%2d] receive RTCM data: %d bytes\n",src->ID,n);

    cors_rtcm_decode(&cors->rtcm_decoder,data,n,src->ID);
    cors_ntrip_agent_send(&cors->agent,src->name,data,n,&cors->nav.data);
    return n;
}

static void close_cb(uv_async_t* handle)
{
    if (uv_loop_alive(handle->loop)) {
        uv_stop(handle->loop);
    }
}

static void free_ntrip_caster(cors_ntrip_caster_t *ctr)
{
    cors_ntrip_source_t *src,*stmp;
    HASH_ITER(hh,ctr->src_tbl,src,stmp) {
        HASH_DEL(ctr->src_tbl,src);
        free(src);
    }
}

static void on_add_source_cb(uv_async_t *handle);
static void on_del_source_cb(uv_async_t *handle);

typedef struct caster_thread_argv {
    cors_ntrip_caster_t *ctr;
    cors_ntrip_source_info_t *info_tbl;
} caster_thread_argv_t;

static void ntrip_caster_thread(void *caster_arg)
{
    caster_thread_argv_t *argv=caster_arg;
    cors_ntrip_caster_t *ctr=argv->ctr;
    cors_ntrip_t *ntrip=ctr->ntrip;
    cors_ntrip_source_info_t *info,*tmp,*itmp;
    cors_t *cors=ntrip->cors;

    uv_loop_t *loop=uv_loop_new();
    ctr->loop=loop;
    ctr->state=1;

    ctr->del_src=calloc(1,sizeof(uv_async_t));
    ctr->add_src=calloc(1,sizeof(uv_async_t));
    uv_async_init(loop,ctr->add_src,on_add_source_cb);
    uv_async_init(loop,ctr->del_src,on_del_source_cb);

    ctr->del_src->data=ctr;
    ctr->add_src->data=ctr;

    ctr->close=calloc(1,sizeof(uv_async_t));
    ctr->close->data=ctr;
    uv_async_init(loop,ctr->close,close_cb);

    QUEUE_INIT(&ctr->del_src_queue);
    QUEUE_INIT(&ctr->add_src_queue);

    uv_mutex_init(&ctr->del_lock);
    uv_mutex_init(&ctr->add_lock);

    HASH_ITER(hh,argv->info_tbl,info,tmp) {
        cors_ntrip_source_t *s=calloc(1,sizeof(*s));

        if (!info->type) {
            if (!cors_ntripcli_start(loop,&s->cli,on_read_cb,info->name,info->addr,info->port,
                    info->user,info->passwd,
                    info->mntpnt,info->pos)) {
                free(s);
                continue;
            }
        }
        info->src=s;
        strcpy(s->name,info->name);
        s->ID=info->ID;
        s->type=info->type;
        s->ctr=ctr;
        HASH_ADD_STR(ctr->src_tbl,name,s);

        HASH_FIND(hh,ntrip->info_tbl[0],s->name,strlen(s->name),itmp);
        if (itmp) {
            HASH_DEL(argv->info_tbl,info);
            free(info);
            continue;
        }
        itmp=calloc(1,sizeof(*info));
        *itmp=*info;
        HASH_ADD(hh,ntrip->info_tbl[0],name,strlen(itmp->name),itmp);
        HASH_ADD(ii,ntrip->info_tbl[1],ID,sizeof(int),itmp);
        kd_insert(ntrip->src_kdtree,info->pos,itmp);

        HASH_DEL(argv->info_tbl,info);
        free(info);
    }
    uv_run(loop,UV_RUN_DEFAULT);
    close_uv_loop(loop);

    ctr->state=0;
    free(loop);
    free(argv);
    log_trace(3,"ntrip caster stop ok\n");
}

extern void cors_ntrip_caster_close(cors_ntrip_caster_t *ctr)
{
    uv_async_send(ctr->close);
    uv_thread_join(&ctr->thread);
    free_ntrip_caster(ctr);
}

extern int cors_ntrip_caster_start(cors_ntrip_caster_t *ctr, cors_ntrip_source_info_t *info_tbl)
{
    caster_thread_argv_t *argv=malloc(sizeof(*argv));
    argv->info_tbl=info_tbl;
    argv->ctr=ctr;

    if (uv_thread_create(&ctr->thread,ntrip_caster_thread,argv)) {
        log_trace(1,"ntrip caster thread create error\n");
        free(argv); return 0;
    }
    log_trace(1,"ntrip caster thread create ok\n");
    return 1;
}

typedef struct ntrip_add_source {
    cors_ntrip_caster_t *ctr;
    cors_ntrip_source_info_t *info;
    QUEUE q;
} ntrip_add_source_t;

typedef struct ntrip_del_source {
    cors_ntrip_caster_t *ctr;
    char name[8];
    QUEUE q;
} ntrip_del_source_t;

static void do_add_source(ntrip_add_source_t *data)
{
    cors_ntrip_source_info_t *info=data->info;
    cors_ntrip_caster_t *ctr=data->ctr;
    cors_t *cors=data->ctr->ntrip->cors;
    cors_ntrip_t *ntrip=ctr->ntrip;

    cors_ntrip_source_t *s;
    HASH_FIND_STR(ctr->src_tbl,info->name,s);
    if (s) {
        free(data); free(info);
        return;
    }
    s=calloc(1,sizeof(cors_ntrip_source_t));

    if (!info->type) {
        if (!cors_ntripcli_start(ctr->loop,&s->cli,on_read_cb,info->name,info->addr,info->port,
                info->user,info->passwd,info->mntpnt,info->pos)) {
            free(s); free(data); free(info);
            return;
        }
    }
    info->src=s;
    strcpy(s->name,info->name);
    s->ID=info->ID;
    s->type=info->type;
    s->ctr=ctr;
    HASH_ADD_STR(ctr->src_tbl,name,s);

    HASH_ADD(hh,ntrip->info_tbl[0],name,strlen(info->name),info);
    HASH_ADD(ii,ntrip->info_tbl[1],ID,sizeof(int),info);
    kd_insert(ntrip->src_kdtree,info->pos,info);
    free(data);
}

static void on_add_source_cb(uv_async_t *handle)
{
    cors_ntrip_caster_t *ctr=(cors_ntrip_caster_t*)handle->data;

    while (!QUEUE_EMPTY(&ctr->add_src_queue)) {
        uv_mutex_lock(&ctr->add_lock);

        QUEUE *q=QUEUE_HEAD(&ctr->add_src_queue);
        ntrip_add_source_t *data=QUEUE_DATA(q,ntrip_add_source_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&ctr->add_lock);
        do_add_source(data);
    }
}

static void do_del_source(ntrip_del_source_t *data)
{
    cors_ntrip_caster_t *ctr=data->ctr;
    cors_ntrip_source_t *s;
    HASH_FIND_STR(ctr->src_tbl,data->name,s);
    if (!s) {
        free(data); return;
    }
    cors_ntripcli_close(&s->cli);
    HASH_DEL(ctr->src_tbl,s);
    free(s); free(data);

    cors_ntrip_source_info_t *info,*i,*t;
    cors_ntrip_t *ntrip=ctr->ntrip;

    HASH_FIND(hh,ntrip->info_tbl[0],data->name,strlen(data->name),info);
    if (info) {
        HASH_DELETE(hh,ntrip->info_tbl[0],info);
        HASH_DELETE(ii,ntrip->info_tbl[1],info);
        free(info);
    }
    kd_clear(ntrip->src_kdtree);
    HASH_ITER(hh,ntrip->info_tbl[0],i,t) {
        kd_insert(ntrip->src_kdtree,i->pos,i);
    }
}

static void on_del_source_cb(uv_async_t *handle)
{
    cors_ntrip_caster_t *ctr=handle->data;

    while (!QUEUE_EMPTY(&ctr->del_src_queue)) {
        uv_mutex_lock(&ctr->del_lock);

        QUEUE *q=QUEUE_HEAD(&ctr->del_src_queue);
        ntrip_del_source_t *data=QUEUE_DATA(q,ntrip_del_source_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&ctr->del_lock);
        do_del_source(data);
    }
}

extern int cors_ntrip_caster_del_source(cors_ntrip_caster_t *ctr, const char *name)
{
    if (ctr->state<=0) return 0;

    uv_mutex_lock(&ctr->del_lock);
    ntrip_del_source_t *data=calloc(1,sizeof(*data));
    data->ctr=ctr;
    strcpy(data->name,name);

    QUEUE_INSERT_TAIL(&ctr->del_src_queue,&data->q);
    uv_mutex_unlock(&ctr->del_lock);
    uv_async_send(ctr->del_src);
    return 1;
}

extern int cors_ntrip_caster_add_source(cors_ntrip_caster_t *ctr, cors_ntrip_source_info_t *info)
{
    if (ctr->state<=0) return 0;

    uv_mutex_lock(&ctr->add_lock);

    ntrip_add_source_t *data=calloc(1,sizeof(*data));
    data->ctr=ctr;
    data->info=info;

    QUEUE_INSERT_TAIL(&ctr->add_src_queue,&data->q);
    uv_mutex_unlock(&ctr->add_lock);
    uv_async_send(ctr->add_src);
    return 1;
}

