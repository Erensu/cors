#include "cors.h"
#include "options.h"

#define CORS_TSET    0

static void timer_close_cb(uv_timer_t *handle)
{
    static int cnt=0;
    cors_t *cors=(cors_t*)handle->data;

    cors_close(cors);
    uv_timer_stop(handle);
    uv_stop(handle->loop);
}

static void timer_addbl_cb(uv_timer_t *handle)
{
    char *bls[]={
            "A001,A002",
            "A001,A003",
            "A002,A003",
            "A004,A005",
            "A004,A002",
            "A004,A003",
            "A002,A005",
            "A003,A005",
            "A004,A006",
            "A004,A007",
            "A004,A008",
            "A004,A009",
            "A004,A010",
            "A005,A010",
            "A004,A006",
            "A004,A011",
            "A005,A012",
            "A006,A012",
            "A007,A012",
            "A008,A012",
            "A011,A012",
            "A010,A012",
            "A001,A012",
            "A002,A012",
            ""
    };
    char rover[16],base[16],*p;
    static int i=0;
    cors_t *cors=handle->data;

    if (strcmp(bls[i],"")==0) {
        uv_timer_stop(handle);
        return;
    }
    p=strstr(bls[i],",");
    strncpy(rover,bls[i],p-bls[i]);
    strcpy(base,p+1);

    i++;

    cors_ntrip_source_info_t *r,*b;
    HASH_FIND_STR(cors->ntrip.info_tbl[0],rover,r);
    HASH_FIND_STR(cors->ntrip.info_tbl[1],base,b);

    if (!r||!b) return;
    cors_srtk_add_baseline(&cors->srtk,r->ID,b->ID);
}

static void timer_delbl_cb(uv_timer_t *handle)
{
    char *bls[]={
            "A001,A002",
            ""
    };
    char rover[16],base[16],*p;
    static int i=0;
    cors_t *cors=handle->data;

    if (strcmp(bls[i],"")==0) {
        uv_timer_stop(handle);
        return;
    }
    p=strstr(bls[i],",");
    strncpy(rover,bls[i],p-bls[i]);
    strcpy(base,p+1);

    i++;

    cors_ntrip_source_info_t *r,*b;
    HASH_FIND_STR(cors->ntrip.info_tbl[0],rover,r);
    HASH_FIND_STR(cors->ntrip.info_tbl[1],base,b);

    if (!r||!b) return;
    cors_srtk_del_baseline(&cors->srtk,r->ID,b->ID);
}

int main(int argc, const char *argv[])
{
    log_trace_open(".\\cors_trace.out");
    log_set_level(1);

    cors_opt_t opt={0};
    cors_t cors={0};

    cors_loadopts(&opt,"..\\..\\conf\\cors.conf");
    cors_start(&cors,&opt);

    uv_loop_t *loop=uv_loop_new();

#if CORS_TSET
    uv_timer_t timer_close;
    timer_close.data=&cors;
    uv_timer_init(loop,&timer_close);
    uv_timer_start(&timer_close,timer_close_cb,300000,10000);

    uv_timer_t timer_addbl;
    timer_addbl.data=&cors;
    uv_timer_init(loop,&timer_addbl);
    uv_timer_start(&timer_addbl,timer_addbl_cb,1000,1000);

    uv_timer_t timer_delbl;
    timer_delbl.data=&cors;
    uv_timer_init(loop,&timer_delbl);
    uv_timer_start(&timer_delbl,timer_delbl_cb,20000000,2000);
#endif
    uv_run(loop,UV_RUN_DEFAULT);
    free(loop);

    while (cors.state) {uv_sleep(100);}
    return 0;
}
