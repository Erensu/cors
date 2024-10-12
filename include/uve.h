/*------------------------------------------------------------------------------
 * uve.h    : UVE constants, types and function prototypes
 *
 * author   : sujinglan
 * version  : $Revision:$ $Date:$
 * history  : 2022/11/15 1.0  new
 *-----------------------------------------------------------------------------*/
#ifndef UV_INL_H
#define UV_INL_H

#ifdef __cplusplus
extern "C" {
#endif
#include "uv.h"

static void on_close_cb(uv_handle_t* handle)
{
    if (handle) free(handle);
}

static void close_walk_cb(uv_handle_t* handle, void* arg)
{
    if (!uv_is_closing(handle)) {
        uv_close(handle,on_close_cb);
    }
}

static void close_loop(uv_loop_t* loop)
{
    uv_walk(loop,close_walk_cb,NULL);
    uv_run(loop,UV_RUN_DEFAULT);
}

#define assert(expr)                                       \
    do {                                                   \
        if (!(expr)) {                                     \
        fprintf(stderr,                                    \
                "Assertion failed in %s on line %d: %s\n", \
                __FILE__,                                  \
                __LINE__,                                  \
                #expr);                                    \
        abort();                                           \
        }                                                  \
    } while (0)                                            \

#define close_uv_loop(loop)                                \
    do {                                                   \
        close_loop(loop);                                  \
        assert(0==uv_loop_close(loop));                    \
        uv_library_shutdown();                             \
    } while(0)                                             \

#define container_of(ptr,type,member)                      \
        ((type*)((char*)(ptr)-offsetof(type,member)))      \

#ifdef __cplusplus
}
#endif
#endif