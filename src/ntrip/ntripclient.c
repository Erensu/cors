/*------------------------------------------------------------------------------
 * ntripclient.c: NTRIP client functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

#define NTRIP_RSP_OK              "ICY 200 OK\r\n"
#define NTRIP_RSP_SOURCETABLE_OK  "SOURCETABLE 200 OK"
#define NTRIP_RSP_UNAUTHORIZED    "HTTP/1.0 401 Unauthorized\r\n"
#define NTRIP_RSP_FORBIDDEN       "HTTP/1.0 403 Forbidden\r\n"
#define NTRIP_RSP_ERROR_PASSED    "ERROR - Bad Password\r\n"
#define NTRIP_RSP_ERROR_MOUNTP    "ERROR - Bad Mountpoint\r\n"

#define ENA_RAW_LOG   1
#define ENA_ERCONN    1

static int reqntrip_cli(cors_ntrip_client_t *cli, char *req_msg)
{
    char user[514],*p=req_msg;

    p+=sprintf(p,"GET %s/%s HTTP/1.0\r\n",cli->url,cli->mntpnt);
    p+=sprintf(p,"User-Agent: NTRIP %s\r\n","Tencent CORS Monitor");

    if (!*cli->user) {
        p+=sprintf(p,"Accept: */*\r\n");
        p+=sprintf(p,"Connection: close\r\n");
    }
    else {
        sprintf(user,"%s:%s",cli->user,cli->passwd);
        p+=sprintf(p,"Authorization: Basic ");
        p+=encbase64(p,(uint8_t *)user,strlen(user));
        p+=sprintf(p,"\r\n");
    }
    p+=sprintf(p,"\r\n");
    return (int)(p-req_msg);
}

static void on_rsp_cb(uv_write_t* req, int status)
{
    free(req);
}

static void alloc_callback(uv_handle_t *handle, size_t suggested_size, uv_buf_t *buf)
{
    buf->base=malloc(suggested_size);
    buf->len=suggested_size;
}

static void on_reconn_cb(uv_async_t *handle)
{
    cors_ntrip_client_t *cli=handle->data;
    uv_loop_t *loop=handle->loop;

    uv_close((uv_handle_t*)handle,on_close_cb);
    cors_ntripcli_close(cli);

    cors_ntripcli_start(loop,cli,cli->read_cb,cli->name,cli->addr,cli->port,
            cli->user,cli->passwd,cli->mntpnt,cli->pos);
}

static void reconn_ntripcli(cors_ntrip_client_t *cli)
{
    uv_async_t *reconn=calloc(1,sizeof(*reconn));
    uv_loop_t *loop=cli->tcp->loop;

    reconn->data=cli;
    uv_async_init(loop,reconn,on_reconn_cb);
    uv_async_send(reconn);
}

static void on_fs_write(uv_fs_t *req)
{
    uv_buf_t *buf=req->data;
    free(buf->base);
    free(buf);
    uv_fs_req_cleanup(req);
    free(req);
}

static void log_data(cors_ntrip_client_t *cli, const char *data, int n)
{
    if (!cli->raw||n<=0) return;

    uv_fs_t *fw_req=calloc(1,sizeof(uv_fs_t));
    uv_buf_t *buf=calloc(1,sizeof(uv_buf_t));
    buf->base=calloc(n,sizeof(char));
    buf->len=n;
    memcpy(buf->base,data,n);
    fw_req->data=buf;

    uv_fs_write(cli->tcp->loop,fw_req,cli->raw,buf,1,-1,on_fs_write);
}

static void on_read_cb(uv_stream_t *str, ssize_t nread, const uv_buf_t *buf)
{
    cors_ntrip_client_t *cli=(cors_ntrip_client_t*)str->data;

    if (nread<0) {
        log_trace(1,"read fail from source: %s %s\n",cli->mntpnt,uv_strerror(nread));
#if ENA_ERCONN
        reconn_ntripcli(cli);
#else
        uv_close((uv_handle_t*)str,on_close_cb);
#endif
        free(buf->base);
        return;
    }
    log_data(cli,buf->base,nread);

    if (strstr(buf->base,NTRIP_RSP_OK)) {
        cli->state=1;
    }
    else if (strstr(buf->base,NTRIP_RSP_SOURCETABLE_OK)) {

    }
    else if (strstr(buf->base,NTRIP_RSP_FORBIDDEN)) {

    }
    else if (strstr(buf->base,NTRIP_RSP_ERROR_MOUNTP)) {

    }
    else if (strstr(buf->base,NTRIP_RSP_ERROR_PASSED)) {

    }
    else if (strstr(buf->base,NTRIP_RSP_UNAUTHORIZED)) {

    }
    else if (cli->state==1) {
        cli->read_cb(cli,buf->base,nread);
    }
    else {
        log_trace(1,"wait source: %s\n",cli->mntpnt);
    }
    free(buf->base);
}

static void on_connect(uv_connect_t* conn, int status)
{
    if (status==UV_ECANCELED) return;

    cors_ntrip_client_t *cli=(cors_ntrip_client_t*)conn->data;
    cli->state=0;

    if (status<0) {
        log_trace(1,"connect fail: %s %s\n",cli->mntpnt,uv_strerror(status));
#if ENA_ERCONN
        reconn_ntripcli(cli);
#endif
        free(conn);
        return;
    }
    char req_msg[1024]={0};
    int r;
    uv_buf_t buf;
    uv_write_t *wr;

    buf.len=reqntrip_cli(cli,req_msg);
    buf.base=req_msg;

    wr=malloc(sizeof(uv_write_t));

    if ((r=uv_write(wr,conn->handle,&buf,1,on_rsp_cb))!=0) {
        log_trace(1,"failed to send req source: %s\n",
                uv_strerror(r));
        free(wr);
        free(conn);
        return;
    }
    uv_read_start((uv_stream_t*)cli->tcp,alloc_callback,on_read_cb);
    free(conn);
}

static void timer_send_gga_cb(uv_timer_t* handle)
{
    cors_ntrip_client_t *cli=(cors_ntrip_client_t*)handle->data;
    uv_stream_t *str=(uv_stream_t*)cli->tcp;

    if (str->type!=UV_TCP||str->loop!=handle->loop) {
        return;
    }
    if (!uv_is_writable(str)) {
        return;
    }
    if (!uv_is_active((uv_handle_t*)cli->tcp)) {
        return;
    }
    uint8_t gga_buf[1024];
    sol_t sol={0};

    sol.time=utc2gpst(timeget());
    sol.stat=SOLQ_FIX;
    matcpy(sol.rr,cli->pos,1,3);

    uv_write_t *wr=malloc(sizeof(uv_write_t));
    uv_buf_t buf;

    buf.len=outnmea_gga(gga_buf,&sol);
    buf.base=gga_buf;

    if (uv_write(wr,str,&buf,1,on_rsp_cb)!=0) {
        free(wr);
        log_trace(1,"send gga to source(%s) fail\n",cli->mntpnt);
        return;
    }
    log_trace(3,"send gga to source(%s) ok: %s",
            cli->mntpnt,gga_buf);
}

static void on_fs_open(uv_fs_t *req)
{
    cors_ntrip_client_t *cli=req->data;
    cli->raw=req->result;
    uv_fs_req_cleanup(req);
    free(req);
}

static void open_raw_logger(uv_loop_t *loop, cors_ntrip_client_t *cli, const char *name,
                            const char *addr, int port, const char *mnt)
{
    char path[256],file[128],*tfmt="%Y_%m_%d_%h_%M_%S";
    uv_fs_t *open_req;

    open_req=calloc(1,sizeof(uv_fs_t));
    open_req->data=cli;

    sprintf(file,".%c%s_%s_%d_%s_%s.raw",FILEPATHSEP,tfmt,addr,port,name,mnt);
    reppath(file,path,timeget(),"","");

    if (uv_fs_open(loop,open_req,path,O_RDWR|O_CREAT,S_IWUSR|S_IRUSR,on_fs_open)) {
        free(open_req); return;
    }
}

extern int cors_ntripcli_start(uv_loop_t *loop, cors_ntrip_client_t *cli, ntrip_cli_read_cb read_cb, const char *name,
                               const char *addr, int port,
                               const char *user, const char *passwd,
                               const char *mnt, const double *pos)
{
    struct addrinfo *res,*curr,hint={0};
    char ipstr[64];
    int ret;

    hint.ai_family=AF_INET;
    hint.ai_socktype=SOCK_STREAM;

    if (getaddrinfo(addr,NULL,&hint,&res)) {
        return 0;
    }
    for (curr=res;curr!=NULL;curr=curr->ai_next) {
        strcpy(ipstr,inet_ntoa(((struct sockaddr_in*)(curr->ai_addr))->sin_addr));
    }
    freeaddrinfo(res);

    strcpy(cli->user,user);
    strcpy(cli->mntpnt,mnt);
    strcpy(cli->addr,addr);
    strcpy(cli->passwd,passwd);
    strcpy(cli->name,name);

    matcpy(cli->pos,pos,1,3);

    cli->read_cb=read_cb;
    cli->port=port;
    cli->timer_send_gga=calloc(1,sizeof(uv_timer_t));
    cli->timer_send_gga->data=cli;
    uv_timer_init(loop,cli->timer_send_gga);
    uv_timer_start(cli->timer_send_gga,timer_send_gga_cb,0,3000);

    cli->tcp=calloc(1,sizeof(uv_tcp_t));
    uv_tcp_init(loop,cli->tcp);
    cli->tcp->data=cli;

    struct sockaddr_in sock;
    uv_ip4_addr(ipstr,port,&sock);

    cli->conn=calloc(1,sizeof(uv_connect_t));
    cli->conn->data=cli;

    if ((ret=uv_tcp_connect(cli->conn,cli->tcp,(const struct sockaddr*)&sock,
            on_connect))) {
        log_trace(1,"connect fail: %s %s\n",cli->mntpnt,uv_strerror(ret));
        free(cli->conn);
        cors_ntripcli_close(cli);
        return 0;
    }
#if ENA_RAW_LOG
    open_raw_logger(loop,cli,name,addr,port,mnt);
#endif
    return 1;
}

static void on_fs_close(uv_fs_t *req)
{
    free(req);
}

extern int cors_ntripcli_close(cors_ntrip_client_t *cli)
{
    if (!uv_is_closing((uv_handle_t*)cli->timer_send_gga)) {
        uv_close((uv_handle_t*)cli->timer_send_gga,on_close_cb);
    }
    if (!uv_is_closing((uv_handle_t*)cli->tcp)) {
        uv_close((uv_handle_t*)cli->tcp,on_close_cb);
    }
    if (cli->raw) {
        uv_fs_t *req=calloc(1,sizeof(uv_fs_t));
        uv_fs_close(cli->tcp->loop,req,cli->raw,on_fs_close);
    }
}