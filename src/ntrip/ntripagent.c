/*------------------------------------------------------------------------------
 * ntripagent.c: NTRIP agent functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

#define NTRIP_AGENT_MNTPNT  "RTCM32"
#define NTRIP_RSP_UNAUTH    "HTTP/1.0 401 Unauthorized\r\n"
#define NTRIP_RSP_OK_CLI    "ICY 200 OK\r\n"
#define NTRIP_AGENT_PORT     8002

typedef struct agent_del_ntripconn {
    cors_ntrip_agent_t *agent;
    cors_ntrip_conn_t *conn;
    QUEUE q;
} agent_del_ntripconn_t;

typedef struct agent_send_data {
    int nb;
    char mntpnt[32];
    char *buff;
    const nav_t *nav;
    cors_ntrip_agent_t *agent;
    QUEUE q;
} agent_send_data_t;

extern void ntripagnet_del_conn(cors_ntrip_agent_t *agent, cors_ntrip_conn_t *conn);

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

static int test_mntpnt(cors_ntrip_agent_t *agent, cors_ntrip_conn_t *conn, const char *mntpnt)
{
    cors_ntrip_source_info_t *info;
    HASH_FIND_STR(agent->ntrip->info_tbl[0],mntpnt,info);

    strcpy(conn->mntpnt,mntpnt);
    if (info) return 1;
    if (strcmp(mntpnt,NTRIP_AGENT_MNTPNT)) return 0;
    return 2;
}

static void on_rsp_cb(uv_write_t* req, int status)
{
    char *p=req->data;
    free(req); free(p);
}

static void agent_send_data(cors_ntrip_conn_t *conn, const char *buff, int nb)
{
    uv_buf_t buf;
    uv_write_t *wreq=malloc(sizeof(uv_write_t));
    int ret;

    buf.base=calloc(nb,sizeof(char));
    buf.len=nb;

    memcpy(buf.base,buff,nb);
    wreq->data=buf.base;

    if (!uv_is_writable((uv_stream_t*)conn->conn)||
        uv_is_closing((uv_handle_t*)conn->conn)) {
        free(wreq);
        free(buf.base);
        return;
    }
    if ((ret=uv_write(wreq,(uv_stream_t *)conn->conn,&buf,1,on_rsp_cb))!=0) {
        log_trace(1,"agent failed to send data: %s\n",
                  uv_strerror(ret));
        free(wreq);
        free(buf.base);
        return;
    }
}

static void send_rsqc(cors_ntrip_agent_t *agent, cors_ntrip_conn_t *cn, const char *rsp)
{
    if (strlen(rsp)<=0) return;
    agent_send_data(cn,rsp,strlen(rsp));
}

static int test_auth(cors_ntrip_agent_t *agent, const char *user, const char *user_pwd)
{
    cors_ntrip_user_t *u;
    HASH_FIND_STR(agent->user_tbl,user,u);
    if (!u||strcmp(u->passwd,user_pwd)) return 0;
    return 1;
}

static int test_vstachg(cors_ntrip_agent_t *agent, cors_ntrip_conn_t *conn, char *new_mntpnt)
{
    struct kdres *res;
    struct kdtree *vts_kdtree=agent->ntrip->src_kdtree;
    cors_ntrip_source_info_t *info;

    if (conn->type!=2||norm(conn->pos,3)<=0.0) return 0;
    if (!vts_kdtree) return 0;
    if (!(res=kd_nearest(vts_kdtree,conn->pos))) return 0;
    info=kd_res_item_data(res);

    if (strcmp(new_mntpnt,info->name)) {
        strcpy(new_mntpnt,info->name);
        return 1;
    }
    return 0;
}

static int test_gga_msg(cors_ntrip_agent_t *agent, cors_ntrip_conn_t *conn, const char *buff)
{
    char ggastr[256];
    char *p=strchr(buff,'$'),*q=strstr(buff,"\r\n");
    sol_t sol={0};

    if (!p||!q) return 1;
    *q='\0';

    log_trace(1,"rcv GGA message: %s\n",p);

    sol.time=timeget();
    if (!decode_nmea(p,&sol)) return 1;
    matcpy(conn->pos,sol.rr,1,3);
    return 1;
}

static cors_ntrip_conn_q_t *new_conn_q(cors_ntrip_agent_t *agent, const char *mntpnt)
{
    cors_ntrip_conn_q_t *q=calloc(1,sizeof(cors_ntrip_conn_q_t));
    strcpy(q->mntpnt,mntpnt);
    HASH_ADD_STR(agent->cq_tbl,mntpnt,q);
    return q;
}

static void agent_upd_conn(cors_ntrip_agent_t *agent, cors_ntrip_conn_t *conn, uv_stream_t *str,
                           const char *new_mntpnt)
{
    cors_ntrip_conn_q_t *q,*q_c;
    cors_ntrip_conn_t *c;

    if (!strcmp(new_mntpnt,"")) return;

    uv_mutex_lock(&agent->cq_lock);
    HASH_FIND_STR(agent->cq_tbl,new_mntpnt,q_c);
    if (!q_c) q_c=new_conn_q(agent,new_mntpnt);

    HASH_FIND_PTR(q_c->cs,&str,c);
    if (c) {
        uv_mutex_unlock(&agent->cq_lock);
        return;
    }
    if (strcmp(new_mntpnt,conn->mntpnt)) {
        HASH_FIND_STR(agent->cq_tbl,conn->mntpnt,q);
        HASH_FIND_PTR(q->cs,&str,c);
        HASH_DEL(q->cs,c);
    }
    else {
        c=calloc(1,sizeof(*c));
        *c=*conn;
        c->conn=(uv_tcp_t*)str;
    }
    strcpy(conn->mntpnt,new_mntpnt);
    strcpy(c->mntpnt,new_mntpnt);
    HASH_ADD_PTR(q_c->cs,conn,c);
    c->sta_chg=1;
    uv_mutex_unlock(&agent->cq_lock);
}

static int agent_test_msgc(cors_ntrip_agent_t *agent, cors_ntrip_conn_t *conn, uv_stream_t *str,
                           char *buff, int nb)
{
    char url[256]="",mntpnt[256]="",proto[256]="",*p,*q;
    char tmp[256]={0},user[513]={0},user_pwd[256]={0},new_mntpnt[32]={0};
    int len=0;

    if (conn->state) {
        test_gga_msg(agent,conn,buff); test_vstachg(agent,conn,new_mntpnt);
        agent_upd_conn(agent,conn,str,new_mntpnt);
        return 1;
    }
    if (!(p=strstr(buff,"GET"))||!(q=strstr(p,"\r\n"))||
        !(q=strstr(q,"User-Agent:"))||!strstr(q,"\r\n")) {
        ntripagnet_del_conn(agent,conn);
        return 0;
    }
    if (sscanf(p,"GET %255s %255s",url,proto)<2||strcmp(proto,"HTTP/1.0")) {
        ntripagnet_del_conn(agent,conn);
        return 0;
    }
    if ((p=strchr(url,'/'))) strcpy(mntpnt,p+1);

    if (!*mntpnt||!(conn->type=test_mntpnt(agent,conn,mntpnt))) {

        send_rsqc(agent,conn,"");
        ntripagnet_del_conn(agent,conn);
        return 0;
    }
    if (!(p=strstr(buff,"Authorization: Basic "))||!(q=strstr(p,"\r\n"))) {
        send_rsqc(agent,conn,NTRIP_RSP_UNAUTH);
        ntripagnet_del_conn(agent,conn);
        return 0;
    }
    p+=strlen("Authorization: Basic ");
    *q='\0';

    if (*p&&!decbase64(p,q-p,tmp,&len)) {
        if ((p=strstr(tmp,":"))) *p='\0';
        strcpy(user,tmp);
        if (++p&&*p) strcpy(user_pwd,p);
    }
    if (!test_auth(agent,user,user_pwd)) {
        send_rsqc(agent,conn,NTRIP_RSP_UNAUTH);
        ntripagnet_del_conn(agent,conn);
        return 0;
    }
    conn->state=1;
    send_rsqc(agent,conn,NTRIP_RSP_OK_CLI);
    agent_upd_conn(agent,conn,str,mntpnt);
    return 1;
}

static void on_read_cb(uv_stream_t *str, ssize_t nr, const uv_buf_t *buf)
{
    cors_ntrip_conn_t *conn=str->data;

    if (nr<0) {
        ntripagnet_del_conn(conn->agent,conn);
        free(buf->base);
        return;
    }
    agent_test_msgc(conn->agent,conn,str,buf->base,nr);
    free(buf->base);
}

static void agent_add_ntripconn(cors_ntrip_agent_t *agent)
{
    uv_async_send(agent->add_conn);
}

static void on_new_ntripconn(uv_stream_t *svr, int status)
{
    if (status<0) {
        log_trace(1,"new connection error %s\n",uv_strerror(status));
        return;
    }
    agent_add_ntripconn(svr->data);
}

static void on_add_ntripconn_cb(uv_async_t *handle)
{
    cors_ntrip_conn_t *conn=calloc(1,sizeof(*conn));
    cors_ntrip_agent_t *agent=handle->data;

    conn->agent=agent;
    conn->conn=calloc(1,sizeof(uv_tcp_t));
    uv_tcp_init(handle->loop,conn->conn);

    if (uv_accept((uv_stream_t*)agent->svr,(uv_stream_t*)conn->conn)!=0) {
        free(conn->conn);
        free(conn); return;
    }
    conn->conn->data=conn;
    uv_read_start((uv_stream_t*)conn->conn,alloc_buffer,on_read_cb);
    HASH_ADD_PTR(agent->conn_tbl,conn,conn);
}

static void do_del_ntripconn(agent_del_ntripconn_t *data)
{
    cors_ntrip_agent_t *agent=data->agent;
    cors_ntrip_conn_t *c,*t;
    cors_ntrip_conn_q_t *cq;

    HASH_FIND_PTR(agent->conn_tbl,&data->conn->conn,c);
    if (!c) return;

    uv_mutex_lock(&agent->cq_lock);
    HASH_FIND_STR(agent->cq_tbl,data->conn->mntpnt,cq);
    if (cq) {
        HASH_FIND_PTR(cq->cs,&c->conn,t);
        if (t) HASH_DEL(cq->cs,t); free(t);
    }
    uv_close((uv_handle_t*)c->conn,on_close_cb);
    HASH_DEL(agent->conn_tbl,c);
    uv_mutex_unlock(&agent->cq_lock);
    free(c); free(data);
}

static void on_del_ntripconn_cb(uv_async_t *handle)
{
    cors_ntrip_agent_t *agent=handle->data;

    while (!QUEUE_EMPTY(&agent->del_queue)) {
        uv_mutex_lock(&agent->del_lock);
        QUEUE *q=QUEUE_HEAD(&agent->del_queue);
        agent_del_ntripconn_t *data=QUEUE_DATA(q,agent_del_ntripconn_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&agent->del_lock);
        do_del_ntripconn(data);
    }
}

extern void ntripagnet_del_conn(cors_ntrip_agent_t *agent, cors_ntrip_conn_t *conn)
{
    uv_mutex_lock(&agent->del_lock);
    agent_del_ntripconn_t *data=calloc(1,sizeof(*data));
    data->agent=agent;
    data->conn=conn;
    QUEUE_INSERT_TAIL(&agent->del_queue,&data->q);
    uv_mutex_unlock(&agent->del_lock);

    uv_async_send(agent->del_conn);
}

static void agent_send_nav_data(cors_ntrip_conn_t *conn, agent_send_data_t *data)
{
    int nb,type[5]={1019,1020,1046,1042,1044};
    char buff[MAXSAT*256];

    if (timediff(timeget(),conn->time)<600.0) return;
    conn->time=timeget();

    if ((nb=rtcm_encode_nav(type,data->nav,buff))) {
        agent_send_data(conn,buff,nb);
    }
}

static void agent_send_sta_data(cors_ntrip_conn_t *conn, agent_send_data_t *data)
{
    cors_ntrip_agent_t *agent=data->agent;
    cors_ntrip_source_info_t *info_tbl=agent->ntrip->info_tbl[0];
    cors_ntrip_source_info_t *s;
    sta_t sta={0};
    int nb;
    char buff[1024];

    if (!conn->sta_chg) return;

    HASH_FIND_STR(info_tbl,data->mntpnt,s);
    if (!s) return;
    sta.staid=s->ID;
    matcpy(sta.pos,s->pos,1,3);
    conn->sta_chg=0;

    if ((nb=rtcm_encode_sta(1005,&sta,buff))) {
        agent_send_data(conn,buff,nb);
    }
}

static void agent_send_obs_data(cors_ntrip_conn_t *conn, agent_send_data_t *data)
{
    agent_send_data(conn,data->buff,data->nb);
}

static void do_agent_send_data_work(agent_send_data_t *data)
{
    cors_ntrip_conn_q_t *q;
    cors_ntrip_agent_t *agent=data->agent;
    cors_ntrip_conn_t *c,*t;

    HASH_FIND_STR(agent->cq_tbl,data->mntpnt,q);
    if (!q) return;

    uv_mutex_lock(&agent->cq_lock);

    HASH_ITER(hh,q->cs,c,t) {
        agent_send_nav_data(c,data);
        agent_send_sta_data(c,data);
        agent_send_obs_data(c,data);
    }
    uv_mutex_unlock(&agent->cq_lock);

    free(data->buff);
    free(data);
}

static void on_agent_send_cb(uv_async_t *handle)
{
    cors_ntrip_agent_t *agent=handle->data;

    while (!QUEUE_EMPTY(&agent->send_queue)) {
        uv_mutex_lock(&agent->send_lock);

        QUEUE *q=QUEUE_HEAD(&agent->send_queue);
        agent_send_data_t *data=QUEUE_DATA(q,agent_send_data_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&agent->send_lock);
        do_agent_send_data_work(data);
    }
}

static void agent_init(uv_loop_t *loop, cors_ntrip_agent_t *agent)
{
    QUEUE_INIT(&agent->send_queue);
    QUEUE_INIT(&agent->del_queue);

    agent->add_conn=calloc(1,sizeof(uv_async_t));
    agent->del_conn=calloc(1,sizeof(uv_async_t));
    agent->add_conn->data=agent;
    agent->del_conn->data=agent;
    uv_async_init(loop,agent->add_conn,on_add_ntripconn_cb);
    uv_async_init(loop,agent->del_conn,on_del_ntripconn_cb);

    agent->close=calloc(1,sizeof(uv_async_t));
    agent->close->data=agent;
    uv_async_init(loop,agent->close,close_cb);

    agent->send=calloc(1,sizeof(uv_async_t));
    agent->send->data=agent;
    uv_async_init(loop,agent->send,on_agent_send_cb);

    uv_mutex_init(&agent->cq_lock);
    uv_mutex_init(&agent->send_lock);
    uv_mutex_init(&agent->del_lock);
    agent->state=1;
}

static void ntrip_agent_thread(void *ntrip_agent_arg)
{
    cors_ntrip_agent_t *agent=ntrip_agent_arg;
    uv_loop_t *loop=uv_loop_new();

    agent->svr=calloc(1,sizeof(uv_tcp_t));
    agent->svr->data=agent;
    uv_tcp_init(loop,agent->svr);
    struct sockaddr_in addr;

    uv_ip4_addr("127.0.0.1",NTRIP_AGENT_PORT,&addr);
    uv_tcp_bind(agent->svr,(const struct sockaddr*)&addr,0);

    int ret=uv_listen((uv_stream_t*)agent->svr,SOMAXCONN,on_new_ntripconn);
    if (ret) {
        log_trace(1,"agent error %s\n",uv_strerror(ret));
        free(loop); return;
    }
    agent_init(loop,agent);

    uv_run(loop,UV_RUN_DEFAULT);
    close_uv_loop(loop);
    free(loop);
}

static void read_users_file(cors_ntrip_agent_t *agent, const char *users_file)
{
    char buff[1024],*p,*q,*val[64];
    cors_ntrip_user_t *user;
    int n;
    FILE *fp;

    if (!(fp=fopen(users_file,"r"))) {
        return;
    }
    while (fgets(buff,sizeof(buff),fp)) {
        for (n=0,p=buff;*p&&n<16;p=q+1) {
            if ((q=strchr(p,','))||(q=strchr(p,'#'))) {val[n++]=p; *q='\0';}
            else break;
        }
        if (n<2) continue;

        HASH_FIND_STR(agent->user_tbl,val[0],user);
        if (user) continue;
        user=calloc(1,sizeof(*user));
        strcpy(user->user,val[0]);
        strcpy(user->passwd,val[1]);
        HASH_ADD_STR(agent->user_tbl,user,user);
    }
    fclose(fp);
}

extern int cors_ntrip_agent_start(cors_ntrip_agent_t *agent, cors_ntrip_t *ntrip, const char *users_file)
{
    agent->ntrip=ntrip;
    read_users_file(agent,users_file);

    if (uv_thread_create(&agent->thread,ntrip_agent_thread,agent)) {
        log_trace(1,"ntrip agent thread create error\n");
        return 0;
    }
    log_trace(1,"ntrip agent thread create ok\n");
    return 1;
}

extern void cors_ntrip_agent_close(cors_ntrip_agent_t *agent)
{
    agent->state=0;
    uv_async_send(agent->close);
    uv_thread_join(&agent->thread);

    cors_ntrip_conn_t *cq,*ct;
    HASH_ITER(hh,agent->conn_tbl,cq,ct) {
        HASH_DEL(agent->conn_tbl,cq);
        free(cq);
    }
    cors_ntrip_conn_q_t *q,*t;
    HASH_ITER(hh,agent->cq_tbl,q,t) {
        HASH_ITER(hh,q->cs,cq,ct) {
            HASH_DEL(q->cs,cq); free(cq);
        }
    }
    cors_ntrip_user_t *u,*s;
    HASH_ITER(hh,agent->user_tbl,u,s) {
        HASH_DEL(agent->user_tbl,u); free(u);
    }
}

extern int cors_ntrip_agent_add_user(cors_ntrip_agent_t *agent, const char *user, const char *passwd)
{
    if (!agent->state) return 0;

    cors_ntrip_user_t *u;
    HASH_FIND_STR(agent->user_tbl,user,u);
    if (user) return 0;

    u=calloc(1,sizeof(*user));
    strcpy(u->user,user);
    strcpy(u->passwd,passwd);
    HASH_ADD_STR(agent->user_tbl,user,u);
    return 1;
}

extern int cors_ntrip_agent_del_user(cors_ntrip_agent_t *agent, const char *user)
{
    if (!agent->state) return 0;

    cors_ntrip_user_t *u;
    HASH_FIND_STR(agent->user_tbl,user,u);
    if (!u) return 0;

    HASH_DEL(agent->user_tbl,u);
    free(u);
    return 1;
}

static agent_send_data_t *new_agent_data(cors_ntrip_agent_t *agent, const char *mntpnt, const char *buff, int nb,
                                         const nav_t *nav)
{
    agent_send_data_t *data=calloc(1,sizeof(*data));
    data->agent=agent;
    data->nb=nb;
    data->nav=nav;
    strcpy(data->mntpnt,mntpnt);
    data->buff=calloc(nb,sizeof(char)*nb);
    memcpy(data->buff,buff,nb);
    return data;
}

extern int cors_ntrip_agent_send(cors_ntrip_agent_t *agent, const char *mntpnt, const char *buff, int nb,
                                 const nav_t *nav)
{
    if (!agent->state) return 0;

    cors_ntrip_conn_q_t *q;
    HASH_FIND_STR(agent->cq_tbl,mntpnt,q);
    if (!q) return 0;

    uv_mutex_lock(&agent->send_lock);
    agent_send_data_t *data=new_agent_data(agent,mntpnt,buff,nb,nav);
    QUEUE_INSERT_TAIL(&agent->send_queue,&data->q);
    uv_mutex_unlock(&agent->send_lock);

    uv_async_send(agent->send);
    return 1;
}


