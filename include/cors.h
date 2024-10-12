/*------------------------------------------------------------------------------
 * cors.h : CORS constants, types and function prototypes
 *
 * author : sujinglan
 * version  : $Revision:$ $Date:$
 * history  : 2022/11/15 1.0  new
 *-----------------------------------------------------------------------------*/
#ifndef CORS_H
#define CORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtklib.h"
#include "log.h"
#include "uve.h"
#include "uthash.h"
#include "queue.h"
#include "triangle.h"
#include "kdtree.h"

typedef int (*ntrip_cli_read_cb)(void* userdata, const uint8_t *data, int n);

#define MAX_RTCM_MSG  32
#define CORS_MONITOR  1
#define CORS_PNT      1

typedef struct cors_ntrip_client {
    uv_connect_t *conn;
    uv_tcp_t *tcp;
    uv_file raw;
    uv_timer_t *timer_send_gga;
    ntrip_cli_read_cb read_cb;
    int port,state;
    char user[64];
    char name[64];
    char passwd[32];
    char mntpnt[64];
    char addr[32],url[32];
    double pos[3];
} cors_ntrip_client_t;

typedef struct cors_ntrip_source {
    char name[64];
    int ID,type;
    cors_ntrip_client_t cli;
    struct cors_ntrip_caster *ctr;
    UT_hash_handle hh;
} cors_ntrip_source_t;

typedef struct cors_ntrip_source_info {
    char user[32],passwd[32];
    char addr[32],url[32];
    char name[64];
    char mntpnt[32];
    int port,ID,type;
    double pos[3];
    cors_ntrip_source_t *src;
    UT_hash_handle hh;
    UT_hash_handle ii;
} cors_ntrip_source_info_t;

typedef struct cors_ntrip_conn {
    uv_tcp_t *conn;
    int state,nb,type,sta_chg;
    char user[32],passwd[32];
    char mntpnt[32];
    char buff[MAXBUFLEN];
    double pos[3];
    gtime_t time;
    struct cors_ntrip_agent *agent;
    UT_hash_handle hh;
} cors_ntrip_conn_t;

typedef struct cors_ntrip_conn_q {
    char mntpnt[32];
    cors_ntrip_conn_t *cs;
    UT_hash_handle hh;
} cors_ntrip_conn_q_t;

typedef struct cors_ntrip_user {
    char user[64];
    char passwd[64];
    UT_hash_handle hh;
} cors_ntrip_user_t;

typedef struct cors_ntrip_agent {
    int state;
    uv_thread_t thread;
    uv_async_t *close;
    uv_async_t *del_conn;
    uv_async_t *add_conn;
    uv_async_t *send;
    uv_tcp_t *svr;
    uv_mutex_t del_lock;
    uv_mutex_t send_lock;
    uv_mutex_t cq_lock;
    struct cors_ntrip *ntrip;
    cors_ntrip_conn_q_t *cq_tbl;
    cors_ntrip_conn_t *conn_tbl;
    cors_ntrip_user_t *user_tbl;
    QUEUE del_queue;
    QUEUE send_queue;
} cors_ntrip_agent_t;

typedef struct cors_ntrip_caster {
    uv_loop_t *loop;
    uv_thread_t thread;
    uv_mutex_t add_lock,del_lock;
    uv_async_t *add_src;
    uv_async_t *del_src;
    uv_async_t *close;
    int state,ID,type;
    cors_ntrip_source_t *src_tbl;
    struct cors_ntrip *ntrip;
    QUEUE add_src_queue;
    QUEUE del_src_queue;
    UT_hash_handle hh;
} cors_ntrip_caster_t;

typedef struct cors_ntrip {
    uv_timer_t *timer_stat;
    uv_async_t *close;
    uv_thread_t thread;
    cors_ntrip_source_info_t *info_tbl[2];
    cors_ntrip_caster_t *ctr_tbl;
    int state;
    struct kdtree *src_kdtree;
    struct cors* cors;
} cors_ntrip_t;

typedef struct cors_rtcm {
    rtcm_t rtcm;
    UT_hash_handle hh;
} cors_rtcm_t;

typedef struct cors_rtcm_decoder {
    uv_thread_t thread;
    uv_mutex_t tbl_lock,qlock;
    uv_timer_t *timer_stat;
    uv_async_t *decode;
    uv_async_t *close;
    cors_rtcm_t *data_tbl;
    QUEUE decode_queue;
    int state;
    struct cors* cors;
} cors_rtcm_decoder_t;

typedef struct cors_obsd {
    obs_t obs;
    int srcid;
    UT_hash_handle hh;
} cors_obsd_t;

typedef struct cors_obs {
    cors_obsd_t *data;
} cors_obs_t;

typedef struct cors_nav {
    nav_t data;
} cors_nav_t;

typedef struct cors_sta {
    int srcid;
    sta_t sta;
    UT_hash_handle hh;
} cors_sta_t;

typedef struct cors_stas {
    cors_sta_t *data;
} cors_stas_t;

typedef struct cors_pnt {
    uv_thread_t thread;
    uv_mutex_t lock_tbl,qlock;
    uv_timer_t *timer_stat;
    uv_async_t *process;
    uv_async_t *close;
    prcopt_t opt;
    QUEUE pnt_queue;
    struct cors* cors;
} cors_pnt_t;

typedef struct cors_monitor_pnt {
    char user[32],passwd[32];
    char name[64];
    char adrr[32],mntpnt[32];
    char site[64];
    double pos[3];
    int port;
} cors_monitor_src_t;

typedef struct cors_monitor_bsta_distr {
    uv_mutex_t lock;
    uv_async_t *bsta_distr_moni;
    QUEUE queue;
} cors_monitor_bsta_distr_t;

typedef struct cors_monitord {
    uv_tcp_t *conn;
    cors_monitor_src_t m_src;
    struct cors_monitor *monitor;
    UT_hash_handle hh;
} cors_monitord_t;

typedef struct cors_monitor_src_q {
    char name[64];
    cors_monitord_t *md_tbl;
    UT_hash_handle hh;
} cors_monitor_src_q_t;

typedef struct cors_monitor_src_qs {
    uv_mutex_t qlock,lock;
    cors_monitor_src_q_t *q_tbl;
    uv_async_t *moni;
    QUEUE data_queue;
} cors_monitor_src_qs_t;

typedef struct cors_monitor_rtcm_msg {
    unsigned int n;
    int srcid;
    char msg[MAX_RTCM_MSG][256];
    UT_hash_handle hh;
} cors_monitor_rtcm_msg_t;

typedef struct cors_monitor_rtcm_msgs {
    cors_monitor_rtcm_msg_t *msg;
} cors_monitor_rtcm_msgs_t;

typedef struct cors_monitor_rtcm_sta {
    int srcid;
    sta_t sta;
    UT_hash_handle hh;
} cors_monitor_rtcm_sta_t;

typedef struct cors_monitor_rtcm_stas {
    cors_monitor_rtcm_sta_t *sta;
} cors_monitor_rtcm_stas_t;

typedef struct cors_monitor_rtcm {
    cors_monitor_rtcm_msgs_t msgs;
    cors_monitor_rtcm_stas_t stas;
} cors_monitor_rtcm_t;

typedef struct cors_navd {
    int srcid;
    cors_nav_t data;
    UT_hash_handle hh;
} cors_monitor_navd_t;

typedef struct cors_monitor_nav {
    cors_monitor_navd_t *data;
} cors_monitor_nav_t;

typedef struct cors_bsta_info {
    char id[32],address[256];
    char province[257],city[64];
    double pos[3];
    int itrf,type;
    UT_hash_handle hh;
    UT_hash_handle pp;
    UT_hash_handle vv;
    UT_hash_handle pr;
    UT_hash_handle prpp;
    UT_hash_handle prvv;
} cors_monitor_bsta_info_t;

typedef struct cors_bsta_info_province {
    char province[64];
    cors_monitor_bsta_info_t *data;
    cors_monitor_bsta_info_t *physic_stas;
    cors_monitor_bsta_info_t *virtual_stas;
    UT_hash_handle hh;
} cors_monitor_bsta_info_province_t;

typedef struct cors_bstas_info {
    cors_monitor_bsta_info_t *data;
    cors_monitor_bsta_info_t *physic_stas;
    cors_monitor_bsta_info_t *virtual_stas;
    cors_monitor_bsta_info_province_t *province_stas;
    struct kdtree *bstas_tree;
} cors_monitor_bstas_info_t;

typedef struct cors_monitor {
    uv_thread_t thread;
    uv_async_t *close;
    uv_async_t *delete_moni;
    uv_async_t *add_moni;
    uv_tcp_t *svr;
    uv_mutex_t dlock;
    cors_monitord_t *moni_tbl;
    cors_monitor_src_qs_t src_qs;
    cors_monitor_bsta_distr_t moni_bsta_distr;
    cors_monitor_rtcm_t moni_rtcm;
    cors_monitor_nav_t moni_nav;
    cors_monitor_bstas_info_t moni_bstas_info;
    char bstas_info_file[MAXSTRPATH];
    int port;
    struct cors* cors;
    QUEUE del_queue;
} cors_monitor_t;

typedef struct cors_opt {
    int monitor_port;
    char ntrip_sources_file[MAXSTRPATH];
    char trace_file[MAXSTRPATH];
    char baselines_file[MAXSTRPATH];
    char bstas_info_file[MAXSTRPATH];
    char vstas_file[MAXSTRPATH];
    char rtk_conf_file[MAXSTRPATH];
    char pnt_conf_file[MAXSTRPATH];
    char agent_user_file[MAXSTRPATH];
} cors_opt_t;

typedef struct cors_ssat {
    int srcid;
    gtime_t time;
    ssat_t ssat[MAXSAT];
    UT_hash_handle hh;
} cors_ssat_t;

typedef struct cors_ssats {
    cors_ssat_t *data;
} cors_ssats_t;

typedef struct cors_blsol {
    int base_srcid,rover_srcid;
    char id[16];
    rtk_t rtk;
    UT_hash_handle hh;
} cors_blsol_t;

typedef struct cors_blsols {
    cors_blsol_t *data;
} cors_blsols_t;

typedef struct cors_baseline {
    int base_srcid,rover_srcid,on;
    char id[16];
    uint64_t wt;
    gtime_t time;
    rtk_t rtk;
    cors_blsol_t *sol;
    UT_hash_handle hh;
} cors_baseline_t;

typedef struct cors_baselines {
    cors_baseline_t *data;
} cors_baselines_t;

typedef struct cors_srtk {
    uv_thread_t thread[2];
    int state,ID;

    uv_mutex_t addbl_lock;
    uv_mutex_t delbl_lock;
    uv_mutex_t rtkpos_lock;

    cors_baselines_t bls;
    struct cors* cors;
    struct cors_nrtk* nrtk;
    prcopt_t opt;
    QUEUE rtkpos_queue;
    QUEUE addbl_queue,delbl_queue;
    UT_hash_handle hh;
} cors_srtk_t;

typedef struct cors_dtrig_vertex_q {
    struct cors_dtrig_vertex *vt;
    UT_hash_handle hh;
} cors_dtrig_vertex_q_t;

typedef struct cors_dtrig_edge_q {
    struct cors_dtrig_edge *edge;
    UT_hash_handle hh;
} cors_dtrig_edge_q_t;

typedef struct cors_dtrig_vertex {
    int srcid;
    double pos[3];
    uint64_t wt;
    gtime_t time;
    cors_dtrig_vertex_q_t *vt_list;
    cors_dtrig_edge_q_t *edge_list;
    struct cors_vrs_sta_q *vsta_list;
    UT_hash_handle hh;
} cors_dtrig_vertex_t;
typedef cors_dtrig_vertex_t cors_master_sta_t;

typedef struct cors_dtrig_edge {
    char id[32];
    cors_dtrig_vertex_t *vt[2];
    cors_baseline_t *bl;
    UT_hash_handle hh;
} cors_dtrig_edge_t;

typedef struct cors_dtrig {
    char id[32];
    uint64_t wt;
    gtime_t time;
    cors_dtrig_vertex_t *vt[3];
    cors_dtrig_edge_t *edge[3];
    cors_dtrig_edge_t *edge_f[3][2];
    UT_hash_handle hh;
} cors_dtrig_t;
typedef cors_dtrig_t cors_bsta_trig_t;

typedef struct cors_dtrig_net {
    cors_dtrig_t *dtrigs;
    cors_dtrig_vertex_t *vertexs;
    cors_dtrig_edge_t *edges;
    struct kdtree *vts_kdtree;
    int state;
} cors_dtrig_net_t;

typedef struct cors_nrtk {
    uv_thread_t thread;
    uv_mutex_t addsrc_lock,delsrc_lock;
    uv_mutex_t addvsta_lock,delvsta_lock;
    uv_mutex_t addbl_lock,delbl_lock;
    uv_mutex_t updbl_lock;

    int state;
    cors_dtrig_net_t dtrig_net;
    cors_srtk_t *srtk;
    struct cors* cors;
    QUEUE addvsta_queue,delvsta_queue;
    QUEUE addsrc_queue,delsrc_queue;
    QUEUE addbl_queue,delbl_queue;
    QUEUE updbl_queue;
} cors_nrtk_t;

typedef struct cors_vrs_sta {
    int srcid,in_dtrig;
    char name[64];
    double pos[3];
    obs_t obs;
    FILE *fp_rnx;
    rnxopt_t rnx_opt;
    rtcm_t rtcm;
    cors_master_sta_t *msta;
    cors_bsta_trig_t *trig;
    UT_hash_handle hh;
} cors_vrs_sta_t;

typedef struct cors_vrs_sta_q {
    cors_vrs_sta_t *vsta;
    UT_hash_handle hh;
} cors_vrs_sta_q_t;

typedef struct cors_vrs_stas {
    cors_vrs_sta_t *data;
} cors_vrs_stas_t;

typedef struct cors_vrs {
    int state;
    uv_thread_t thread;
    uv_async_t *upd_vrs;
    uv_async_t *close;
    uv_mutex_t upd_vrs_lock;
    cors_vrs_stas_t stas;
    cors_nrtk_t *nrtk;
    struct cors* cors;
    QUEUE upd_vrs_queue;
} cors_vrs_t;

typedef struct cors_cli {
    uv_pipe_t *conn;
    UT_hash_handle hh;
} cors_cli_t;

typedef struct cors_ci {
    uv_pipe_t *svr;
    cors_cli_t *cli;
} cors_ci_t;

typedef struct cors {
    int state;
    uv_thread_t thread;
    uv_timer_t *timer_stat;
    uv_async_t *close;

    cors_rtcm_decoder_t rtcm_decoder;
    cors_ntrip_t ntrip;
    cors_ntrip_agent_t agent;
    cors_pnt_t pnt;
    cors_monitor_t monitor;
    cors_srtk_t srtk;
    cors_nrtk_t nrtk;
    cors_vrs_t vrs;
    cors_ci_t ci;
    cors_obs_t obs;
    cors_nav_t nav;
    cors_ssats_t ssats;
    cors_stas_t stas;
    cors_blsols_t blsols;
    cors_opt_t opt;
} cors_t;

typedef struct mcors {

} mcors_t;

EXPORT int cors_ntripcli_start(uv_loop_t *loop, cors_ntrip_client_t *cli, ntrip_cli_read_cb read_cb, const char *name,
                               const char *addr, int port, const char *user, const char *passwd,
                               const char *mntp, const double *pos);
EXPORT int cors_ntripcli_close(cors_ntrip_client_t *cli);

EXPORT int cors_ntrip_caster_start(cors_ntrip_caster_t *ctr, cors_ntrip_source_info_t *info_tbl);
EXPORT void cors_ntrip_caster_close(cors_ntrip_caster_t *ctr);
EXPORT int cors_ntrip_caster_add_source(cors_ntrip_caster_t *ctr, cors_ntrip_source_info_t *info);
EXPORT int cors_ntrip_caster_del_source(cors_ntrip_caster_t *ctr, const char *name);

EXPORT int cors_ntrip_start(cors_ntrip_t *ntrip, cors_t *cors, const char *sources_file);
EXPORT void cors_ntrip_close(cors_ntrip_t *ntrip);
EXPORT void cors_ntrip_add_source(cors_ntrip_t *ntrip, cors_ntrip_source_info_t *info);
EXPORT void cors_ntrip_del_source(cors_ntrip_t *ntrip, const char *name);
EXPORT void cors_ntrip_source_updpos(cors_ntrip_t *ntrip, const double *pos, int srcid);

EXPORT void cors_updnav(cors_nav_t *cors_nav, const nav_t *nav, int ephsat, int ephset);
EXPORT void cors_updobs(cors_obs_t *cors_obs, const obsd_t *obsd, int n, int srcid);
EXPORT int cors_start(cors_t* cors, const cors_opt_t *opt);
EXPORT void cors_updssat(cors_ssats_t *ssats, ssat_t *ssat, int srcid, int upd_flag, gtime_t time);
EXPORT void cors_updsta(cors_stas_t *stas, const sta_t *sta, int srcid);
EXPORT void cors_updblsol(cors_blsols_t *blsols, cors_baseline_t *bl, const rtk_t *rtk, int base_srcid, int rover_srcid);
EXPORT void cors_freenav(cors_nav_t *nav);
EXPORT void cors_freeobs(cors_obs_t *obs);
EXPORT void cors_freessat(cors_ssats_t *ssat);
EXPORT void cors_freeblsol(cors_blsols_t *blsols);
EXPORT void cors_close(cors_t* cors);
EXPORT void cors_freesta(cors_stas_t *stas);
EXPORT void cors_initssat(cors_ssats_t *ssat);
EXPORT void cors_initsta(cors_stas_t *stas);
EXPORT int cors_initnav(cors_nav_t *nav);
EXPORT void cors_initobs(cors_obs_t *obs);
EXPORT void cors_add_source(cors_t *cors, const char *name, const char *addr, int port, const char *user, const char *passwd,
                            const char *mntpnt, const double *pos);
EXPORT void cors_del_source(cors_t *cors, const char *name);
EXPORT void cors_add_baseline(cors_t *cors, const char *base, const char *rover);
EXPORT void cors_del_baseline(cors_t *cors, const char *base, const char *rover);
EXPORT void cors_basenet_del_baseline(cors_t *cors, const char *base, const char *rover);
EXPORT void cors_basenet_add_baseline(cors_t *cors, const char *base, const char *rover);

EXPORT int cors_rtcm_decoder_start(cors_rtcm_decoder_t *decoder, cors_t* cors);
EXPORT void cors_rtcm_decoder_close(cors_rtcm_decoder_t *decoder);
EXPORT int cors_rtcm_decode(cors_rtcm_decoder_t *decoder, const uint8_t *data, int n, int srcid);

EXPORT int cors_pnt_pos(cors_pnt_t *pnt, const obsd_t *obs, int n, int srcid);
EXPORT int cors_pnt_start(cors_pnt_t *pnt, cors_t* cors);
EXPORT void cors_pnt_close(cors_pnt_t *pnt);

EXPORT void cors_monitor_src(cors_monitor_t *monitor, const ssat_t *ssat, const sol_t *sol, const obs_t *obs, int srcid);
EXPORT void cors_monitor_close(cors_monitor_t *monitor);
EXPORT int cors_monitor_start(cors_monitor_t *monitor, cors_t *cors);
EXPORT int cors_monitor_add(cors_monitor_t *monitor);
EXPORT void cors_monitor_del(cors_monitor_t *monitor, cors_monitord_t *md);

EXPORT void cors_monitor_rtcm(cors_monitor_rtcm_t *moni_rtcm, const rtcm_t* rtcm, int srcid);
EXPORT void cors_monitor_freertcm(cors_monitor_rtcm_t *moni_rtcm);
EXPORT void cors_monitor_initrtcm(cors_monitor_rtcm_t *moni_rtcm);
EXPORT int cors_monitor_rtcm_msg(cors_monitor_rtcm_t *moni_rtcm, int srcid, char **msg_data);
EXPORT int cors_monitor_rtcm_sta(cors_monitor_rtcm_t *moni_rtcm, int srcid, sta_t *sta);

EXPORT void cors_monitor_nav(cors_monitor_nav_t *moni_nav, const nav_t *nav, int ephsat, int ephset, int srcid);
EXPORT void cors_monitor_initnav(cors_monitor_nav_t *moni_nav);
EXPORT void cors_monitor_freenav(cors_monitor_nav_t *moni_nav);

EXPORT int cors_srtk_start(cors_srtk_t *srtk, cors_t *cors, cors_nrtk_t *nrtk, const char *bls_file);
EXPORT int cors_srtk_close(cors_srtk_t *srtk);
EXPORT int cors_srtk_add_baseline(cors_srtk_t *srtk, int base_srcid, int rover_srcid);
EXPORT int cors_srtk_del_baseline(cors_srtk_t *srtk, int base_srcid, int rover_srcid);

EXPORT void cors_dtrignet_init(cors_dtrig_net_t *dtrig_net);
EXPORT void cors_dtrignet_free(cors_dtrig_net_t *dtrig_net);
EXPORT int cors_dtrignet_build(cors_dtrig_net_t *dtrig_net, cors_dtrig_edge_t **edge_add, cors_dtrig_edge_t **edge_del);
EXPORT int cors_dtrignet_add_vertex(cors_dtrig_net_t *dtrig_net, const double *pos, int srcid, cors_dtrig_edge_t **edge_add,
                                    cors_dtrig_edge_t **edge_del);
EXPORT void cors_dtrignet_upd_vertex(cors_dtrig_net_t *dtrig_net, const double *pos, int srcid);
EXPORT void cors_dtrignet_del_vertex(cors_dtrig_net_t *dtrig_net, int srcid, cors_dtrig_edge_t **edge_add,
                                     cors_dtrig_edge_t **edge_del);
EXPORT void cors_dtrignet_upd_edge(cors_dtrig_net_t *dtrig_net, cors_baseline_t *bl, int base_srcid, int rover_srcid);
EXPORT void cors_dtrignet_del_edge(cors_dtrig_net_t *dtrig_net, int srcid1, int srcid2);
EXPORT void cors_dtrignet_add_edge(cors_dtrig_net_t *dtrig_net, int srcid1, int srcid2);

EXPORT int cors_nrtk_start(cors_nrtk_t *nrtk, cors_t *cors);
EXPORT void cors_nrtk_close(cors_nrtk_t *nrtk);
EXPORT void cors_nrtk_upd_baseline(cors_nrtk_t *nrtk, cors_baseline_t *bl, int base_srcid, int rover_srcid);
EXPORT void cors_nrtk_del_baseline(cors_nrtk_t *nrtk, int base_srcid, int rover_srcid);
EXPORT void cors_nrtk_add_baseline(cors_nrtk_t *nrtk, int base_srcid, int rover_srcid);
EXPORT void cors_nrtk_add_source(cors_nrtk_t *nrtk, int srcid, const double *pos);
EXPORT void cors_nrtk_del_source(cors_nrtk_t *nrtk, int srcid);
EXPORT void cors_nrtk_add_vsta(cors_nrtk_t *nrtk, const char *name, const double *pos);
EXPORT void cors_nrtk_del_vsta(cors_nrtk_t *nrtk, const char *name);

EXPORT int cors_vrs_upd(cors_vrs_t *vrs, cors_vrs_sta_t *vsta, const cors_master_sta_t *msta, const obs_t *mobs, const rtk_t **rtk, int n);
EXPORT int cors_vrs_start(cors_vrs_t *vrs, cors_t *cors, cors_nrtk_t *nrtk, const char *vstas_file);
EXPORT void cors_vrs_close(cors_vrs_t *vrs);

EXPORT int cors_ntrip_agent_start(cors_ntrip_agent_t *agent, cors_ntrip_t *ntrip, const char *users_file);
EXPORT void cors_ntrip_agent_close(cors_ntrip_agent_t *agent);
EXPORT int cors_ntrip_agent_send(cors_ntrip_agent_t *agent, const char *mntpnt, const char *buff, int nb,
                                 const nav_t *nav);
EXPORT int cors_ntrip_agent_add_user(cors_ntrip_agent_t *agent, const char *user, const char *passwd);
EXPORT int cors_ntrip_agent_del_user(cors_ntrip_agent_t *agent, const char *user);

#ifdef __cplusplus
}
#endif
#endif