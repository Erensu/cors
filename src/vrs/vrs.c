/*------------------------------------------------------------------------------
 * vrs.c   : VRS functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

#define VRS_TROP_MAPFUNC         1
#define VRS_UPD_DTRIG            1
#define VRS_CHK_IN_DTRIG         1
#define VRS_OUT_OBSRNX           1
#define VRS_BLSOL_VSTA           1
#define VRS_HIGH_RESOLUTION      1
#define VRS_NEAREST_BL           0

typedef struct upd_vrs_task {
    obs_t mobs;
    rtk_t *rtk;
    int n,*dire;
    cors_vrs_sta_t *vsta;
    cors_vrs_t *vrs;
    cors_master_sta_t msta;
    QUEUE q;
} upd_vrs_task_t;

static int generate_vsta_id()
{
    static int mID=0;
    return -(++mID);
}

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

static void read_vstas_file(cors_vrs_t *vrs, const char *file)
{
    char buff[256],*p,*q,*val[16];
    cors_vrs_stas_t *stas=&vrs->stas;
    int n;
    FILE *fp;

    if (!(fp=fopen(file,"r"))) {
        log_trace(1,"read vrs stations file fail: %s\n",file);
        return;
    }
    while (fgets(buff,sizeof(buff),fp)) {
        for (n=0,p=buff;*p&&n<16;p=q+1) {
            if ((q=strchr(p,','))||(q=strchr(p,'#'))) {
                val[n++]=p; *q='\0';
            }
            else break;
        }
        if (n<4) continue;
        cors_vrs_sta_t *sta;

        HASH_FIND_STR(stas->data,val[0],sta);
        if (sta) continue;

        sta=calloc(1,sizeof(*sta));
        strcpy(sta->name,val[0]);
        sta->pos[0]=atof(val[1]);
        sta->pos[1]=atof(val[2]);
        sta->pos[2]=atof(val[3]);
        sta->srcid=generate_vsta_id();
        sta->obs.data=calloc(MAXOBS,sizeof(obsd_t));
        HASH_ADD_STR(stas->data,name,sta);
    }
    fclose(fp);
}

static int is_in_dtrig(const cors_dtrig_t *dtrig, const double *pos)
{
    double coord_p2x[3],coord_p2y[3],pc[3]={0};
    double pos0[3],dr[3],e[3];
    int i,j;

    for (i=0;i<3;i++) {
        for (j=0;j<3;j++) pc[i]+=dtrig->vt[j]->pos[i];
        pc[i]/=3.0;
    }
    ecef2pos(pc,pos0);

    for (i=0;i<3;i++) {
        for (j=0;j<3;j++) dr[j]=dtrig->vt[i]->pos[j]-pc[j];
        ecef2enu(pos0,dr,e);
        coord_p2x[i]=e[0];
        coord_p2y[i]=e[1];
    }
    for (i=0;i<3;i++) dr[i]=pos[i]-pc[i];
    ecef2enu(pos0,dr,e);
    return pnpoly(3,coord_p2x,coord_p2y,e[0],e[1]);
}

static cors_bsta_trig_t* find_bsta_trig(const cors_vrs_t *vrs, cors_vrs_sta_t *sta)
{
    cors_dtrig_net_t *dtrig_net=&vrs->nrtk->dtrig_net;
    cors_dtrig_t *d,*t,*m=NULL;
    double md=-1E9,pc[3],dr[3];
    int i,j;

    HASH_ITER(hh,dtrig_net->dtrigs,d,t) {
        if (!is_in_dtrig(d,sta->pos)) continue;
        sta->in_dtrig=1;
        return d;
    }
    HASH_ITER(hh,dtrig_net->dtrigs,d,t) {
        for (i=0;i<3;i++) {
            for (pc[i]=0.0,j=0;j<3;j++) pc[i]+=d->vt[j]->pos[i];
            pc[i]/=3.0;
        }
        for (i=0;i<3;i++) dr[i]=sta->pos[i]-pc[i];
        if (md<0||norm(dr,3)<md) {
            sta->in_dtrig=0; m=d;
        }
    }
    return m;
}

static cors_master_sta_t* find_master_bsta(const cors_dtrig_t *dtrig, cors_vrs_sta_t *sta)
{
    if (norm(sta->pos,3)<=0.0) return NULL;
    if (!dtrig) return NULL;
    int i,j,k;
    double dr[3],d=-1.0;

    for (k=-1,i=0;i<3;i++) {
        for (j=0;j<3;j++) dr[j]=sta->pos[j]-dtrig->vt[i]->pos[j];
        if (d<0.0||norm(dr,3)<d) {
            d=norm(dr,3); k=i;
        }
    }
    if (k<0) return NULL;

    cors_master_sta_t *msta=dtrig->vt[k];
    cors_vrs_sta_q_t *vq;
    HASH_FIND_PTR(msta->vsta_list,&sta,vq);
    if (!vq) {
        vq=calloc(1,sizeof(*vq));
        vq->vsta=sta;
        HASH_ADD_PTR(msta->vsta_list,vsta,vq);
    }
    return dtrig->vt[k];
}

static void upd_vrs_stas(cors_vrs_t *vrs)
{
    cors_vrs_sta_t *s,*t;

    HASH_ITER(hh,vrs->stas.data,s,t) {
        s->trig=find_bsta_trig(vrs,s);
        s->msta=find_master_bsta(s->trig,s);
    }
}

static int upd_vrs_coef(const cors_vrs_sta_t *vsta, const cors_master_sta_t *msta, rtk_t *rtk, int n, const int *dire,
                        int sat, int frq, double *c, double *Q)
{
    double *H,*v,*var,e[3],dr[3],pos[3],age=30.0;
    int i,j,k,m,info;

    memset(Q,0,sizeof(double)*9);
    memset(c,0,sizeof(double)*3);

#if VRS_CHK_IN_DTRIG
    if (!vsta->in_dtrig) return 0;
#endif
    if (n<2) return 0;
    ecef2pos(vsta->msta->pos,pos);

    H=mat(n+1,3); v=mat(n+1,1);
    var=mat(n+1,1);

    for (m=i=0;i<n;i++) {
        if (fabs(timediff(msta->time,rtk[i].time))>age) continue;
        if (!rtk[i].ssat[sat-1].vsat[frq]) continue;
        if ( rtk[i].ssat[sat-1].slip[frq]) continue;
        if ( rtk[i].ssat[sat-1].lflg[frq]) continue;
        if ( rtk[i].ssat[sat-1].fix[frq]<2) continue;
        if (rtk[i].ssat[sat-1].refsat[frq]!=rtk[0].ssat[sat-1].refsat[frq]) continue;
        if (norm(rtk[i].bl,3)<=0.0) continue;
        for (k=0;k<3;k++) dr[k]=dire[i]*rtk[i].bl[k];
        ecef2enu(pos,dr,e);
        H[3*m+0]=e[0];
        H[3*m+1]=e[1];
        H[3*m+2]=e[2];
        v[m]=dire[i]*rtk[i].ssat[sat-1].resc[frq];
        var[m++]=1.0;
    }
    if (m<=1) return 0;
    if (m<3) {
        H[3*m+0]=H[3*m+1]=0.0;
        H[3*m+2]=1.0; v[m]=0.0; var[m++]=1E-6;
    }
    for (j=0;j<m;j++) {
        v[j]/=sqrt(var[j]);
        for (k=0;k<3;k++) H[k+j*3]/=sqrt(var[j]);
    }
    info=lsq(H,v,3,m,c,Q);
    free(H); free(v); return !info;
}

static int generate_vrs_obs_sat(cors_vrs_t *vrs, const cors_vrs_sta_t *vsta, const cors_master_sta_t *msta, const double *coef,
                                const double *Q, int f, const obsd_t *mobs, obsd_t *vobs)
{
    double corr,dr[3],e[3],frq;
    double pm[3],pv[3],rm,rv,dtm,dtv,tropm,tropv,em[3],ev[3];
    double zazel[2]={0.0,90.0*D2R},azelm[2],azelv[2];
    const nav_t *nav=&vrs->cors->nav.data;
    int i;

    vobs->P[f]=vobs->L[f]=0.0;

    if (!mobs->L[f]||!mobs->P[f]) return 0;
    if (!(frq=sat2freq(vobs->sat,vobs->code[f],nav))) return 0;

    ecef2pos(msta->pos,pm);
    ecef2pos(vsta->pos,pv);
    for (i=0;i<3;i++) dr[i]=vsta->pos[i]-msta->pos[i];
    ecef2enu(pm,dr,e);

    corr=dot(coef,e,3);
    rm=satdis(msta->pos,vobs,nav,EPHOPT_BRDC,&dtm,em);
    rv=satdis(vsta->pos,vobs,nav,EPHOPT_BRDC,&dtv,ev);
    if (rm<=0.0||rv<=0.0) return 0;

    satazel(pm,em,azelm);
    satazel(pv,ev,azelv);

#if VRS_TROP_MAPFUNC
    tropm=tropmapf(vobs->time,pm,azelm,NULL)*tropmodel(vobs->time,pm,zazel,0.0);
    tropv=tropmapf(vobs->time,pv,azelv,NULL)*tropmodel(vobs->time,pv,zazel,0.0);
#else
    tropm=tropmodel(obs->time,posm,azelm,0.7);
    tropv=tropmodel(obs->time,posv,azelv,0.7);
#endif
    if (tropm<=0.0||tropv<=0.0) return 0;

    vobs->L[f]=mobs->L[f]+(rv-rm+tropv-tropm+(dtv-dtm)*CLIGHT+corr)/(CLIGHT/frq);
    vobs->P[f]=mobs->P[f]+(rv-rm+tropv-tropm+(dtv-dtm)*CLIGHT+corr);

    char tbuf[32];
    time2str(vobs->time,tbuf,3);
    log_trace(1,"vobs: time=%s sat=%4d f=%d el=%6.1lf corr=%6.3lf\n",tbuf,vobs->sat,
            f,azelv[1]*R2D,corr);
    return 1;
}

static int generate_vrs_obs_satf(cors_vrs_t *vrs, const cors_vrs_sta_t *vsta, const cors_master_sta_t *msta, rtk_t *rtk,
                                 int n, const obsd_t *mobs, const int *dire, obsd_t *vobs)
{
    double coef[3],Q[9];
    int f,flag;

    *vobs=*mobs;

    for (flag=f=0;f<NFREQ;f++) {
        if (upd_vrs_coef(vsta,msta,rtk,n,dire,vobs->sat,f,coef,Q)<0) continue;
        flag|=generate_vrs_obs_sat(vrs,vsta,msta,coef,Q,f,mobs,vobs);
    }
    return flag;
}

static int generate_vrs_obs(cors_vrs_t *vrs, cors_vrs_sta_t *vsta, const cors_master_sta_t *msta,
                            const obs_t *mobs, const int *dire, rtk_t *rtk, int n)
{
    double coef[3],Q[9];
    int i,f,flag;

    vsta->obs.n=0;

    for (i=0;i<mobs->n;i++) {
        if (generate_vrs_obs_satf(vrs,vsta,msta,rtk,n,&mobs->data[i],dire,
                &vsta->obs.data[vsta->obs.n])<=0) continue;
        vsta->obs.n++;
    }
    return vsta->obs.n;
}

static int cmprtk(const void *p1, const void *p2)
{
    rtk_t *q1=(rtk_t*)p1;
    rtk_t *q2=(rtk_t*)p2;
    return (timediff(q1->sol.time,q2->sol.time)<0.0)?-1:1;
}

static void upd_vrs_obs(cors_vrs_t *vrs, cors_vrs_sta_t *vsta, const cors_master_sta_t *msta, const obs_t *mobs,
                        const int *dire, rtk_t *rtk, int n)
{
    qsort(rtk,n,sizeof(rtk_t),cmprtk);
    generate_vrs_obs(vrs,vsta,msta,mobs,dire,rtk,n);

    cors_updobs(&vrs->cors->obs,
            vsta->obs.data,vsta->obs.n,vsta->srcid);
}

static upd_vrs_task_t* new_upd_vrs_task(cors_vrs_t *vrs, cors_vrs_sta_t *vsta, const cors_master_sta_t *msta,
                                        const obs_t *mobs, const int *dire, const rtk_t **rtk, int n)
{
    upd_vrs_task_t *task;
    int i;

    task=calloc(1,sizeof(*task));
    task->rtk=calloc(n,sizeof(rtk_t));
    task->msta=*msta;
    task->mobs.data=calloc(mobs->n,sizeof(obsd_t));
    task->mobs.n=mobs->n;
    task->vrs=vrs;
    task->dire=imat(1,n);
    task->n=n;
    task->vsta=vsta;

    memcpy(task->dire,dire,sizeof(int)*n);
    memcpy(task->mobs.data,mobs->data,sizeof(obsd_t)*mobs->n);
    for (i=0;i<n;i++) {
        if (!rtk[i]) continue;
        task->rtk[i]=*rtk[i];
    }
    return task;
}

static const rtk_t* vrs_near_bl(const cors_master_sta_t *msta, const cors_vrs_sta_t *vsta,
                                   const rtk_t **rtk, int n, int dire)
{
    int i,j=-1;
    double dr[3],pos[3],e[3],hv,hm,dh,dp=1E6;

    for (i=0;i<3;i++) dr[i]=vsta->pos[i]-msta->pos[i];
    ecef2pos(msta->pos,pos);
    ecef2enu(pos,dr,e);
    hv=atan2(e[0],e[1]);

    for (i=0;i<n;i++) {
        if (!rtk[i]) continue;
        if (norm(rtk[i]->bl,3)<=0.0) continue;
        ecef2enu(pos,rtk[i]->bl,e);
        hm=atan2(e[0],e[1]);

        if ((dh=dire*(hm-hv))<0.0) dh+=2.0*PI;
        if (j<0||dh<dp) j=i,dp=dh;
    }
    return j<0?NULL:rtk[j];
}

static int vrs_upd_dtrig(cors_vrs_t *vrs, cors_vrs_sta_t *vsta, const cors_master_sta_t *msta, const obs_t *mobs,
                         const rtk_t **rtk, int n)
{
    cors_dtrig_edge_q_t *q,*t;
    cors_dtrig_edge_t *e;
    const rtk_t *rtks[64],*rtkp;
    int i,j,k,m=0,dire[64]={0};

#if VRS_NEAREST_BL
    if (!(rtkp=vrs_near_bl(msta,vsta,rtk,n, 1))) rtks[m++]=rtkp;
    if (!(rtkp=vrs_near_bl(msta,vsta,rtk,n,-1))) rtks[m++]=rtkp;
#else
    for (j=-1,i=0;i<3;i++) {
        if (msta==vsta->trig->vt[i]) {j=i; break;}
    }
    if (j<0) return -1;

    for (i=m=0;i<2;i++) {
        e=vsta->trig->edge_f[j][i];
        k=0;
        HASH_ITER(hh,msta->edge_list,q,t) {
            if (q->edge!=e) {k++; continue;}
            if (strcmp(q->edge->id,q->edge->bl->id)) dire[m]=-1;
            else dire[m]=1;
            rtks[m++]=rtk[k++]; break;
        }
    }
    if (m<2) {
        m=0;
        if (!(rtkp=vrs_near_bl(msta,vsta,rtk,n, 1))) rtks[m++]=rtkp;
        if (!(rtkp=vrs_near_bl(msta,vsta,rtk,n,-1))) rtks[m++]=rtkp;
    }
#endif
    if (m<=0) return -1;

    uv_mutex_lock(&vrs->upd_vrs_lock);
    upd_vrs_task_t *task=new_upd_vrs_task(vrs,vsta,msta,mobs,dire,rtks,m);
    QUEUE_INSERT_TAIL(&vrs->upd_vrs_queue,&task->q);
    uv_mutex_unlock(&vrs->upd_vrs_lock);

    uv_async_send(vrs->upd_vrs);
}

static int vrs_upd_subnet(cors_vrs_t *vrs, cors_vrs_sta_t *vsta, const cors_master_sta_t *msta, const obs_t *mobs,
                          const rtk_t **rtk, int n)
{
    cors_dtrig_edge_q_t *q,*t;
    const rtk_t *rtks[64];
    int i,j,m,dire[64]={0};

    HASH_ITER(hh,msta->edge_list,q,t) {
        if (!rtk[i]) {i++; continue;}
        if (strcmp(q->edge->id,q->edge->bl->id)) dire[m]=-1;
        else dire[m]=1;
        rtks[m++]=rtk[i++];
    }
    for (i=m=0;i<n;i++) {
        if (rtk[i]) rtks[m++]=rtk[i];
    }
    if (m<=0) {
        return -1;
    }
    uv_mutex_lock(&vrs->upd_vrs_lock);
    upd_vrs_task_t *task=new_upd_vrs_task(vrs,vsta,msta,mobs,dire,rtks,m);
    QUEUE_INSERT_TAIL(&vrs->upd_vrs_queue,&task->q);
    uv_mutex_unlock(&vrs->upd_vrs_lock);

    uv_async_send(vrs->upd_vrs);
}

extern int cors_vrs_upd(cors_vrs_t *vrs, cors_vrs_sta_t *vsta, const cors_master_sta_t *msta, const obs_t *mobs,
                        const rtk_t **rtk, int n)
{
#if VRS_UPD_DTRIG
    return vrs_upd_dtrig(vrs,vsta,msta,mobs,rtk,n);
#else
    return vrs_upd_subnet(vrs,vsta,msta,mobs,rtk,n);
#endif
}

static void out_vrs_obsrnx(cors_vrs_t *vrs, cors_vrs_sta_t *vsta)
{
    char path[256],file[128],*tfmt="%Y_%m_%d_%h_%M_%S";
    const nav_t *nav=&vrs->cors->nav.data;

    if (!vsta->fp_rnx) {
        sprintf(file,".%c%s_%s.rnx",FILEPATHSEP,tfmt,vsta->name);
        reppath(file,path,timeget(),"","");

        if (!(vsta->fp_rnx=fopen(path,"w"))) {
            log_trace(1,"open vrs obsrnx file fail\n");
            return;
        }
    }
    if (!vsta->rnx_opt.rnxver&&vsta->obs.n) {
        init_rnxopt(&vsta->rnx_opt,vsta->obs.data,vsta->obs.n);
        matcpy(vsta->rnx_opt.apppos,vsta->pos,1,3);
        outrnxobsh(vsta->fp_rnx,&vsta->rnx_opt,nav);
    }
    outrnxobsb(vsta->fp_rnx,&vsta->rnx_opt,vsta->obs.data,vsta->obs.n,0);
}

static void out_vrs_obsrtcm(cors_vrs_t *vrs, cors_vrs_sta_t *vsta)
{
#if VRS_HIGH_RESOLUTION
    int nb,type[5]={1076,1086,1096,1126,1116};
#else
    int nb,type[5]={1074,1084,1094,1124,1114};
#endif
    char buff[MAXOBS*256];
    cors_ntrip_agent_t *agent=&vrs->cors->agent;
    nav_t *nav=&vrs->cors->nav.data;

    if ((nb=rtcm_encode_obs(&vsta->rtcm,type,5,nav,vsta->obs.data,vsta->obs.n,buff))) {
        cors_ntrip_agent_send(agent,vsta->name,buff,nb,nav);
    }
}

static void do_upd_vrs_work(upd_vrs_task_t *task)
{
    upd_vrs_obs(task->vrs,task->vsta,&task->msta,&task->mobs,task->dire,task->rtk,task->n);

#if VRS_OUT_OBSRNX
    out_vrs_obsrnx(task->vrs,task->vsta);
#endif
    out_vrs_obsrtcm(task->vrs,task->vsta);

    freeobs(&task->mobs);
    free(task->rtk); free(task);
}

static void upd_vrs_process(uv_async_t* handle)
{
    cors_vrs_t *vrs=handle->data;

    while (!QUEUE_EMPTY(&vrs->upd_vrs_queue)) {
        uv_mutex_lock(&vrs->upd_vrs_lock);

        QUEUE *q=QUEUE_HEAD(&vrs->upd_vrs_queue);
        upd_vrs_task_t *task=QUEUE_DATA(q,upd_vrs_task_t,q);
        QUEUE_REMOVE(q);
        uv_mutex_unlock(&vrs->upd_vrs_lock);
        do_upd_vrs_work(task);
    }
}

static void vrs_blsol_vsta(cors_vrs_t *vrs, const cors_vrs_sta_t *vsta)
{
    cors_dtrig_vertex_t *vt;
    cors_srtk_t *srtk=&vrs->cors->srtk;
    int i;

    for (i=0;i<3;i++) {
        if (!(vt=vsta->trig->vt[i])) continue;
        cors_srtk_add_baseline(srtk,vt->srcid,vsta->srcid);
    }
}

static void close_cb(uv_async_t* handle)
{
    if (uv_loop_alive(handle->loop)) {
        uv_stop(handle->loop);
    }
}

static void init_vrs(cors_vrs_t *vrs, cors_t *cors, cors_nrtk_t *nrtk, const char *vfile)
{
    QUEUE_INIT(&vrs->upd_vrs_queue);
    uv_mutex_init(&vrs->upd_vrs_lock);

    vrs->cors=cors;
    vrs->nrtk=nrtk;
    read_vstas_file(vrs,vfile);
    upd_vrs_stas(vrs);

    cors_ntrip_source_info_t *info;
    cors_vrs_sta_t *s,*t;

    HASH_ITER(hh,vrs->stas.data,s,t) {
        info=calloc(1,sizeof(*info));
        info->type=1;
        strcpy(info->name,s->name);
        matcpy(info->pos,s->pos,1,3);
        cors_ntrip_add_source(&cors->ntrip,info);
    }
#if VRS_BLSOL_VSTA
    HASH_ITER(hh,vrs->stas.data,s,t) {
        vrs_blsol_vsta(vrs,s);
    }
#endif
}

static void vrs_thread(void *vrs_arg)
{
    cors_vrs_t *vrs=vrs_arg;
    uv_loop_t *loop=uv_loop_new();

    vrs->upd_vrs=calloc(1,sizeof(uv_async_t));
    vrs->upd_vrs->data=vrs;
    uv_async_init(loop,vrs->upd_vrs,upd_vrs_process);

    vrs->close=calloc(1,sizeof(uv_async_t));
    uv_async_init(loop,vrs->close,close_cb);

    vrs->state=1;
    uv_run(loop,UV_RUN_DEFAULT);

    close_uv_loop(loop);
    free(loop);
}

extern int cors_vrs_start(cors_vrs_t *vrs, cors_t *cors, cors_nrtk_t *nrtk, const char *vfile)
{
    init_vrs(vrs,cors,nrtk,vfile);

    if (uv_thread_create(&vrs->thread,vrs_thread,vrs)) {
        log_trace(1,"vrs thread create fail\n");
        return 0;
    }
    log_trace(1,"vrs thread create ok\n");
    return 1;
}

extern void cors_vrs_close(cors_vrs_t *vrs)
{
    vrs->state=0;
    uv_async_send(vrs->close);
    uv_thread_join(&vrs->thread);
    vrs->cors=NULL;
    vrs->nrtk=NULL;

    cors_vrs_sta_t *s,*t;
    HASH_ITER(hh,vrs->stas.data,s,t) {
        freeobs(&s->obs);
        if (s->fp_rnx) fclose(s->fp_rnx);
        HASH_DEL(vrs->stas.data,s);
        free(s);
    }
}

extern int vrs_add_vsta(cors_vrs_t *vrs, const char *name, const double *pos)
{
    cors_ntrip_source_info_t *info;
    cors_vrs_sta_t *sta;
    cors_t *cors=vrs->cors;

    HASH_FIND_STR(vrs->stas.data,name,sta);
    if (sta) return 0;
    if (norm(pos,3)<=0) return 0;

    sta=calloc(1,sizeof(cors_vrs_sta_t));
    strcpy(sta->name,name);
    matcpy(sta->pos,pos,1,3);
    sta->srcid=generate_vsta_id();
    sta->obs.data=calloc(MAXOBS,sizeof(obsd_t));

    sta->trig=find_bsta_trig(vrs,sta);
    sta->msta=find_master_bsta(sta->trig,sta);
    HASH_ADD_STR(vrs->stas.data,name,sta);

#if VRS_BLSOL_VSTA
    vrs_blsol_vsta(vrs,sta);
#endif
    info=calloc(1,sizeof(cors_ntrip_source_info_t));
    info->type=1;
    strcpy(info->name,sta->name);
    matcpy(info->pos,sta->pos,1,3);
    cors_ntrip_add_source(&cors->ntrip,info);
    return 1;
}

extern int vrs_del_vsta(cors_vrs_t *vrs, const char *name)
{
    cors_t *cors=vrs->cors;
    cors_vrs_sta_q_t *vq;
    cors_vrs_sta_t *vsta;

    HASH_FIND_STR(vrs->stas.data,name,vsta);
    if (!vsta) return 0;

#if VRS_BLSOL_VSTA
    cors_dtrig_vertex_t *vt;
    cors_srtk_t *srtk=&vrs->cors->srtk;
    int i;

    for (i=0;i<3;i++) {
        if (!(vt=vsta->trig->vt[i])) continue;
        cors_srtk_del_baseline(srtk,vt->srcid,vsta->srcid);
    }
#endif
    HASH_FIND_PTR(vsta->msta->vsta_list,&vsta,vq);
    if (vq) {HASH_DEL(vsta->msta->vsta_list,vq); free(vq);}
    HASH_DEL(vrs->stas.data,vsta);
    freeobs(&vsta->obs);
    if (vsta->fp_rnx) fclose(vsta->fp_rnx);

    cors_ntrip_del_source(&cors->ntrip,name);
    free(vsta); return 1;
}

extern void vrs_upd_vsta(cors_vrs_t *vrs)
{
    return upd_vrs_stas(vrs);
}
