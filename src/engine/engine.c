/*------------------------------------------------------------------------------
 * engine.c: engine functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"
#include "vt.h"
#include "options.h"

#define CMDPROMPT   "cors-engine> "
#define MAXCMD      256
#define MAXSTR      1024
#define MAXCON      32
#define MAXARG      10
#define TRACEFILE   "cors_engine_%Y%m%d%h%M.trace"

#define SQRT(x)     ((x)<=0.0||(x)!=(x)?0.0:sqrt(x))

typedef struct {
    int state;
    vt_t *vt;
    uv_thread_t thread;
} con_t;

static cors_t cors;
static cors_opt_t cors_opt;

static int intflg=0;

static void sigshut(int sig)
{
    intflg=1;
}

static int startcors(vt_t *vt)
{
    if (cors_start(&cors,&cors_opt)>0) return 1;
    vt_printf(vt,"cors server start error\n");
    return 0;
}

static void stopcors(vt_t *vt)
{
    if (!cors.state) {
        vt_printf(vt,"cors server has not been started\n");
        return;
    }
    cors_close(&cors);
    vt_printf(vt,"stop cors server\n");
}

static void cmd_start(char **args, int narg, vt_t *vt)
{
    if (cors.state) {
        vt_printf(vt,"cors server already start\n");
        return;
    }
    if (!startcors(vt)) return;
    vt_printf(vt,"cors server start\n");
}

static void cmd_stop(char **args, int narg, vt_t *vt)
{
    stopcors(vt);
}

static void cmd_add_source(char **args, int narg, vt_t *vt)
{
    if (!cors.state) {
        vt_printf(vt,"cors server has not been started\n");
        return;
    }
    if (narg<7) return;
    if (inet_addr(args[2])==INADDR_NONE) return;

    double pos[3];
    cors_ntrip_source_info_t *info=calloc(1,sizeof(*info));
    strcpy(info->name,args[1]);
    strcpy(info->addr,args[2]);
    strcpy(info->user,args[4]);
    strcpy(info->passwd,args[5]);
    strcpy(info->mntpnt,args[6]);
    info->port=atoi(args[3]);

    if (narg>=9) {
        pos[0]=atof(args[7])*D2R;
        pos[1]=atof(args[8])*D2R;
        pos[2]=atof(args[9]);
        pos2ecef(pos,info->pos);
    }
    cors_ntrip_add_source(&cors.ntrip,info);
    cors_nrtk_add_source(&cors.nrtk,info->ID,info->pos);
    vt_printf(vt,"\n");
}

static void cmd_delete_source(char **args, int narg, vt_t *vt)
{
    if (!cors.state) {
        vt_printf(vt,"cors server has not been started\n");
        return;
    }
    if (narg<2) return;

    cors_ntrip_source_info_t *s;
    HASH_FIND_STR(cors.ntrip.info_tbl[0],args[1],s);
    if (!s) return;
    cors_nrtk_del_source(&cors.nrtk,s->ID);
    cors_ntrip_del_source(&cors.ntrip,args[1]);
    vt_printf(vt,"\n");
}

static void cmd_loadopt(char **args, int narg, vt_t *vt)
{
    if (narg<2) return;
    cors_loadopts(&cors_opt,args[1]);
    vt_printf(vt,"\n");
}

static void prnavidata(vt_t *vt)
{
    eph_t eph[MAXSAT];
    geph_t geph[MAXPRNGLO];
    double ion[8],utc[8];
    gtime_t time;
    char id[32],s1[64],s2[64],s3[64];
    int i,valid,prn;

    time=utc2gpst(timeget());
    for (i=0;i<MAXSAT;i++) eph[i]=cors.nav.data.eph[i];
    for (i=0;i<MAXPRNGLO;i++) geph[i]=cors.nav.data.geph[i];
    for (i=0;i<8;i++) ion[i]=cors.nav.data.ion_gps[i];
    for (i=0;i<8;i++) utc[i]=cors.nav.data.utc_gps[i];

    vt_printf(vt,"\n%3s %3s %3s %3s %3s %3s %3s %19s %19s %19s %3s %3s\n",
              "SAT","S","IOD","IOC","FRQ","A/A","SVH","Toe","Toc",
              "Ttr/Tof","L2C","L2P");
    for (i=0;i<MAXSAT;i++) {
        if (!(satsys(i+1,&prn)&(SYS_GPS|SYS_GAL|SYS_QZS|SYS_CMP))||
            eph[i].sat!=i+1) continue;
        valid=eph[i].toe.time!=0&&!eph[i].svh&&
              fabs(timediff(time,eph[i].toe))<=MAXDTOE;
        satno2id(i+1,id);
        if (eph[i].toe.time!=0) time2str(eph[i].toe,s1,0); else strcpy(s1,"-");
        if (eph[i].toc.time!=0) time2str(eph[i].toc,s2,0); else strcpy(s2,"-");
        if (eph[i].ttr.time!=0) time2str(eph[i].ttr,s3,0); else strcpy(s3,"-");
        vt_printf(vt,"%3s %3s %3d %3d %3d %3d %03X %19s %19s %19s %3d %3d\n",
                  id,valid?"OK":"-",eph[i].iode,eph[i].iodc,0,eph[i].sva,
                  eph[i].svh,s1,s2,s3,eph[i].code,eph[i].flag);
    }
    for (i=0;i<MAXSAT;i++) {
        if (!(satsys(i+1,&prn)&SYS_GLO)||geph[prn-1].sat!=i+1) continue;
        valid=geph[prn-1].toe.time!=0&&!geph[prn-1].svh&&
              fabs(timediff(time,geph[prn-1].toe))<=MAXDTOE_GLO;
        satno2id(i+1,id);
        if (geph[prn-1].toe.time!=0) time2str(geph[prn-1].toe,s1,0); else strcpy(s1,"-");
        if (geph[prn-1].tof.time!=0) time2str(geph[prn-1].tof,s2,0); else strcpy(s2,"-");
        vt_printf(vt,"%3s %3s %3d %3d %3d %3d  %02X %19s %19s %19s %3d %3d\n",
                  id,valid?"OK":"-",geph[prn-1].iode,0,geph[prn-1].frq,
                  geph[prn-1].age,geph[prn].svh,s1,"-",s2,0,0);
    }
    vt_printf(vt,"ION: %9.2E %9.2E %9.2E %9.2E %9.2E %9.2E %9.2E %9.2E\n",
              ion[0],ion[1],ion[2],ion[3],ion[4],ion[5],ion[6],ion[7]);
    vt_printf(vt,"UTC: %9.2E %9.2E %9.2E %9.2E  LEAPS: %.0f\n",utc[0],utc[1],
              utc[2],utc[3],utc[4]);
}

static void cmd_navidata(char **args, int narg, vt_t *vt)
{
    if (!cors.state) return;

    int cycle=0;
    if (narg>1) cycle=(int)(atof(args[1])*1000.0);

    while (!vt_chkbrk(vt)) {
        prnavidata(vt);
        if (cycle>0) sleepms(cycle); else return;
    }
    vt_printf(vt,"\n");
}

static int cors_getobs(const char *name, obsd_t *obs)
{
    cors_ntrip_source_info_t *s,*t;
    cors_obsd_t *o;
    int nobs;
    HASH_FIND_STR(cors.ntrip.info_tbl[0],name,s);
    if (!s) return 0;

    HASH_FIND_INT(cors.obs.data,&s->ID,o);

    if (!o) return -1;
    nobs=o->obs.n;
    memcpy(obs,o->obs.data,sizeof(obsd_t)*nobs);
    return nobs;
}

static void probserv(vt_t *vt, int nf, const char *name)
{
    obsd_t obs[MAXOBS*2];
    char tstr[64],id[32];
    int i,j,n=0,frq[]={1,2,5,7,8,6,9};

    if ((n=cors_getobs(name,obs))<=0) {
        vt_printf(vt,"no observation data: %s\n",name);
        return;
    }
    if (nf<=0||nf>NFREQ) nf=NFREQ;
    vt_printf(vt,"\n%-22s %3s","      TIME(GPST)","SAT");
    for (i=0;i<nf;i++) vt_printf(vt,"        P%d(m)" ,frq[i]);
    for (i=0;i<nf;i++) vt_printf(vt,"       L%d(cyc)",frq[i]);
    for (i=0;i<nf;i++) vt_printf(vt,"  D%d(Hz)"      ,frq[i]);
    for (i=0;i<nf;i++) vt_printf(vt," S%d"           ,frq[i]);
    vt_printf(vt," LLI\n");
    for (i=0;i<n;i++) {
        time2str(obs[i].time,tstr,2);
        satno2id(obs[i].sat,id);
        vt_printf(vt,"%s %3s",tstr,id);
        for (j=0;j<nf;j++) vt_printf(vt,"%13.3f",obs[i].P[j]);
        for (j=0;j<nf;j++) vt_printf(vt,"%14.3f",obs[i].L[j]);
        for (j=0;j<nf;j++) vt_printf(vt,"%8.1f" ,obs[i].D[j]);
        for (j=0;j<nf;j++) vt_printf(vt,"%3.0f" ,obs[i].SNR[j]*SNR_UNIT);
        for (j=0;j<nf;j++) vt_printf(vt,"%2d"   ,obs[i].LLI[j]);
        vt_printf(vt,"\n");
    }
}

static void cmd_observ(char **args, int narg, vt_t *vt)
{
    int i,nf=2,cycle=0;

    if (!cors.state) {
        vt_printf(vt,"cors server has not been started\n");
        return;
    }
    if (narg<3) return;

    for (i=2;i<narg;i++) {
        if (sscanf(args[i],"-%d",&nf)<1) cycle=(int)(atof(args[i])*1000.0);
    }
    while (!vt_chkbrk(vt)) {
        probserv(vt,nf,args[1]);
        if (cycle>0) sleepms(cycle); else return;
    }
    vt_printf(vt,"\n");
}

static void prsatellite(vt_t *vt, int nf, const char *name)
{
    ssat_t ssat[MAXSAT];
    double az,el;
    char id[32];
    int i,j,fix,frq[]={1,2,5,7,8,6};

    cors_ntrip_source_info_t *s;
    HASH_FIND_STR(cors.ntrip.info_tbl[0],name,s);
    if (!s) return;

    cors_ssat_t *t;
    HASH_FIND_INT(cors.ssats.data,&s->ID,t);
    if (!t) {
        vt_printf(vt,"no satellite data: %s\n",name);
        return;
    }
    memcpy(ssat,t->ssat,sizeof(ssat_t)*MAXSAT);

    if (nf<=0||nf>NFREQ) nf=NFREQ;
    vt_printf(vt,"\n%3s %2s %5s %4s","SAT","C1","Az","El");
    for (j=0;j<nf;j++) vt_printf(vt," L%d"    ,frq[j]);
    for (j=0;j<nf;j++) vt_printf(vt,"  Fix%d" ,frq[j]);
    for (j=0;j<nf;j++) vt_printf(vt,"  P%dRes",frq[j]);
    for (j=0;j<nf;j++) vt_printf(vt,"   L%dRes",frq[j]);
    for (j=0;j<nf;j++) vt_printf(vt,"  Sl%d"  ,frq[j]);
    for (j=0;j<nf;j++) vt_printf(vt,"  Lock%d",frq[j]);
    for (j=0;j<nf;j++) vt_printf(vt," Rj%d"   ,frq[j]);
    vt_printf(vt,"\n");

    for (i=0;i<MAXSAT;i++) {
        if (ssat[i].azel[1]<=0.0) continue;
        satno2id(i+1,id);
        vt_printf(vt,"%3s %2s",id,ssat[i].vs?"OK":"-");
        az=ssat[i].azel[0]*R2D; if (az<0.0) az+=360.0;
        el=ssat[i].azel[1]*R2D;
        vt_printf(vt," %5.1f %4.1f",az,el);
        for (j=0;j<nf;j++) vt_printf(vt," %2s",ssat[i].vsat[j]?"OK":"-");
        for (j=0;j<nf;j++) {
            fix=ssat[i].fix[j];
            vt_printf(vt," %5s",fix==1?"FLOAT":(fix==2?"FIX":(fix==3?"HOLD":"-")));
        }
        for (j=0;j<nf;j++) vt_printf(vt,"%7.3f",ssat[i].resp[j]);
        for (j=0;j<nf;j++) vt_printf(vt,"%8.4f",ssat[i].resc[j]);
        for (j=0;j<nf;j++) vt_printf(vt," %4d",ssat[i].slipc[j]);
        for (j=0;j<nf;j++) vt_printf(vt," %6d",ssat[i].lock [j]);
        for (j=0;j<nf;j++) vt_printf(vt," %3d",ssat[i].rejc [j]);
        vt_printf(vt,"\n");
    }
}

static void cmd_satellite(char **args, int narg, vt_t *vt)
{
    int i,nf=2,cycle=0;

    if (!cors.state) {
        vt_printf(vt,"cors server has not been started\n");
        return;
    }
    if (narg<3) return;

    for (i=2;i<narg;i++) {
        if (sscanf(args[i],"-%d",&nf)<1) cycle=(int)(atof(args[i])*1000.0);
    }
    while (!vt_chkbrk(vt)) {
        prsatellite(vt,nf,args[1]);
        if (cycle>0) sleepms(cycle); else return;
    }
    vt_printf(vt,"\n");
}

static void prsourceinfo_prc(vt_t *vt, cors_ntrip_source_info_t *s)
{
    char pos_str[64]={0};


    if (norm(s->pos,3)) {
        sprintf(pos_str,"%8.3lf,%8.3lf,%8.3lf",s->pos[0],s->pos[1],s->pos[2]);
    }
    vt_printf(vt, "%8s %4d %8s %16s %6d %8s %13s %12s %12s %40s\n",s->name,s->ID,"NTRIP",
              s->addr,s->port,s->user,s->passwd,s->mntpnt,
              s->src?(s->src->cli.state?"connected":"disconnected"):"---",
              strcmp(pos_str,"")==0?"---":pos_str);
}

static void prsourceinfo(vt_t *vt, const char *name)
{
    cors_ntrip_source_info_t *s,*t;
    if (HASH_COUNT(cors.ntrip.info_tbl[0])<=0) return;

    vt_printf(vt,"\n%8s %4s %8s %16s %6s %8s %13s %12s %12s %40s\n","Name","ID","Type","IP","Port",
            "User","Password","Mountpoint","Status","ECEF-XYZ/m");

    if (strcmp(name,"all")==0) {
        HASH_ITER(hh,cors.ntrip.info_tbl[0],s,t) {
            prsourceinfo_prc(vt,s);
        }
        vt_printf(vt,"\n");
        return;
    }
    HASH_FIND_STR(cors.ntrip.info_tbl[0],name,s);
    if (!s) return;
    prsourceinfo_prc(vt,s);
    vt_printf(vt,"\n");
}

static void cmd_sourceinfo(char **args, int narg, vt_t *vt)
{
    if (!cors.state) {
        vt_printf(vt,"cors server has not been started\n");
        return;
    }
    if (narg<2) return;
    prsourceinfo(vt,args[1]);
}

static void prmonirtcm_sta(vt_t *vt, const char *name)
{
    cors_ntrip_source_info_t *s,*t;
    sta_t sta;

    if (strcmp(name,"all")==0) {
        if (HASH_COUNT(cors.ntrip.info_tbl[0])<=0) return;
        vt_printf(vt,"%4s %8s %8s %8s %8s %8s  %4s %40s\n","Name","Antdes","Antsno","Rectype",
                "Recver","Recsno","ITRF","ECEF-XYZ/m");

        HASH_ITER(hh,cors.ntrip.info_tbl[0],s,t) {
            if (cors_monitor_rtcm_sta(&cors.monitor.moni_rtcm,s->ID,&sta)<=0) {
                vt_printf(vt,"no source rtcm data: %s\n",s->name);
                continue;
            }
            char pos_str[64];
            sprintf(pos_str,"%8.3lf,%8.3lf,%8.3lf",sta.pos[0],sta.pos[1],sta.pos[2]);

            vt_printf(vt,"%4s %8s %8s %8s %8s %8s  %04d %40s\n",
                    s->name,sta.antdes,sta.antsno,sta.rectype,sta.recver,sta.recsno,
                    sta.itrf,pos_str);
        }
        vt_printf(vt,"\n");
        return;
    }
    HASH_FIND_STR(cors.ntrip.info_tbl[0],name,s);
    if (!s) {
        vt_printf(vt,"no source: %s\n",name);
        return;
    }
    if (cors_monitor_rtcm_sta(&cors.monitor.moni_rtcm,s->ID,&sta)<=0) {
        vt_printf(vt,"no source rtcm data: %s\n",name);
        return;
    }
    vt_printf(vt,"%4s %8s %8s %8s %8s %8s  %4s %40s\n","Name","Antdes","Antsno","Rectype",
            "Recver","Recsno","ITRF","ECEF-XYZ/m");

    char pos_str[64];
    sprintf(pos_str,"%8.3lf,%8.3lf,%8.3lf",sta.pos[0],sta.pos[1],sta.pos[2]);

    vt_printf(vt,"%4s %8s %8s %8s %8s %8s  %04d %40s\n",name,
            sta.antdes,sta.antsno,sta.rectype,sta.recver,sta.recsno,
            sta.itrf,pos_str);

    vt_printf(vt,"\n");
}

static void prmonirtcm_msg(vt_t *vt, const char *name)
{
    cors_ntrip_source_info_t *s;
    HASH_FIND_STR(cors.ntrip.info_tbl[0],name,s);
    if (!s) {
        vt_printf(vt,"no source: %s\n",name);
        return;
    }
    int i;
    char **msg=calloc(32,sizeof(*msg));
    for (i=0;i<32;i++) msg[i]=calloc(256,sizeof(char));

    sta_t sta;
    if (cors_monitor_rtcm_msg(&cors.monitor.moni_rtcm,s->ID,msg)<=0) {
        vt_printf(vt,"no source rtcm data: %s\n",name);
        return;
    }
    for (i=0;i<32;i++) {
        if (strcmp(msg[i],"")==0) continue;
        vt_printf(vt,"%s\n",msg[i]);
    }
    for (i=0;i<32;i++) free(msg[i]); free(msg);
    vt_printf(vt,"\n");
}

static void cmd_monirtcm(char **args, int narg, vt_t *vt)
{
    int i,cycle=0;

    if (!cors.state) {
        vt_printf(vt,"cors server has not been started\n");
        return;
    }
    if (narg<3) return;
    if (narg>3) {
        cycle=(int)(atof(args[3])*1000.0);
    }
    while (!vt_chkbrk(vt)) {
        if (!strcmp(args[1],"-sta")) prmonirtcm_sta(vt,args[2]);
        if (!strcmp(args[1],"-msg")) prmonirtcm_msg(vt,args[2]);
        if (cycle>0) sleepms(cycle); else return;
    }
    vt_printf(vt,"\n");
}

static void cmd_rtkpos_addbl(char **args, int narg, vt_t *vt)
{
    char rover[32],base[32];
    int i;

    for (i=2;i<narg;i++) {
        if      (!strcmp(args[i],"-r")&&i+1<narg) strcpy(rover,args[++i]);
        else if (!strcmp(args[i],"-b")&&i+1<narg) strcpy(base,args[++i]);
    }
    cors_ntrip_source_info_t *r,*b;
    HASH_FIND_STR(cors.ntrip.info_tbl[0],rover,r);
    HASH_FIND_STR(cors.ntrip.info_tbl[0],base,b);

    if (!r||!b) return;
    cors_srtk_add_baseline(&cors.srtk,b->ID,r->ID);
}

static void cmd_rtkpos_delbl(char **args, int narg, vt_t *vt)
{
    char rover[32],base[32];
    int i;

    for (i=2;i<narg;i++) {
        if      (!strcmp(args[i],"-r")&&i+1<narg) strcpy(rover,args[++i]);
        else if (!strcmp(args[i],"-b")&&i+1<narg) strcpy(base,args[++i]);
    }
    cors_ntrip_source_info_t *r,*b;
    HASH_FIND_STR(cors.ntrip.info_tbl[0],rover,r);
    HASH_FIND_STR(cors.ntrip.info_tbl[0],base,b);

    if (!r||!b) return;
    cors_srtk_del_baseline(&cors.srtk,b->ID,r->ID);
}

static void prtime(vt_t *vt, int timetype, gtime_t time)
{
    double tow;
    int week;
    char tstr[64]="";

    if (timetype==1) {
        time2str(gpst2utc(time),tstr,2);
    }
    else if (timetype==2) {
        time2str(timeadd(gpst2utc(time),9*3600.0),tstr,2);
    }
    else if (timetype==3) {
        tow=time2gpst(time,&week); sprintf(tstr,"  %04d %9.2f",week,tow);
    }
    else time2str(time,tstr,1);
    vt_printf(vt,"%s ",tstr);
}

static void prsolution(vt_t *vt, const char *rover, const char *base, const sol_t *sol, const double *rb,
                       int timetype, int soltype)
{
    const char *solstr[]={"------","FIX","FLOAT","SBAS","DGPS","SINGLE","PPP",""};
    double pos[3]={0},Qr[9],Qe[9]={0},dms1[3]={0},dms2[3]={0},bl[3]={0};
    double enu[3]={0},pitch=0.0,yaw=0.0,len;
    int i;

    if (sol->time.time==0||!sol->stat) return;
    vt_printf(vt,"%6s ->%6s  ",base,rover);
    prtime(vt,timetype,sol->time);
    vt_printf(vt,"(%-6s)",solstr[sol->stat]);

    if (norm(sol->rr,3)>0.0&&norm(rb,3)>0.0) {
        for (i=0;i<3;i++) bl[i]=sol->rr[i]-rb[i];
    }
    len=norm(bl,3);
    Qr[0]=sol->qr[0];
    Qr[4]=sol->qr[1];
    Qr[8]=sol->qr[2];
    Qr[1]=Qr[3]=sol->qr[3];
    Qr[5]=Qr[7]=sol->qr[4];
    Qr[2]=Qr[6]=sol->qr[5];

    if (soltype==0) {
        if (norm(sol->rr,3)>0.0) {
            ecef2pos(sol->rr,pos);
            covenu(pos,Qr,Qe);
            deg2dms(pos[0]*R2D,dms1,4);
            deg2dms(pos[1]*R2D,dms2,4);
        }
        vt_printf(vt," %s:%2.0f %02.0f %07.4f",pos[0]<0?"S":"N",fabs(dms1[0]),dms1[1],dms1[2]);
        vt_printf(vt," %s:%3.0f %02.0f %07.4f",pos[1]<0?"W":"E",fabs(dms2[0]),dms2[1],dms2[2]);
        vt_printf(vt," H:%8.3f",pos[2]);
        vt_printf(vt," (N:%6.3f E:%6.3f U:%6.3f)",SQRT(Qe[4]),SQRT(Qe[0]),SQRT(Qe[8]));
    }
    else if (soltype==1) {
        if (norm(sol->rr,3)>0.0) {
            ecef2pos(sol->rr,pos);
            covenu(pos,Qr,Qe);
        }
        vt_printf(vt," %s:%11.8f",pos[0]<0.0?"S":"N",fabs(pos[0])*R2D);
        vt_printf(vt," %s:%12.8f",pos[1]<0.0?"W":"E",fabs(pos[1])*R2D);
        vt_printf(vt," H:%8.3f",pos[2]);
        vt_printf(vt," (E:%6.3f N:%6.3f U:%6.3fm)",SQRT(Qe[0]),SQRT(Qe[4]),SQRT(Qe[8]));
    }
    else if (soltype==2) {
        vt_printf(vt," X:%12.3f",sol->rr[0]);
        vt_printf(vt," Y:%12.3f",sol->rr[1]);
        vt_printf(vt," Z:%12.3f",sol->rr[2]);
        vt_printf(vt," (X:%6.3f Y:%6.3f Z:%6.3f)",SQRT(Qr[0]),SQRT(Qr[4]),SQRT(Qr[8]));
    }
    else if (soltype==3) {
        if (len>0.0) {
            ecef2pos(rb,pos);
            ecef2enu(pos,bl,enu);
            covenu(pos,Qr,Qe);
        }
        vt_printf(vt," E:%12.3f",enu[0]);
        vt_printf(vt," N:%12.3f",enu[1]);
        vt_printf(vt," U:%12.3f",enu[2]);
        vt_printf(vt," (E:%6.3f N:%6.3f U:%6.3f)",SQRT(Qe[0]),SQRT(Qe[4]),SQRT(Qe[8]));
    }
    else if (soltype==4) {
        if (len>0.0) {
            ecef2pos(rb,pos);
            ecef2enu(pos,bl,enu);
            covenu(pos,Qr,Qe);
            pitch=asin(enu[2]/len);
            yaw=atan2(enu[0],enu[1]); if (yaw<0.0) yaw+=2.0*PI;
        }
        vt_printf(vt," P:%12.3f",pitch*R2D);
        vt_printf(vt," Y:%12.3f",yaw*R2D);
        vt_printf(vt," L:%12.3f",len);
        vt_printf(vt," (E:%6.3f N:%6.3f U:%6.3f)",SQRT(Qe[0]),SQRT(Qe[4]),SQRT(Qe[8]));
    }
    vt_printf(vt," A:%4.1f R:%5.1f N:%2d",sol->age,sol->ratio,sol->ns);
    vt_printf(vt,"\n");
}

static void prrtkblsol(vt_t *vt, const char *rover, const char *base, int timetype, int soltype)
{
    cors_ntrip_source_info_t *r,*b;
    cors_baseline_t *bl,*s,*t;
    double rb[3];
    sol_t sol={0};

    if (strcmp(rover,"all")==0&&strcmp(base,"all")==0) {
        HASH_ITER(hh,cors.srtk.bls.data,s,t) {
            matcpy(rb,s->rtk.rb,1,3);
            sol=s->rtk.sol;
            HASH_FIND(ii,cors.ntrip.info_tbl[1],&s->rover_srcid,sizeof(int),r);
            HASH_FIND(ii,cors.ntrip.info_tbl[1],&s->base_srcid,sizeof(int),b);
            if (!r||!b) continue;
            prsolution(vt,r->name,b->name,&sol,rb,timetype,soltype);
        }
        vt_printf(vt,"\n");
        return;
    }
    HASH_FIND_STR(cors.ntrip.info_tbl[0],rover,r);
    HASH_FIND_STR(cors.ntrip.info_tbl[0],base,b);

    if (!r||!b) {
        vt_printf(vt,"no baseline: %s->%s\n",base,rover);
        return;
    }
    char bl_id[32];

    sprintf(bl_id,"%d->%d",b->ID,r->ID);
    HASH_FIND_STR(cors.srtk.bls.data,bl_id,bl);

    if (!bl) {
        vt_printf(vt,"no baseline: %s->%s\n",base,rover);
        return;
    }
    matcpy(rb,bl->rtk.rb,1,3);
    sol=bl->rtk.sol;
    prsolution(vt,rover,base,&sol,rb,timetype,soltype);
    vt_printf(vt,"\n");
}

static void cmd_rtkpos_solbl(char **args, int narg, vt_t *vt)
{
    char rover[32],base[32];
    int i,cycle=0;
    int timetype=0,soltype=0;

    for (i=2;i<narg;i++) {
        if      (!strcmp(args[i],       "-r")&&i+1<narg) strcpy(rover,args[++i]);
        else if (!strcmp(args[i],       "-b")&&i+1<narg) strcpy(base,args[++i]);
        else if (!strcmp(args[i],   "-cycle")&&i+1<narg) cycle=atoi(args[++i])*1000;
        else if (!strcmp(args[i],"-timetype")&&i+1<narg) timetype=atoi(args[++i]);
        else if (!strcmp(args[i], "-soltype")&&i+1<narg) soltype=atoi(args[++i]);
    }
    while (!vt_chkbrk(vt)) {
        prrtkblsol(vt,rover,base,timetype,soltype);
        if (cycle>0) sleepms(cycle); else return;
    }
}

static void cmd_rtkpos(char **args, int narg, vt_t *vt)
{
    int i,cycle=0;

    if (!cors.state) {
        vt_printf(vt,"cors server has not been started\n");
        return;
    }
    if (!strcmp(args[1],"-add")) {
        cmd_rtkpos_addbl(args,narg,vt);
    }
    else if (!strcmp(args[1],"-del")) {
        cmd_rtkpos_delbl(args,narg,vt);
    }
    else if (!strcmp(args[1],"-sol")) {
        cmd_rtkpos_solbl(args,narg,vt);
    }
}

static void cmd_addvsta(char **args, int narg, vt_t *vt)
{
    if (!cors.state) {
        vt_printf(vt,"cors server has not been started\n");
        return;
    }
    if (narg<5) return;
    double pos[3];
    pos[0]=atof(args[2]);
    pos[1]=atof(args[3]);
    pos[2]=atof(args[4]);
    cors_nrtk_add_vsta(&cors.nrtk,args[1],pos);
}

static void cmd_delvsta(char **args, int narg, vt_t *vt)
{
    if (!cors.state) {
        vt_printf(vt,"cors server has not been started\n");
        return;
    }
    if (narg<2) return;
    cors_nrtk_del_vsta(&cors.nrtk,args[1]);
}

static void cmd_showdtrigs(char **args, int narg, vt_t *vt)
{
    cors_dtrig_t *d,*t;
    int id=0;

    HASH_ITER(hh,cors.nrtk.dtrig_net.dtrigs,d,t) {
        vt_printf(vt,"%3d: %12s\n",++id,d->id);
    }
}

static void cmd_showbls(char **args, int narg, vt_t *vt)
{
    const char *solstr[]={"------","FIX","FLOAT","SBAS","DGPS","SINGLE","PPP",""};
    char tbuf[16];
    cors_baseline_t *b,*t;
    cors_srtk_t *s,*st;

    HASH_ITER(hh,cors.nrtk.srtk,s,st) {
        HASH_ITER(hh,s->bls.data,b,t) {
            if (!b->sol) continue;
            time2str(b->time,tbuf,3);
            vt_printf(vt,"%s: %3d->%3d stat=%6s\n",tbuf,b->base_srcid,b->rover_srcid,solstr[b->sol->rtk.sol.stat]);
        }
    }
    HASH_ITER(hh,cors.srtk.bls.data,b,t) {
        if (!b->sol) continue;
        time2str(b->time,tbuf,3);
        vt_printf(vt,"%s: %3d->%3d stat=%6s\n",tbuf,b->base_srcid,b->rover_srcid,solstr[b->sol->rtk.sol.stat]);
    }
}

static void cmd_showsubnet(char **args, int narg, vt_t *vt)
{
    cors_dtrig_vertex_t *v,*t;
    cors_dtrig_vertex_q_t *vq,*tq;

    HASH_ITER(hh,cors.nrtk.dtrig_net.vertexs,v,t) {
        vt_printf(vt,"%3d: ",v->srcid);
        HASH_ITER(hh,v->vt_list,vq,tq) vt_printf(vt,"%3d ",vq->vt->srcid);
        vt_printf(vt,"\n");
    }
}

static void cmd_adduser(char **args, int narg, vt_t *vt)
{
    if (!cors.state) {
        vt_printf(vt,"cors server has not been started\n");
        return;
    }
    if (narg<3) return;
    cors_ntrip_agent_add_user(&cors.agent,args[1],args[2]);
}

static void cmd_deluser(char **args, int narg, vt_t *vt)
{
    if (!cors.state) {
        vt_printf(vt,"cors server has not been started\n");
        return;
    }
    if (narg<2) return;
    cors_ntrip_agent_del_user(&cors.agent,args[1]);
}

static void cmd_showusers(char **args, int narg, vt_t *vt)
{
    cors_ntrip_user_t *u,*t;
    HASH_ITER(hh,cors.agent.user_tbl,u,t) {
        vt_printf(vt,"%8s %8s\n",u->user,u->passwd);
    }
}

static void cmd_showvstas(char **args, int narg, vt_t *vt)
{
    cors_vrs_sta_t *s,*t;
    HASH_ITER(hh,cors.vrs.stas.data,s,t) {
        vt_printf(vt,"%6s %3d: pos=%8.3lf %8.3lf %8.3lf\n",s->name,s->srcid,s->pos[0],s->pos[1],s->pos[2]);
    }
}

static void con_thread(void *arg)
{
    const char *cmds[]={
            "start","stop","addsource","delsource","loadopt",
            "navidata","observ","satellite","sourceinfo","monirtcm",
            "rtkpos","addvsta","delvsta","showdtrigs","showbls","showsubnet","adduser","deluser",
            "showvstas","showusers","shutdown",""
    };
    char buff[MAXCMD],*args[MAXARG],*p;
    int i,j,narg;
    con_t *con=(con_t *)arg;

    while (con->state) {
        if (!vt_puts(con->vt,CMDPROMPT)) break;
        if (!vt_gets(con->vt,buff,sizeof(buff))) break;

        narg=0;
        for (p=strtok(buff," \t\n");p&&narg<MAXARG;p=strtok(NULL," \t\n")) {
            args[narg++]=p;
        }
        if (narg==0) continue;

        for (i=0,j=-1;*cmds[i];i++) {
            if (strstr(cmds[i],args[0])) {
                j=i;
                break;
            }
        }
        switch (j) {
            case  0: cmd_start        (args,narg,con->vt); break;
            case  1: cmd_stop         (args,narg,con->vt); break;
            case  2: cmd_add_source   (args,narg,con->vt); break;
            case  3: cmd_delete_source(args,narg,con->vt); break;
            case  4: cmd_loadopt      (args,narg,con->vt); break;
            case  5: cmd_navidata     (args,narg,con->vt); break;
            case  6: cmd_observ       (args,narg,con->vt); break;
            case  7: cmd_satellite    (args,narg,con->vt); break;
            case  8: cmd_sourceinfo   (args,narg,con->vt); break;
            case  9: cmd_monirtcm     (args,narg,con->vt); break;
            case 10: cmd_rtkpos       (args,narg,con->vt); break;
            case 11: cmd_addvsta      (args,narg,con->vt); break;
            case 12: cmd_delvsta      (args,narg,con->vt); break;
            case 13: cmd_showdtrigs   (args,narg,con->vt); break;
            case 14: cmd_showbls      (args,narg,con->vt); break;
            case 15: cmd_showsubnet   (args,narg,con->vt); break;
            case 16: cmd_adduser      (args,narg,con->vt); break;
            case 17: cmd_deluser      (args,narg,con->vt); break;
            case 18: cmd_showvstas    (args,narg,con->vt); break;
            case 19: cmd_showusers    (args,narg,con->vt); break;
            case 20:
                if (!strcmp(args[0],"shutdown")) {
                    vt_printf(con->vt,"cors server shutdown ...\n");
                    sleepms(1000);
                    con->state=0;
                }
                break;
            default:
                vt_printf(con->vt,"unknown command: %s.\n",args[0]);
                break;
        }
    }
    intflg=1;
}

static con_t *con_open(const char *dev)
{
    con_t *con;

    if (!(con=(con_t *)malloc(sizeof(con_t)))) {
        return NULL;
    }
    if (!(con->vt=vt_open(dev))) {
        free(con);
        return NULL;
    }
    con->state=1;
    if (uv_thread_create(&con->thread,con_thread,con)) {
        free(con);
        return NULL;
    }
    return con;
}

static void con_close(con_t *con)
{
    if (!con) return;
    vt_close(con->vt);
    free(con);
}

int main(int argc, char **argv)
{
    con_t *con;
    char optfile[MAXSTR]="",*dev="";
    int i,trace=3,sock=0,start=0;

    for (i=1;i<argc;i++) {
        if      (!strcmp(argv[i],"-o")&&i+1<argc) strcpy(optfile,argv[++i]);
        else if (!strcmp(argv[i],"-t")&&i+1<argc) trace=atoi(argv[++i]);
        else if (!strcmp(argv[i],"-d")&&i+1<argc) dev=argv[++i];
        else if (!strcmp(argv[i],"-s")) start=1;
    }
    cors_loadopts(&cors_opt,optfile);

    if (trace>0) {
        log_trace_open(strcmp(cors_opt.trace_file,"")==0?TRACEFILE:cors_opt.trace_file);
        log_set_level(trace);
    }
    if (!(con=con_open(dev))) {
        fprintf(stderr,"console open error dev=%s\n",dev);
        log_trace_close();
        return -1;
    }
    signal(SIGINT, sigshut);
    signal(SIGTERM,sigshut);
    signal(SIGHUP ,SIG_IGN);

    if (start) startcors(con->vt);
    while (!intflg) sleepms(100);
    stopcors(con->vt);

    con_close(con);
    log_trace_close();
    return 0;
}
