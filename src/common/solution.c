/*-----------------------------------------------------------------------------
 * solution.c : solution functions
 *----------------------------------------------------------------------------*/
#include <ctype.h>
#include "rtklib.h"

/* constants and macros ------------------------------------------------------*/
#define SQR(x)     ((x)<0.0?-(x)*(x):(x)*(x))
#define SQRT(x)    ((x)<0.0||(x)!=(x)?0.0:sqrt(x))

#define NMEA_TID   "GN"         /* NMEA talker ID for RMC and GGA sentences */
#define MAXFIELD   64           /* max number of fields in a record */
#define COMMENTH   "cors-engine"

/* solution option to field separator ----------------------------------------*/
static const char *opt2sep(const solopt_t *opt)
{
    if (!*opt->sep) return " ";
    else if (!strcmp(opt->sep,"\\t")) return "\t";
    return opt->sep;
}
/* separate fields -----------------------------------------------------------*/
static int tonum(char *buff, const char *sep, double *v)
{
    int n,len=(int)strlen(sep);
    char *p,*q;
    
    for (p=buff,n=0;n<MAXFIELD;p=q+len) {
        if ((q=strstr(p,sep))) *q='\0'; 
        if (*p) v[n++]=atof(p);
        if (!q) break;
    }
    return n;
}
/* sqrt of covariance --------------------------------------------------------*/
static double sqvar(double covar)
{
    return covar<0.0?-sqrt(-covar):sqrt(covar);
}
/* solution to covariance ----------------------------------------------------*/
static void soltocov(const sol_t *sol, double *P)
{
    P[0]     =sol->qr[0]; /* xx or ee */
    P[4]     =sol->qr[1]; /* yy or nn */
    P[8]     =sol->qr[2]; /* zz or uu */
    P[1]=P[3]=sol->qr[3]; /* xy or en */
    P[5]=P[7]=sol->qr[4]; /* yz or nu */
    P[2]=P[6]=sol->qr[5]; /* zx or ue */
}
/* covariance to solution ----------------------------------------------------*/
static void covtosol(const double *P, sol_t *sol)
{
    sol->qr[0]=(float)P[0]; /* xx or ee */
    sol->qr[1]=(float)P[4]; /* yy or nn */
    sol->qr[2]=(float)P[8]; /* zz or uu */
    sol->qr[3]=(float)P[1]; /* xy or en */
    sol->qr[4]=(float)P[5]; /* yz or nu */
    sol->qr[5]=(float)P[2]; /* zx or ue */
}
/* solution to velocity covariance -------------------------------------------*/
static void soltocov_vel(const sol_t *sol, double *P)
{
    P[0]     =sol->qv[0]; /* xx */
    P[4]     =sol->qv[1]; /* yy */
    P[8]     =sol->qv[2]; /* zz */
    P[1]=P[3]=sol->qv[3]; /* xy */
    P[5]=P[7]=sol->qv[4]; /* yz */
    P[2]=P[6]=sol->qv[5]; /* zx */
}
/* velocity covariance to solution -------------------------------------------*/
static void covtosol_vel(const double *P, sol_t *sol)
{
    sol->qv[0]=(float)P[0]; /* xx */
    sol->qv[1]=(float)P[4]; /* yy */
    sol->qv[2]=(float)P[8]; /* zz */
    sol->qv[3]=(float)P[1]; /* xy */
    sol->qv[4]=(float)P[5]; /* yz */
    sol->qv[5]=(float)P[2]; /* zx */
}
/* decode solution time ------------------------------------------------------*/
static char *decode_soltime(char *buff, const solopt_t *opt, gtime_t *time)
{
    double v[MAXFIELD];
    char *p,*q,s[64]=" ";
    int n,len;

    if (!strcmp(opt->sep,"\\t")) strcpy(s,"\t");
    else if (*opt->sep) strcpy(s,opt->sep);
    len=(int)strlen(s);
    
    if (opt->posf==SOLF_STAT) {
        return buff;
    }
    if (opt->posf==SOLF_GSIF) {
        if (sscanf(buff,"%lf %lf %lf %lf:%lf:%lf",v,v+1,v+2,v+3,v+4,v+5)<6) {
            return NULL;
        }
        *time=timeadd(epoch2time(v),-12.0*3600.0);
        if (!(p=strchr(buff,':'))||!(p=strchr(p+1,':'))) return NULL;
        for (p++;isdigit((int)*p)||*p=='.';) p++;
        return p+len;
    }
    /* yyyy/mm/dd hh:mm:ss or yyyy mm dd hh:mm:ss */
    if (sscanf(buff,"%lf/%lf/%lf %lf:%lf:%lf",v,v+1,v+2,v+3,v+4,v+5)>=6) {
        if (v[0]<100.0) {
            v[0]+=v[0]<80.0?2000.0:1900.0;
        }
        *time=epoch2time(v);
        if (opt->times==TIMES_UTC) {
            *time=utc2gpst(*time);
        }
        else if (opt->times==TIMES_JST) {
            *time=utc2gpst(timeadd(*time,-9*3600.0));
        }
        if (!(p=strchr(buff,':'))||!(p=strchr(p+1,':'))) return NULL;
        for (p++;isdigit((int)*p)||*p=='.';) p++;
        return p+len;
    }
    else { /* wwww ssss */
        for (p=buff,n=0;n<2;p=q+len) {
            if ((q=strstr(p,s))) *q='\0'; 
            if (sscanf(p,"%lf",v+n)==1) n++;
            if (!q) break;
        }
        if (n>=2&&0.0<=v[0]&&v[0]<=3000.0&&0.0<=v[1]&&v[1]<604800.0) {
            *time=gpst2time((int)v[0],v[1]);
            return p;
        }
    }
    return NULL;
}
/* decode x/y/z-ecef ---------------------------------------------------------*/
static int decode_solxyz(char *buff, const solopt_t *opt, sol_t *sol)
{
    double val[MAXFIELD],P[9]={0};
    int i=0,j,n;
    const char *sep=opt2sep(opt);

    if ((n=tonum(buff,sep,val))<3) return 0;
    
    for (j=0;j<3;j++) {
        sol->rr[j]=val[i++]; /* xyz */
    }
    if (i<n) sol->stat=(uint8_t)val[i++];
    if (i<n) sol->ns  =(uint8_t)val[i++];
    if (i+3<=n) {
        P[0]=SQR(val[i]); i++; /* sdx */
        P[4]=SQR(val[i]); i++; /* sdy */
        P[8]=SQR(val[i]); i++; /* sdz */
        if (i+3<=n) {
            P[1]=P[3]=SQR(val[i]); i++; /* sdxy */
            P[5]=P[7]=SQR(val[i]); i++; /* sdyz */
            P[2]=P[6]=SQR(val[i]); i++; /* sdzx */
        }
        covtosol(P,sol);
    }
    if (i<n) sol->age  =(float)val[i++];
    if (i<n) sol->ratio=(float)val[i++];
    
    if (i+3<=n) { /* velocity */
        for (j=0;j<3;j++) {
            sol->rr[j+3]=val[i++]; /* xyz */
        }
    }
    if (i+3<=n) {
        for (j=0;j<9;j++) P[j]=0.0;
        P[0]=SQR(val[i]); i++; /* sdx */
        P[4]=SQR(val[i]); i++; /* sdy */
        P[8]=SQR(val[i]); i++; /* sdz */
        if (i+3<n) {
            P[1]=P[3]=SQR(val[i]); i++; /* sdxy */
            P[5]=P[7]=SQR(val[i]); i++; /* sdyz */
            P[2]=P[6]=SQR(val[i]); i++; /* sdzx */
        }
        covtosol_vel(P,sol);
    }
    sol->type=0; /* postion type = xyz */
    
    if (MAXSOLQ<sol->stat) sol->stat=SOLQ_NONE;
    return 1;
}
/* decode lat/lon/height -----------------------------------------------------*/
static int decode_solllh(char *buff, const solopt_t *opt, sol_t *sol)
{
    double val[MAXFIELD],pos[3],vel[3],Q[9]={0},P[9];
    int i=0,j,n;
    const char *sep=opt2sep(opt);

    n=tonum(buff,sep,val);
    
    if (!opt->degf) {
        if (n<3) return 0;
        pos[0]=val[i++]*D2R; /* lat/lon/hgt (ddd.ddd) */
        pos[1]=val[i++]*D2R;
        pos[2]=val[i++];
    }
    else {
        if (n<7) return 0;
        pos[0]=dms2deg(val  )*D2R; /* lat/lon/hgt (ddd mm ss) */
        pos[1]=dms2deg(val+3)*D2R;
        pos[2]=val[6];
        i+=7;
    }
    pos2ecef(pos,sol->rr);
    if (i<n) sol->stat=(uint8_t)val[i++];
    if (i<n) sol->ns  =(uint8_t)val[i++];
    if (i+3<=n) {
        Q[4]=SQR(val[i]); i++; /* sdn */
        Q[0]=SQR(val[i]); i++; /* sde */
        Q[8]=SQR(val[i]); i++; /* sdu */
        if (i+3<n) {
            Q[1]=Q[3]=SQR(val[i]); i++; /* sdne */
            Q[2]=Q[6]=SQR(val[i]); i++; /* sdeu */
            Q[5]=Q[7]=SQR(val[i]); i++; /* sdun */
        }
        covecef(pos,Q,P);
        covtosol(P,sol);
    }
    if (i<n) sol->age  =(float)val[i++];
    if (i<n) sol->ratio=(float)val[i++];
    
    if (i+3<=n) { /* velocity */
        vel[1]=val[i++]; /* vel-n */
        vel[0]=val[i++]; /* vel-e */
        vel[2]=val[i++]; /* vel-u */
        enu2ecef(pos,vel,sol->rr+3);
    }
    if (i+3<=n) {
        for (j=0;j<9;j++) Q[j]=0.0;
        Q[4]=SQR(val[i]); i++; /* sdn */
        Q[0]=SQR(val[i]); i++; /* sde */
        Q[8]=SQR(val[i]); i++; /* sdu */
        if (i+3<=n) {
            Q[1]=Q[3]=SQR(val[i]); i++; /* sdne */
            Q[2]=Q[6]=SQR(val[i]); i++; /* sdeu */
            Q[5]=Q[7]=SQR(val[i]); i++; /* sdun */
        }
        covecef(pos,Q,P);
        covtosol_vel(P,sol);
    }
    sol->type=0; /* postion type = xyz */
    
    if (MAXSOLQ<sol->stat) sol->stat=SOLQ_NONE;
    return 1;
}
/* decode e/n/u-baseline -----------------------------------------------------*/
static int decode_solenu(char *buff, const solopt_t *opt, sol_t *sol)
{
    double val[MAXFIELD],Q[9]={0};
    int i=0,j,n;
    const char *sep=opt2sep(opt);

    if ((n=tonum(buff,sep,val))<3) return 0;
    
    for (j=0;j<3;j++) {
        sol->rr[j]=val[i++]; /* enu */
    }
    if (i<n) sol->stat=(uint8_t)val[i++];
    if (i<n) sol->ns  =(uint8_t)val[i++];
    if (i+3<=n) {
        Q[0]=SQR(val[i]); i++; /* sde */
        Q[4]=SQR(val[i]); i++; /* sdn */
        Q[8]=SQR(val[i]); i++; /* sdu */
        if (i+3<=n) {
            Q[1]=Q[3]=SQR(val[i]); i++; /* sden */
            Q[5]=Q[7]=SQR(val[i]); i++; /* sdnu */
            Q[2]=Q[6]=SQR(val[i]); i++; /* sdue */
        }
        covtosol(Q,sol);
    }
    if (i<n) sol->age  =(float)val[i++];
    if (i<n) sol->ratio=(float)val[i++];
    
    sol->type=1; /* postion type = enu */
    
    if (MAXSOLQ<sol->stat) sol->stat=SOLQ_NONE;
    return 1;
}
/* decode solution status ----------------------------------------------------*/
static int decode_solsss(char *buff, sol_t *sol)
{
    double tow,pos[3],std[3]={0};
    int i,week,solq;

    if (sscanf(buff,"$POS,%d,%lf,%d,%lf,%lf,%lf,%lf,%lf,%lf",&week,&tow,&solq,
               pos,pos+1,pos+2,std,std+1,std+2)<6) {
        return 0;
    }
    if (week<=0||norm(pos,3)<=0.0||solq==SOLQ_NONE) {
        return 0;
    }
    sol->time=gpst2time(week,tow);
    for (i=0;i<6;i++) {
        sol->rr[i]=i<3?pos[i]:0.0;
        sol->qr[i]=i<3?(float)SQR(std[i]):0.0f;
        sol->dtr[i]=0.0;
    }
    sol->ns=0;
    sol->age=sol->ratio=sol->thres=0.0f;
    sol->type=0; /* position type = xyz */
    sol->stat=solq;
    return 1;
}
/* decode GSI F solution -----------------------------------------------------*/
static int decode_solgsi(char *buff, const solopt_t *opt, sol_t *sol)
{
    double val[MAXFIELD];
    int i=0,j;

    if (tonum(buff," ",val)<3) return 0;
    
    for (j=0;j<3;j++) {
        sol->rr[j]=val[i++]; /* xyz */
    }
    sol->stat=SOLQ_FIX;
    return 1;
}
/* decode solution position --------------------------------------------------*/
static int decode_solpos(char *buff, const solopt_t *opt, sol_t *sol)
{
    sol_t sol0={{0}};
    char *p=buff;

    *sol=sol0;
    
    /* decode solution time */
    if (!(p=decode_soltime(p,opt,&sol->time))) {
        return 0;
    }
    /* decode solution position */
    switch (opt->posf) {
        case SOLF_XYZ : return decode_solxyz(p,opt,sol);
        case SOLF_LLH : return decode_solllh(p,opt,sol);
        case SOLF_ENU : return decode_solenu(p,opt,sol);
        case SOLF_GSIF: return decode_solgsi(p,opt,sol);
    }
    return 0;
}
/* decode reference position -------------------------------------------------*/
static void decode_refpos(char *buff, const solopt_t *opt, double *rb)
{
    double val[MAXFIELD],pos[3];
    int i,n;
    const char *sep=opt2sep(opt);

    if ((n=tonum(buff,sep,val))<3) return;
    
    if (opt->posf==SOLF_XYZ) { /* xyz */
        for (i=0;i<3;i++) rb[i]=val[i];
    }
    else if (opt->degf==0) { /* lat/lon/hgt (ddd.ddd) */
        pos[0]=val[0]*D2R;
        pos[1]=val[1]*D2R;
        pos[2]=val[2];
        pos2ecef(pos,rb);
    }
    else if (opt->degf==1&&n>=7) { /* lat/lon/hgt (ddd mm ss) */
        pos[0]=dms2deg(val  )*D2R;
        pos[1]=dms2deg(val+3)*D2R;
        pos[2]=val[6];
        pos2ecef(pos,rb);
    }
}
/* decode solution -----------------------------------------------------------*/
static int decode_sol(char *buff, const solopt_t *opt, sol_t *sol, double *rb)
{
    char *p;

    if (!strncmp(buff,COMMENTH,1)) { /* reference position */
        if (!strstr(buff,"ref pos")&&!strstr(buff,"slave pos")) return 0;
        if (!(p=strchr(buff,':'))) return 0;
        decode_refpos(p+1,opt,rb);
        return 0;
    }
    /* decode position record */
    return decode_solpos(buff,opt,sol);
}
/* decode solution options ---------------------------------------------------*/
static void decode_solopt(char *buff, solopt_t *opt)
{
    char *p;

    if (strncmp(buff,COMMENTH,1)&&strncmp(buff,"+",1)) return;
    
    if      (strstr(buff,"GPST")) opt->times=TIMES_GPST;
    else if (strstr(buff,"UTC" )) opt->times=TIMES_UTC;
    else if (strstr(buff,"JST" )) opt->times=TIMES_JST;
    
    if ((p=strstr(buff,"x-ecef(m)"))) {
        opt->posf=SOLF_XYZ;
        opt->degf=0;
        strncpy(opt->sep,p+9,1);
        opt->sep[1]='\0';
    }
    else if ((p=strstr(buff,"latitude(d'\")"))) {
        opt->posf=SOLF_LLH;
        opt->degf=1;
        strncpy(opt->sep,p+14,1);
        opt->sep[1]='\0';
    }
    else if ((p=strstr(buff,"latitude(deg)"))) {
        opt->posf=SOLF_LLH;
        opt->degf=0;
        strncpy(opt->sep,p+13,1);
        opt->sep[1]='\0';
    }
    else if ((p=strstr(buff,"e-baseline(m)"))) {
        opt->posf=SOLF_ENU;
        opt->degf=0;
        strncpy(opt->sep,p+13,1);
        opt->sep[1]='\0';
    }
    else if ((p=strstr(buff,"+SITE/INF"))) { /* gsi f2/f3 solution */
        opt->times=TIMES_GPST;
        opt->posf=SOLF_GSIF;
        opt->degf=0;
        strcpy(opt->sep," ");
    }
}
/* read solution option ------------------------------------------------------*/
static void readsolopt(FILE *fp, solopt_t *opt)
{
    char buff[MAXSOLMSG+1];
    int i;

    for (i=0;fgets(buff,sizeof(buff),fp)&&i<100;i++) { /* only 100 lines */
        
        /* decode solution options */
        decode_solopt(buff,opt);
    }
}
/* input solution data from stream ---------------------------------------------
* input solution data from stream
* args   : uint8_t data     I stream data
*          gtime_t ts       I  start time (ts.time==0: from start)
*          gtime_t te       I  end time   (te.time==0: to end)
*          double tint      I  time interval (0: all)
*          int    qflag     I  quality flag  (0: all)
*          solbuf_t *solbuf IO solution buffer
* return : status (1:solution received,0:no solution,-1:disconnect received)
*-----------------------------------------------------------------------------*/
extern int inputsol(uint8_t data, gtime_t ts, gtime_t te, double tint,
                    int qflag, const solopt_t *opt, solbuf_t *solbuf)
{
    sol_t sol={{0}};
    int stat;

    sol.time=solbuf->time;

    if (data=='$'||(!isprint(data)&&data!='\r'&&data!='\n')) { /* sync header */
        solbuf->nb=0;
    }
    if (data!='\r'&&data!='\n') {
        solbuf->buff[solbuf->nb++]=data;
    }
    if (data!='\n'&&solbuf->nb<MAXSOLMSG) return 0; /* sync trailer */

    solbuf->buff[solbuf->nb]='\0';
    solbuf->nb=0;

    /* decode solution */
    sol.time=solbuf->time;
    if ((stat=decode_sol((char *)solbuf->buff,opt,&sol,solbuf->rb))>0) {
        if (stat) solbuf->time=sol.time; /* update current time */
        if (stat!=1) return 0;
    }
    if (stat!=1||!screent(sol.time,ts,te,tint)||(qflag&&sol.stat!=qflag)) {
        return 0;
    }
    /* add solution to solution buffer */
    return addsol(solbuf,&sol);
}
/* read solution data --------------------------------------------------------*/
static int readsoldata(FILE *fp, gtime_t ts, gtime_t te, double tint, int qflag,
                      const solopt_t *opt, solbuf_t *solbuf)
{
    int c;

    while ((c=fgetc(fp))!=EOF) {

        /* input solution */
        inputsol((uint8_t)c,ts,te,tint,qflag,opt,solbuf);
    }
    return solbuf->n>0;
}
/* compare solution data -----------------------------------------------------*/
static int cmpsol(const void *p1, const void *p2)
{
    sol_t *q1=(sol_t *)p1,*q2=(sol_t *)p2;
    double tt=timediff(q1->time,q2->time);
    return tt<-0.0?-1:(tt>0.0?1:0);
}
/* sort solution data --------------------------------------------------------*/
static int sort_solbuf(solbuf_t *solbuf)
{
    sol_t *solbuf_data;

    if (solbuf->n<=0) return 0;
    
    if (!(solbuf_data=(sol_t *)realloc(solbuf->data,sizeof(sol_t)*solbuf->n))) {
        free(solbuf->data); solbuf->data=NULL; solbuf->n=solbuf->nmax=0;
        return 0;
    }
    solbuf->data=solbuf_data;
    qsort(solbuf->data,solbuf->n,sizeof(sol_t),cmpsol);
    solbuf->nmax=solbuf->n;
    solbuf->start=0;
    solbuf->end=solbuf->n-1;
    return 1;
}
/* read solutions data from solution files -------------------------------------
* read solution data from soluiton files
* args   : char   *files[]  I  solution files
*          int    nfile     I  number of files
*         (gtime_t ts)      I  start time (ts.time==0: from start)
*         (gtime_t te)      I  end time   (te.time==0: to end)
*         (double tint)     I  time interval (0: all)
*         (int    qflag)    I  quality flag  (0: all)
*          solbuf_t *solbuf O  solution buffer
* return : status (1:ok,0:no data or error)
*-----------------------------------------------------------------------------*/
extern int readsolt(char *files[], int nfile, gtime_t ts, gtime_t te,
                    double tint, int qflag, solbuf_t *solbuf)
{
    FILE *fp;
    solopt_t opt=solopt_default;
    int i;

    initsolbuf(solbuf,0,0);
    
    for (i=0;i<nfile;i++) {
        if (!(fp=fopen(files[i],"rb"))) {
            continue;
        }
        /* read solution options in header */
        readsolopt(fp,&opt);
        rewind(fp);
        
        /* read solution data */
        readsoldata(fp,ts,te,tint,qflag,&opt,solbuf);
        fclose(fp);
    }
    return sort_solbuf(solbuf);
}
extern int readsol(char *files[], int nfile, solbuf_t *sol)
{
    gtime_t time={0};
    return readsolt(files,nfile,time,time,0.0,0,sol);
}
/* add solution data to solution buffer ----------------------------------------
* add solution data to solution buffer
* args   : solbuf_t *solbuf IO solution buffer
*          sol_t  *sol      I  solution data
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int addsol(solbuf_t *solbuf, const sol_t *sol)
{
    sol_t *solbuf_data;

    if (solbuf->cyclic) { /* ring buffer */
        if (solbuf->nmax<=1) return 0;
        solbuf->data[solbuf->end]=*sol;
        if (++solbuf->end>=solbuf->nmax) solbuf->end=0;
        if (solbuf->start==solbuf->end) {
            if (++solbuf->start>=solbuf->nmax) solbuf->start=0;
        }
        else solbuf->n++;
        
        return 1;
    }
    if (solbuf->n>=solbuf->nmax) {
        solbuf->nmax=solbuf->nmax==0?8192:solbuf->nmax*2;
        if (!(solbuf_data=(sol_t *)realloc(solbuf->data,sizeof(sol_t)*solbuf->nmax))) {
            free(solbuf->data); solbuf->data=NULL; solbuf->n=solbuf->nmax=0;
            return 0;
        }
        solbuf->data=solbuf_data;
    }
    solbuf->data[solbuf->n++]=*sol;
    return 1;
}
/* get solution data from solution buffer --------------------------------------
* get solution data by index from solution buffer
* args   : solbuf_t *solbuf I  solution buffer
*          int    index     I  index of solution (0...)
* return : solution data pointer (NULL: no solution, out of range)
*-----------------------------------------------------------------------------*/
extern sol_t *getsol(solbuf_t *solbuf, int index)
{
    if (index<0||solbuf->n<=index) return NULL;
    if ((index=solbuf->start+index)>=solbuf->nmax) {
        index-=solbuf->nmax;
    }
    return solbuf->data+index;
}
/* initialize solution buffer --------------------------------------------------
* initialize position solutions
* args   : solbuf_t *solbuf I  solution buffer
*          int    cyclic    I  solution data buffer type (0:linear,1:cyclic)
*          int    nmax      I  initial number of solution data
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern void initsolbuf(solbuf_t *solbuf, int cyclic, int nmax)
{
#if 0
    gtime_t time0={0};
#endif
    int i;

    solbuf->n=solbuf->nmax=solbuf->start=solbuf->end=solbuf->nb=0;
    solbuf->cyclic=cyclic;
#if 0
    solbuf->time=time0;
#endif
    solbuf->data=NULL;
    for (i=0;i<3;i++) {
        solbuf->rb[i]=0.0;
    }
    if (cyclic) {
        if (nmax<=2) nmax=2;
        if (!(solbuf->data=malloc(sizeof(sol_t)*nmax))) {
            return;
        }
        solbuf->nmax=nmax;
    }
}
/* free solution ---------------------------------------------------------------
* free memory for solution buffer
* args   : solbuf_t *solbuf I  solution buffer
* return : none
*-----------------------------------------------------------------------------*/
extern void freesolbuf(solbuf_t *solbuf)
{
    int i;

    free(solbuf->data);
    solbuf->n=solbuf->nmax=solbuf->start=solbuf->end=solbuf->nb=0;
    solbuf->data=NULL;
    for (i=0;i<3;i++) {
        solbuf->rb[i]=0.0;
    }
}
/* output solution as the form of x/y/z-ecef ---------------------------------*/
static int outecef(uint8_t *buff, const char *s, const sol_t *sol,
                   const solopt_t *opt)
{
    const char *sep=opt2sep(opt);
    char *p=(char *)buff;

    p+=sprintf(p,"%s%s%14.4f%s%14.4f%s%14.4f%s%3d%s%3d%s%8.4f%s%8.4f%s%8.4f%s"
               "%8.4f%s%8.4f%s%8.4f%s%6.2f%s%6.1f",
               s,sep,sol->rr[0],sep,sol->rr[1],sep,sol->rr[2],sep,sol->stat,sep,
               sol->ns,sep,SQRT(sol->qr[0]),sep,SQRT(sol->qr[1]),sep,
               SQRT(sol->qr[2]),sep,sqvar(sol->qr[3]),sep,sqvar(sol->qr[4]),sep,
               sqvar(sol->qr[5]),sep,sol->age,sep,sol->ratio);
    
    if (opt->outvel) { /* output velocity */
        p+=sprintf(p,"%s%10.5f%s%10.5f%s%10.5f%s%9.5f%s%8.5f%s%8.5f%s%8.5f%s"
                   "%8.5f%s%8.5f",
                   sep,sol->rr[3],sep,sol->rr[4],sep,sol->rr[5],sep,
                   SQRT(sol->qv[0]),sep,SQRT(sol->qv[1]),sep,SQRT(sol->qv[2]),
                   sep,sqvar(sol->qv[3]),sep,sqvar(sol->qv[4]),sep,
                   sqvar(sol->qv[5]));
    }
    p+=sprintf(p,"\n");
    return p-(char *)buff;
}
/* output solution as the form of lat/lon/height -----------------------------*/
static int outpos(uint8_t *buff, const char *s, const sol_t *sol,
                  const solopt_t *opt)
{
    double pos[3],vel[3],dms1[3],dms2[3],P[9],Q[9];
    const char *sep=opt2sep(opt);
    char *p=(char *)buff;

    ecef2pos(sol->rr,pos);
    soltocov(sol,P);
    covenu(pos,P,Q);
    if (opt->height==1) { /* geodetic height */
        pos[2]-=geoidh(pos);
    }
    if (opt->degf) {
        deg2dms(pos[0]*R2D,dms1,5);
        deg2dms(pos[1]*R2D,dms2,5);
        p+=sprintf(p,"%s%s%4.0f%s%02.0f%s%08.5f%s%4.0f%s%02.0f%s%08.5f",s,sep,
                   dms1[0],sep,dms1[1],sep,dms1[2],sep,dms2[0],sep,dms2[1],sep,
                   dms2[2]);
    }
    else {
        p+=sprintf(p,"%s%s%14.9f%s%14.9f",s,sep,pos[0]*R2D,sep,pos[1]*R2D);
    }
    p+=sprintf(p,"%s%10.4f%s%3d%s%3d%s%8.4f%s%8.4f%s%8.4f%s%8.4f%s%8.4f%s%8.4f"
               "%s%6.2f%s%6.1f",
               sep,pos[2],sep,sol->stat,sep,sol->ns,sep,SQRT(Q[4]),sep,
               SQRT(Q[0]),sep,SQRT(Q[8]),sep,sqvar(Q[1]),sep,sqvar(Q[2]),
               sep,sqvar(Q[5]),sep,sol->age,sep,sol->ratio);
    
    if (opt->outvel) { /* output velocity */
        soltocov_vel(sol,P);
        ecef2enu(pos,sol->rr+3,vel);
        covenu(pos,P,Q);
        p+=sprintf(p,"%s%10.5f%s%10.5f%s%10.5f%s%9.5f%s%8.5f%s%8.5f%s%8.5f%s"
                   "%8.5f%s%8.5f",
                   sep,vel[1],sep,vel[0],sep,vel[2],sep,SQRT(Q[4]),sep,
                   SQRT(Q[0]),sep,SQRT(Q[8]),sep,sqvar(Q[1]),sep,sqvar(Q[2]),
                   sep,sqvar(Q[5]));
    }
    p+=sprintf(p,"\n");
    return p-(char *)buff;
}
/* output solution as the form of e/n/u-baseline -----------------------------*/
static int outenu(uint8_t *buff, const char *s, const sol_t *sol,
                  const double *rb, const solopt_t *opt)
{
    double pos[3],rr[3],enu[3],P[9],Q[9];
    int i;
    const char *sep=opt2sep(opt);
    char *p=(char *)buff;

    for (i=0;i<3;i++) rr[i]=sol->rr[i]-rb[i];
    ecef2pos(rb,pos);
    soltocov(sol,P);
    covenu(pos,P,Q);
    ecef2enu(pos,rr,enu);
    p+=sprintf(p,"%s%s%14.4f%s%14.4f%s%14.4f%s%3d%s%3d%s%8.4f%s%8.4f%s%8.4f%s"
               "%8.4f%s%8.4f%s%8.4f%s%6.2f%s%6.1f\n",
               s,sep,enu[0],sep,enu[1],sep,enu[2],sep,sol->stat,sep,sol->ns,sep,
               SQRT(Q[0]),sep,SQRT(Q[4]),sep,SQRT(Q[8]),sep,sqvar(Q[1]),
               sep,sqvar(Q[5]),sep,sqvar(Q[2]),sep,sol->age,sep,sol->ratio);
    return p-(char *)buff;
}
/* output processing options ---------------------------------------------------
* output processing options to buffer
* args   : uint8_t *buff    IO  output buffer
*          prcopt_t *opt    I   processing options
* return : number of output bytes
*-----------------------------------------------------------------------------*/
extern int outprcopts(uint8_t *buff, const prcopt_t *opt)
{
    const int sys[]={
        SYS_GPS,SYS_GLO,SYS_GAL,SYS_QZS,SYS_CMP,SYS_IRN,SYS_SBS,0
    };
    const char *s1[]={
        "Single","DGPS","Kinematic","Static","Moving-Base","Fixed",
        "PPP Kinematic","PPP Static","PPP Fixed","","",""
    };
    const char *s2[]={
        "L1","L1+2","L1+2+3","L1+2+3+4","L1+2+3+4+5","L1+2+3+4+5+6","","",""
    };
    const char *s3[]={
        "Forward","Backward","Combined","","",""
    };
    const char *s4[]={
        "OFF","Broadcast","SBAS","Iono-Free LC","Estimate TEC","IONEX TEC",
        "QZSS Broadcast","","","",""
    };
    const char *s5[]={
        "OFF","Saastamoinen","SBAS","Estimate ZTD","Estimate ZTD+Grad","","",""
    };
    const char *s6[]={
        "Broadcast","Precise","Broadcast+SBAS","Broadcast+SSR APC",
        "Broadcast+SSR CoM","","",""
    };
    const char *s7[]={
        "GPS","GLONASS","Galileo","QZSS","BDS","NavIC","SBAS","","",""
    };
    const char *s8[]={
        "OFF","Continuous","Instantaneous","Fix and Hold","","",""
    };
    const char *s9[]={
        "OFF","ON","","",""
    };
    int i;
    char *p=(char *)buff;

    p+=sprintf(p,"%s pos mode  : %s\r\n",COMMENTH,s1[opt->mode]);
    
    if (PMODE_DGPS<=opt->mode&&opt->mode<=PMODE_FIXED) {
        p+=sprintf(p,"%s freqs     : %s\r\n",COMMENTH,s2[opt->nf-1]);
    }
    if (opt->mode>PMODE_SINGLE) {
        p+=sprintf(p,"%s solution  : %s\r\n",COMMENTH,s3[opt->soltype]);
    }
    p+=sprintf(p,"%s elev mask : %.1f deg\r\n",COMMENTH,opt->elmin*R2D);
    if (opt->mode>PMODE_SINGLE) {
        p+=sprintf(p,"%s dynamics  : %s\r\n",COMMENTH,opt->dynamics?"on":"off");
        p+=sprintf(p,"%s tidecorr  : %s\r\n",COMMENTH,opt->tidecorr?"on":"off");
    }
    if (opt->mode<=PMODE_FIXED) {
        p+=sprintf(p,"%s ionos opt : %s\r\n",COMMENTH,s4[opt->ionoopt]);
    }
    p+=sprintf(p,"%s tropo opt : %s\r\n",COMMENTH,s5[opt->tropopt]);
    p+=sprintf(p,"%s ephemeris : %s\r\n",COMMENTH,s6[opt->sateph]);
    p+=sprintf(p,"%s navi sys  :",COMMENTH);
    for (i=0;sys[i];i++) {
        if (opt->navsys&sys[i]) p+=sprintf(p," %s",s7[i]);
    }
    p+=sprintf(p,"\r\n");
    if (PMODE_KINEMA<=opt->mode&&opt->mode<=PMODE_FIXED) {
        p+=sprintf(p,"%s amb res   : %s\r\n",COMMENTH,s8[opt->modear]);
        if (opt->navsys&SYS_GLO) {
            p+=sprintf(p,"%s amb glo   : %s\r\n",COMMENTH,s9[opt->glomodear]);
        }
        if (opt->thresar[0]>0.0) {
            p+=sprintf(p,"%s val thres : %.1f\r\n",COMMENTH,opt->thresar[0]);
        }
    }
    for (i=0;i<2;i++) {
        if (opt->mode==PMODE_SINGLE||(i>=1&&opt->mode>PMODE_FIXED)) continue;
        p+=sprintf(p,"%s antenna%d  : %-21s (%7.4f %7.4f %7.4f)\r\n",COMMENTH,
                   i+1,opt->anttype[i],opt->antdel[i][0],opt->antdel[i][1],
                   opt->antdel[i][2]);
    }
    return p-(char *)buff;
}
/* output solution header ------------------------------------------------------
* output solution header to buffer
* args   : uint8_t *buff    IO  output buffer
*          solopt_t *opt    I   solution options
* return : number of output bytes
*-----------------------------------------------------------------------------*/
extern int outsolheads(uint8_t *buff, const solopt_t *opt)
{
    const char *s1[]={"WGS84","Tokyo"},*s2[]={"ellipsoidal","geodetic"};
    const char *s3[]={"GPST","UTC ","JST "},*sep=opt2sep(opt);
    const char *leg1="Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp";
    const char *leg2="ns=# of satellites";
    char *p=(char *)buff;
    int timeu=opt->timeu<0?0:(opt->timeu>20?20:opt->timeu);

    if (opt->posf==SOLF_NMEA||opt->posf==SOLF_STAT||opt->posf==SOLF_GSIF) {
        return 0;
    }
    if (opt->outhead) {
        p+=sprintf(p,"%s (",COMMENTH);
        if      (opt->posf==SOLF_XYZ) p+=sprintf(p,"x/y/z-ecef=WGS84");
        else if (opt->posf==SOLF_ENU) p+=sprintf(p,"e/n/u-baseline=WGS84");
        else p+=sprintf(p,"lat/lon/height=%s/%s",s1[opt->datum],s2[opt->height]);
        p+=sprintf(p,",%s,%s)\r\n",leg1,leg2);
    }
    p+=sprintf(p,"%s  %-*s%s",COMMENTH,(opt->timef?16:8)+timeu+1,s3[opt->times],
               sep);
    
    if (opt->posf==SOLF_LLH) { /* lat/lon/hgt */
        if (opt->degf) {
            p+=sprintf(p,"%16s%s%16s%s%10s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s"
                       "%8s%s%6s%s%6s",
                       "latitude(d'\")",sep,"longitude(d'\")",sep,"height(m)",
                       sep,"Q",sep,"ns",sep,"sdn(m)",sep,"sde(m)",sep,"sdu(m)",
                       sep,"sdne(m)",sep,"sdeu(m)",sep,"sdue(m)",sep,"age(s)",
                       sep,"ratio");
        }
        else {
            p+=sprintf(p,"%14s%s%14s%s%10s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s"
                       "%8s%s%6s%s%6s",
                       "latitude(deg)",sep,"longitude(deg)",sep,"height(m)",sep,
                       "Q",sep,"ns",sep,"sdn(m)",sep,"sde(m)",sep,"sdu(m)",sep,
                       "sdne(m)",sep,"sdeu(m)",sep,"sdun(m)",sep,"age(s)",sep,
                       "ratio");
        }
        if (opt->outvel) {
            p+=sprintf(p,"%s%10s%s%10s%s%10s%s%9s%s%8s%s%8s%s%8s%s%8s%s%8s",
                       sep,"vn(m/s)",sep,"ve(m/s)",sep,"vu(m/s)",sep,"sdvn",sep,
                       "sdve",sep,"sdvu",sep,"sdvne",sep,"sdveu",sep,"sdvun");
        }
    }
    else if (opt->posf==SOLF_XYZ) { /* x/y/z-ecef */
        p+=sprintf(p,"%14s%s%14s%s%14s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s%8s"
                   "%s%6s%s%6s",
                   "x-ecef(m)",sep,"y-ecef(m)",sep,"z-ecef(m)",sep,"Q",sep,"ns",
                   sep,"sdx(m)",sep,"sdy(m)",sep,"sdz(m)",sep,"sdxy(m)",sep,
                   "sdyz(m)",sep,"sdzx(m)",sep,"age(s)",sep,"ratio");
        
        if (opt->outvel) {
            p+=sprintf(p,"%s%10s%s%10s%s%10s%s%9s%s%8s%s%8s%s%8s%s%8s%s%8s",
                       sep,"vx(m/s)",sep,"vy(m/s)",sep,"vz(m/s)",sep,"sdvx",sep,
                       "sdvy",sep,"sdvz",sep,"sdvxy",sep,"sdvyz",sep,"sdvzx");
        }
    }
    else if (opt->posf==SOLF_ENU) { /* e/n/u-baseline */
        p+=sprintf(p,"%14s%s%14s%s%14s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s%8s"
                   "%s%6s%s%6s",
                   "e-baseline(m)",sep,"n-baseline(m)",sep,"u-baseline(m)",sep,
                   "Q",sep,"ns",sep,"sde(m)",sep,"sdn(m)",sep,"sdu(m)",sep,
                   "sden(m)",sep,"sdnu(m)",sep,"sdue(m)",sep,"age(s)",sep,
                   "ratio");
    }
    p+=sprintf(p,"\r\n");
    return p-(char *)buff;
}
/* std-dev of soltuion -------------------------------------------------------*/
static double sol_std(const sol_t *sol)
{
    /* approximate as max std-dev of 3-axis std-devs */
    if (sol->qr[0]>sol->qr[1]&&sol->qr[0]>sol->qr[2]) return SQRT(sol->qr[0]);
    if (sol->qr[1]>sol->qr[2]) return SQRT(sol->qr[1]);
    return SQRT(sol->qr[2]);
}
/* output solution body --------------------------------------------------------
* output solution body to buffer
* args   : uint8_t *buff    IO  output buffer
*          sol_t  *sol      I   solution
*          double *rb       I   base station position {x,y,z} (ecef) (m)
*          solopt_t *opt    I   solution options
* return : number of output bytes
*-----------------------------------------------------------------------------*/
extern int outsols(uint8_t *buff, const sol_t *sol, const double *rb,
                   const solopt_t *opt)
{
    gtime_t time,ts={0};
    double gpst;
    int week,timeu;
    const char *sep=opt2sep(opt);
    char s[64];
    uint8_t *p=buff;

    /* suppress output if std is over opt->maxsolstd */
    if (opt->maxsolstd>0.0&&sol_std(sol)>opt->maxsolstd) {
        return 0;
    }
    if (opt->posf==SOLF_NMEA) {
        if (opt->nmeaintv[0]<0.0) return 0;
        if (!screent(sol->time,ts,ts,opt->nmeaintv[0])) return 0;
    }
    if (sol->stat<=SOLQ_NONE||(opt->posf==SOLF_ENU&&norm(rb,3)<=0.0)) {
        return 0;
    }
    timeu=opt->timeu<0?0:(opt->timeu>20?20:opt->timeu);
    
    time=sol->time;
    if (opt->times>=TIMES_UTC) time=gpst2utc(time);
    if (opt->times==TIMES_JST) time=timeadd(time,9*3600.0);
    
    if (opt->timef) time2str(time,s,timeu);
    else {
        gpst=time2gpst(time,&week);
        if (86400*7-gpst<0.5/pow(10.0,timeu)) {
            week++;
            gpst=0.0;
        }
        sprintf(s,"%4d%.16s%*.*f",week,sep,6+(timeu<=0?0:timeu+1),timeu,gpst);
    }
    switch (opt->posf) {
        case SOLF_LLH:  p+=outpos (p,s,sol,opt);   break;
        case SOLF_XYZ:  p+=outecef(p,s,sol,opt);   break;
        case SOLF_ENU:  p+=outenu(p,s,sol,rb,opt); break;
    }
    return p-buff;
}
/* output processing option ----------------------------------------------------
* output processing option to file
* args   : FILE   *fp       I   output file pointer
*          prcopt_t *opt    I   processing options
* return : none
*-----------------------------------------------------------------------------*/
extern void outprcopt(FILE *fp, const prcopt_t *opt)
{
    uint8_t buff[MAXSOLMSG+1];
    int n;

    if ((n=outprcopts(buff,opt))>0) {
        fwrite(buff,n,1,fp);
    }
}
/* output solution header ------------------------------------------------------
* output solution heade to file
* args   : FILE   *fp       I   output file pointer
*          solopt_t *opt    I   solution options
* return : none
*-----------------------------------------------------------------------------*/
extern void outsolhead(FILE *fp, const solopt_t *opt)
{
    uint8_t buff[MAXSOLMSG+1];
    int n;

    if ((n=outsolheads(buff,opt))>0) {
        fwrite(buff,n,1,fp);
    }
}
/* output solution body --------------------------------------------------------
* output solution body to file
* args   : FILE   *fp       I   output file pointer
*          sol_t  *sol      I   solution
*          double *rb       I   base station position {x,y,z} (ecef) (m)
*          solopt_t *opt    I   solution options
* return : none
*-----------------------------------------------------------------------------*/
extern void outsol(FILE *fp, const sol_t *sol, const double *rb,
                   const solopt_t *opt)
{
    uint8_t buff[MAXSOLMSG+1];
    int n;

    if ((n=outsols(buff,sol,rb,opt))>0) {
        fwrite(buff,n,1,fp);
    }
}
