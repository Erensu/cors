/*------------------------------------------------------------------------------
 * nmea.c  : NMEA functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define NMEA_TID   "GN"         /* NMEA talker ID for RMC and GGA sentences */
#define MAXNMEA    256          /* max length of nmea sentence */
#define MAXFIELD   64           /* max number of fields in a record */

static const int nmea_sys[]={   /* NMEA systems */
        SYS_GPS|SYS_SBS,SYS_GLO,SYS_GAL,SYS_CMP,SYS_QZS,SYS_IRN,0
};
static const char *nmea_tid[]={ /* NMEA talker IDs [2] */
        "GP","GL","GA","GB","GQ","GI",""
};
static const int nmea_sid[]={   /* NMEA system IDs [1] table 21 */
        1,2,3,4,5,6,0
};
static const int nmea_solq[]={  /* NMEA GPS quality indicator [1] */
        /* 0=Fix not available or invalidi */
        /* 1=GPS SPS Mode, fix valid */
        /* 2=Differential GPS, SPS Mode, fix valid */
        /* 3=GPS PPS Mode, fix valid */
        /* 4=Real Time Kinematic. System used in RTK mode with fixed integers */
        /* 5=Float RTK. Satellite system used in RTK mode, floating integers */
        /* 6=Estimated (dead reckoning) Mode */
        /* 7=Manual Input Mode */
        /* 8=Simulation Mode */
        SOLQ_NONE ,SOLQ_SINGLE, SOLQ_DGPS, SOLQ_PPP , SOLQ_FIX,
        SOLQ_FLOAT,SOLQ_DR    , SOLQ_NONE, SOLQ_NONE, SOLQ_NONE
};

extern int outnmea_gga(uint8_t *buff, const sol_t *sol)
{
    gtime_t time;
    double h,ep[6],pos[3],dms1[3],dms2[3],dop=1.0;
    int solq,refid=0;
    char *p=(char *)buff,*q,sum;

    if (sol->stat<=SOLQ_NONE) {
        p+=sprintf(p,"$%sGGA,,,,,,,,,,,,,,",NMEA_TID);
        for (q=(char *)buff+1,sum=0;*q;q++) sum^=*q;
        p+=sprintf(p,"*%02X%c%c",sum,0x0D,0x0A);
        return p-(char *)buff;
    }
    for (solq=0;solq<8;solq++) if (nmea_solq[solq]==sol->stat) break;
    if (solq>=8) solq=0;
    time=gpst2utc(sol->time);
    if (time.sec>=0.995) {time.time++; time.sec=0.0;}
    time2epoch(time,ep);
    ecef2pos(sol->rr,pos);
    h=geoidh(pos);
    deg2dms(fabs(pos[0])*R2D,dms1,7);
    deg2dms(fabs(pos[1])*R2D,dms2,7);
    p+=sprintf(p,"$%sGGA,%02.0f%02.0f%05.2f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,"
                 "%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,%04d",
               NMEA_TID,ep[3],ep[4],ep[5],dms1[0],dms1[1]+dms1[2]/60.0,
               pos[0]>=0?"N":"S",dms2[0],dms2[1]+dms2[2]/60.0,pos[1]>=0?"E":"W",
               solq,sol->ns,dop,pos[2]-h,h,sol->age,refid);
    for (q=(char *)buff+1,sum=0;*q;q++) sum^=*q; /* check-sum */
    p+=sprintf(p,"*%02X\r\n",sum);
    return p-(char *)buff;
}
/* solution option to field separator ----------------------------------------*/
static const char *opt2sep(const solopt_t *opt)
{
    if (!*opt->sep) return " ";
    else if (!strcmp(opt->sep,"\\t")) return "\t";
    return opt->sep;
}
/* convert ddmm.mm in nmea format to deg -------------------------------------*/
static double dmm2deg(double dmm)
{
    return floor(dmm/100.0)+fmod(dmm,100.0)/60.0;
}
/* convert time in nmea format to time ---------------------------------------*/
static void septime(double t, double *t1, double *t2, double *t3)
{
    *t1=floor(t/10000.0);
    t-=*t1*10000.0;
    *t2=floor(t/100.0);
    *t3=t-*t2*100.0;
}
/* decode nmea gxgga: fix information ----------------------------------------*/
static int decode_nmeagga(char **val, int n, sol_t *sol)
{
    gtime_t time;
    double tod=0.0,lat=0.0,lon=0.0,hdop=0.0,alt=0.0,msl=0.0,ep[6],tt;
    double pos[3]={0};
    char ns='N',ew='E',ua=' ',um=' ';
    int i,solq=0,nrcv=0;

    for (i=0;i<n;i++) {
        switch (i) {
            case  0: tod =atof(val[i]); break; /* time in utc (hhmmss) */
            case  1: lat =atof(val[i]); break; /* latitude (ddmm.mmm) */
            case  2: ns  =*val[i];      break; /* N=north,S=south */
            case  3: lon =atof(val[i]); break; /* longitude (dddmm.mmm) */
            case  4: ew  =*val[i];      break; /* E=east,W=west */
            case  5: solq=atoi(val[i]); break; /* fix quality */
            case  6: nrcv=atoi(val[i]); break; /* # of satellite tracked */
            case  7: hdop=atof(val[i]); break; /* hdop */
            case  8: alt =atof(val[i]); break; /* altitude in msl */
            case  9: ua  =*val[i];      break; /* unit (M) */
            case 10: msl =atof(val[i]); break; /* height of geoid */
            case 11: um  =*val[i];      break; /* unit (M) */
        }
    }
    if ((ns!='N'&&ns!='S')||(ew!='E'&&ew!='W')) {
        return 0;
    }
    if (sol->time.time==0.0) {
        return 0;
    }
    pos[0]=(ns=='N'?1.0:-1.0)*dmm2deg(lat)*D2R;
    pos[1]=(ew=='E'?1.0:-1.0)*dmm2deg(lon)*D2R;
    pos[2]=alt+msl;

    time2epoch(sol->time,ep);
    septime(tod,ep+3,ep+4,ep+5);
    time=utc2gpst(epoch2time(ep));
    tt=timediff(time,sol->time);
    if      (tt<-43200.0) sol->time=timeadd(time, 86400.0);
    else if (tt> 43200.0) sol->time=timeadd(time,-86400.0);
    else sol->time=time;
    pos2ecef(pos,sol->rr);
    sol->stat=0<=solq&&solq<=8?nmea_solq[solq]:SOLQ_NONE;
    sol->ns=nrcv;

    sol->type=0; /* postion type = xyz */
    return 1;
}
/* decode nmea ---------------------------------------------------------------*/
extern int decode_nmea(char *buff, sol_t *sol)
{
    char *p,*q,*val[MAXFIELD];
    int n=0;

    /* parse fields */
    for (p=buff;*p&&n<MAXFIELD;p=q+1) {
        if ((q=strchr(p,','))||(q=strchr(p,'*'))) {
            val[n++]=p; *q='\0';
        }
        else break;
    }
    if (!strcmp(val[0]+3,"GGA")) { /* $xxGGA */
        return decode_nmeagga(val+1,n-1,sol);
    }
    return 0;
}