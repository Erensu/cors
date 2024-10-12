/*------------------------------------------------------------------------------
 * monitor_nav.c: monitor navigation data functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

extern int monitor_nav_str(const cors_monitor_t *monitor, const char *name, char *buff)
{
    cors_t *cors=monitor->cors;
    cors_ntrip_source_info_t *s;
    eph_t eph[MAXSAT];
    geph_t geph[MAXPRNGLO];
    gtime_t time;
    char id[32],s1[64],s2[64],s3[64],tmp[4096];
    int i,valid,prn;

    buff[0]='\0';

    HASH_FIND_STR(cors->ntrip.info_tbl[0],name,s);
    if (!s) return 0;

    cors_monitor_navd_t *nav;
    HASH_FIND_INT(monitor->moni_nav.data,&s->ID,nav);
    if (!nav) return 0;

    time=utc2gpst(timeget());
    for (i=0;i<MAXSAT;i++) eph[i]=nav->data.data.eph[i];
    for (i=0;i<MAXPRNGLO;i++) geph[i]=nav->data.data.geph[i];

    for (i=0;i<MAXSAT;i++) {
        if (!(satsys(i+1,&prn)&(SYS_GPS|SYS_GAL|SYS_QZS|SYS_CMP))||
            eph[i].sat!=i+1) continue;
        valid=eph[i].toe.time!=0&&!eph[i].svh&&
              fabs(timediff(time,eph[i].toe))<=MAXDTOE;
        satno2id(i+1,id);
        if (eph[i].toe.time!=0) time2str(eph[i].toe,s1,0); else strcpy(s1,"-");
        if (eph[i].toc.time!=0) time2str(eph[i].toc,s2,0); else strcpy(s2,"-");
        if (eph[i].ttr.time!=0) time2str(eph[i].ttr,s3,0); else strcpy(s3,"-");

        sprintf(tmp,"{[sys:%d],[sat:%s],[valid:%s],[IODE:%d],[IODC:%d],[FRQ:%d],[A/A:%d],[SVH:%d],[Toe:%s],[Toc:%s],[Ttr/Tof:%s],[L2C:%d],[L2P:%d]},",
                satsys(i+1,NULL),id,valid?"OK":"-",
                eph[i].iode,eph[i].iodc,0,
                eph[i].sva,eph[i].svh,
                s1,s2,s3,
                eph[i].code,eph[i].flag);
        strcat(buff,tmp);
    }
    for (i=0;i<MAXSAT;i++) {
        if (!(satsys(i+1,&prn)&SYS_GLO)||geph[prn-1].sat!=i+1) continue;
        valid=geph[prn-1].toe.time!=0&&!geph[prn-1].svh&&
              fabs(timediff(time,geph[prn-1].toe))<=MAXDTOE_GLO;
        satno2id(i+1,id);
        if (geph[prn-1].toe.time!=0) time2str(geph[prn-1].toe,s1,0); else strcpy(s1,"-");
        if (geph[prn-1].tof.time!=0) time2str(geph[prn-1].tof,s2,0); else strcpy(s2,"-");

        sprintf(tmp,"{[sys:%d],[sat:%s],[valid:%s],[IODE:%d],[IODC:%d],[FRQ:%d],[A/A:%d],[SVH:%d],[Toe:%s],[Toc:%s],[Ttr/Tof:%s],[L2C:%d],[L2P:%d]},",
                satsys(i+1,NULL),id,valid?"OK":"-",
                geph[prn-1].iode,0,
                geph[prn-1].frq,
                geph[prn-1].age,
                geph[prn].svh,s1,"-",s2,0,0);
        strcat(buff,tmp);
    }
    if (strlen(buff)) buff[strlen(buff)-1]='\0';
    return strlen(buff);
}