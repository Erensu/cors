/*------------------------------------------------------------------------------
 * decoder.c: RTCM encoder functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

#define RTCM_TYPE_GPSL1          1001    /* basic L1-only gps rtk observables */
#define RTCM_TYPE_GPSL1EX        1002    /* extended L1-only gps rtk observables */
#define RTCM_TYPE_GPSL12         1003    /* basic L1&L2 gps rtk observables */
#define RTCM_TYPE_GPSL12EX       1004    /* extended L1&L2 gps rtk observables */
#define RTCM_TYPE_STA_ARP        1005    /* stationary rtk reference station arp */
#define RTCM_TYPE_STA_ARP_H      1006    /* stationary rtk reference station arp with height */
#define RTCM_TYPE_ANT            1007    /* antenna descriptor */
#define RTCM_TYPE_ANTS           1008    /* antenna descriptor & serial number */
#define RTCM_TYPE_GLOL1          1009    /* basic L1-only glonass rtk observables */
#define RTCM_TYPE_GLOL1EX        1010    /* extended L1-only glonass rtk observables */
#define RTCM_TYPE_GLOL12         1011    /* basic  L1&L2 glonass rtk observables */
#define RTCM_TYPE_GLOL12EX       1012    /* extended L1&L2 glonass rtk observables */
#define RTCM_TYPE_GPS_EPH        1019    /* gps ephemerides */
#define RTCM_TYPE_GLO_EPH        1020    /* glonass ephemerides */
#define RTCM_TYPE_STA_DESC       1033    /* receiver and antenna descriptor */
#define RTCM_TYPE_QZSS_EPH       1044    /* qzss ephemerides (ref [15]) */
#define RTCM_TYPE_GAL_EPHF       1045    /* galileo F/NAV satellite ephemerides */
#define RTCM_TYPE_GAL_EPHI       1046    /* galileo I/NAV satellite ephemerides */
#define RTCM_TYPE_BDS_EPH        1042    /* beidou ephemerides */
#define RTCM_TYPE_BDS_EPH1       63      /* 1042/63: beidou ephemerides (rtcm draft) */

#define RTCM_TYPE_MSM4_GPS       1074    /* msm 4: full pseudorange and phaserange plus cnr for GPS */
#define RTCM_TYPE_MSM5_GPS       1075    /* msm 5: full pseudorange, phaserange, phaserangerate and cnr for GPS */
#define RTCM_TYPE_MSM6_GPS       1076    /* msm 6: full pseudorange and phaserange plus cnr (high-res) for GPS */
#define RTCM_TYPE_MSM7_GPS       1077    /* msm 7: full pseudorange, phaserange, phaserangerate and cnr (h-res) for GPS */

#define RTCM_TYPE_MSM4_GLO       1084
#define RTCM_TYPE_MSM5_GLO       1085
#define RTCM_TYPE_MSM6_GLO       1086
#define RTCM_TYPE_MSM7_GLO       1087

#define RTCM_TYPE_MSM4_GAL       1094
#define RTCM_TYPE_MSM5_GAL       1095
#define RTCM_TYPE_MSM6_GAL       1096
#define RTCM_TYPE_MSM7_GAL       1097

#define RTCM_TYPE_MSM4_SBS       1104
#define RTCM_TYPE_MSM5_SBS       1105
#define RTCM_TYPE_MSM6_SBS       1106
#define RTCM_TYPE_MSM7_SBS       1107

#define RTCM_TYPE_MSM4_QZS       1114
#define RTCM_TYPE_MSM5_QZS       1115
#define RTCM_TYPE_MSM6_QZS       1116
#define RTCM_TYPE_MSM7_QZS       1117

#define RTCM_TYPE_MSM4_CMP       1124
#define RTCM_TYPE_MSM5_CMP       1125
#define RTCM_TYPE_MSM6_CMP       1126
#define RTCM_TYPE_MSM7_CMP       1127

/* test rtcm nav data --------------------------------------------------------*/
static int is_nav(int type)
{
    return type==RTCM_TYPE_GPS_EPH ||
           type==RTCM_TYPE_QZSS_EPH||
           type==RTCM_TYPE_GAL_EPHF||
           type==RTCM_TYPE_GAL_EPHI||
           type==RTCM_TYPE_BDS_EPH ||
           type==RTCM_TYPE_BDS_EPH1;
}
/* test rtcm gnav data -------------------------------------------------------*/
static int is_gnav(int type)
{
    return type==RTCM_TYPE_GLO_EPH;
}
/* test rtcm ant info --------------------------------------------------------*/
static int is_sta(int type)
{
    return type==RTCM_TYPE_STA_ARP  ||
           type==RTCM_TYPE_STA_ARP_H||
           type==RTCM_TYPE_ANT      ||
           type==RTCM_TYPE_ANTS     ||
           type==RTCM_TYPE_STA_DESC;
}
/* test rtcm bds obs data (msm)------------------------------------------------*/
static int is_obs_bds(int type)
{
    return type==1121||type==1122||type==1123||
           type==RTCM_TYPE_MSM4_CMP||type==RTCM_TYPE_MSM5_CMP||type==RTCM_TYPE_MSM6_CMP||
           type==RTCM_TYPE_MSM7_CMP;
}
/* test rtcm gps obs data (msm)-----------------------------------------------*/
static int is_obs_gps(int type)
{
    return type==RTCM_TYPE_MSM4_GPS||type==RTCM_TYPE_MSM5_GPS||
           type==RTCM_TYPE_MSM6_GPS||type==RTCM_TYPE_MSM7_GPS;
}
/* test rtcm glo obs data (msm)-----------------------------------------------*/
static int is_obs_glo(int type)
{
    return type==RTCM_TYPE_MSM4_GLO||type==RTCM_TYPE_MSM5_GLO||
           type==RTCM_TYPE_MSM6_GLO||type==RTCM_TYPE_MSM7_GLO;
}
/* test rtcm qzs obs data (msm)-----------------------------------------------*/
static int is_obs_qzs(int type)
{
    return type==RTCM_TYPE_MSM4_QZS||type==RTCM_TYPE_MSM5_QZS||
           type==RTCM_TYPE_MSM6_QZS||type==RTCM_TYPE_MSM7_QZS;
}
/* test rtcm sbs obs data (msm)-----------------------------------------------*/
static int is_obs_sbs(int type)
{
    return type==RTCM_TYPE_MSM4_SBS||type==RTCM_TYPE_MSM5_SBS||
           type==RTCM_TYPE_MSM6_SBS||type==RTCM_TYPE_MSM7_SBS;
}
/* test rtcm gal obs data (msm)-----------------------------------------------*/
static int is_obs_gal(int type)
{
    return type==RTCM_TYPE_MSM4_GAL||type==RTCM_TYPE_MSM5_GAL||
           type==RTCM_TYPE_MSM6_GAL||type==RTCM_TYPE_MSM7_GAL;
}
/* test rtcm obs data---------------------------------------------------------*/
static int is_obs(int type)
{
    return type==RTCM_TYPE_GPSL1   ||
           type==RTCM_TYPE_GPSL1EX ||
           type==RTCM_TYPE_GPSL12  ||
           type==RTCM_TYPE_GPSL12EX||
           type==RTCM_TYPE_GLOL1   ||
           type==RTCM_TYPE_GLOL1EX ||
           type==RTCM_TYPE_GLOL12  ||
           type==RTCM_TYPE_GLOL12EX||
           is_obs_bds(type)||is_obs_gps(type)||
           is_obs_glo(type)||is_obs_qzs(type)||
           is_obs_sbs(type)||is_obs_gal(type);
}

static int test_sys(int sys)
{
    switch (sys) {
        case SYS_GPS: return 0;
        case SYS_GLO: return 1;
        case SYS_GAL: return 2;
        case SYS_CMP: return 3;
        case SYS_QZS: return 4;
    }
    return -1;
}

extern int rtcm_encode_obs(rtcm_t *rtcm, const int *type, int nt, const nav_t *nav, obsd_t *obs, int n, char *buff)
{
    int i,nb=0,j;

    if (n<=0) return 0;
    rtcm->time=obs[0].time;
    rtcm->obs.data=obs;
    rtcm->obs.n=n;
    rtcm->seqno++;
    memcpy(rtcm->nav.glo_fcn,nav->glo_fcn,sizeof(int)*32);

    for (i=0;i<nt;i++) {
        if (!is_obs(type[i])) continue;
        j=i;
    }
    for (i=0;i<nt;i++) {
        if (!is_obs(type[i])) continue;
        if (!gen_rtcm3(rtcm,type[i],0,i!=j)) continue;
        if (rtcm->nbyte) {
            memcpy(buff+nb,rtcm->buff,sizeof(char)*rtcm->nbyte);
            nb+=rtcm->nbyte;
        }
    }
    return nb;
}

extern int rtcm_encode_eph(const int type, const eph_t *eph, char *buff)
{
    rtcm_t rtcm={0};
    eph_t eph_[MAXSAT]={0};

    if (!is_nav(type)) return 0;
    rtcm.time=eph->ttr;
    rtcm.nav.eph=eph_;
    rtcm.nav.eph[eph->sat-1]=*eph;
    rtcm.ephsat=eph->sat;
    if (!gen_rtcm3(&rtcm,type,0,0)) return 0;
    if (rtcm.nbyte) {
        memcpy(buff,rtcm.buff,sizeof(char)*rtcm.nbyte);
    }
    return rtcm.nbyte;
}

extern int rtcm_encode_geph(const int type, const geph_t *geph, char *buff)
{
    rtcm_t rtcm={0};
    geph_t geph_[MAXPRNGLO]={0};
    int prn;

    if (!is_gnav(type)) return 0;
    if (satsys(geph->sat,&prn)!=SYS_GLO) return 0;

    rtcm.time=geph->tof;
    rtcm.nav.geph=geph_;
    rtcm.nav.geph[prn-1]=*geph;
    rtcm.ephsat=geph->sat;
    if (!gen_rtcm3(&rtcm,type,0,0)) return 0;
    if (rtcm.nbyte) {
        memcpy(buff,rtcm.buff,sizeof(char)*rtcm.nbyte);
    }
    return rtcm.nbyte;
}

extern int rtcm_encode_nav(const int *type, const nav_t *nav, char *buff)
{
    int i,nb=0;

    for (i=0;i<nav->ng;i++) {
        nb+=rtcm_encode_geph(type[test_sys(satsys(nav->geph[i].sat,NULL))],&nav->geph[i],buff+nb);
    }
    for (i=0;i<nav->n;i++) {
        nb+=rtcm_encode_eph(type[test_sys(satsys(nav->eph[i].sat,NULL))],&nav->eph[i],buff+nb);
    }
    return nb;
}

extern int rtcm_encode_sta(const int type, const sta_t *sta, char *buff)
{
    rtcm_t rtcm={0};

    rtcm.sta=*sta;
    if (!is_sta(type)||!gen_rtcm3(&rtcm,type,0,0)) return 0;
    if (rtcm.nbyte) memcpy(buff,rtcm.buff,sizeof(char)*rtcm.nbyte);
    return rtcm.nbyte;;
}