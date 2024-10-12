/*------------------------------------------------------------------------------
 * monitor_src.c: monitor source data functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

extern int monitor_rtcm_str(const cors_monitor_t *monitor, const char *name, char *buff);
extern int monitor_nav_str(const cors_monitor_t *monitor, const char *name, char *buff);

static void upd_basepos_prc(double *rb, const sta_t *sta)
{
    double pos[3],del[3],dr[3];
    int i;

    matcpy(rb,sta->pos,1,3);
    ecef2pos(rb,pos);

    if (sta->deltype) {
        del[2]=sta->hgt; enu2ecef(pos,del,dr);
        for (i=0;i<3;i++) rb[i]+=sta->del[i]+dr[i];
    }
    else {
        enu2ecef(pos,sta->del,dr);
        for (i=0;i<3;i++) rb[i]+=dr[i];
    }
}

extern int monitor_src_str(const cors_monitor_t *monitor, const cors_monitor_src_t *m_pnt, const ssat_t *ssat,
                           const sol_t *sol, const obs_t *obs, char *buff)
{
    cors_t *cors=container_of(monitor,cors_t,monitor);
    const cors_monitor_bstas_info_t *bstas=&monitor->moni_bstas_info;
    cors_monitor_bsta_info_t *bsta;
    cors_sta_t *sta;
    cors_ntrip_source_info_t *info;

    buff[0]='\0';

    HASH_FIND_STR(cors->ntrip.info_tbl[0],m_pnt->name,info);
    if (!info) return 0;

    HASH_FIND_STR(bstas->data,m_pnt->name,bsta);
    HASH_FIND_INT(cors->stas.data,&info->ID,sta);

    int i;
    char tmp[32678],tmp1[32678];
    gtime_t utc=gpst2utc(obs->data[0].time);
    sprintf(tmp,"{%s,%.3lf,%d,",m_pnt->name,utc.time+utc.sec,obs->n);

    strcat(buff,tmp);

    sprintf(tmp,"{");
    strcat(buff,tmp);

    for (i=0;i<obs->n;i++) {
        char prn[6];
        satno2id(obs->data[i].sat,prn);

        if (i==(obs->n-1)) {
            sprintf(tmp,"{[sys:%d],[sat:%s],[azel:%.3lf,%.3lf],[snr:%.3lf,%.3lf,%.3lf],[health:%d],[P:%.4lf,%.4lf,%.4lf],[L:%.4lf,%.4lf,%.4lf],[LLI:%d,%d,%d]}",
                    satsys(obs->data[i].sat,NULL),prn,
                    ssat[obs->data[i].sat-1].azel[0]*R2D,
                    ssat[obs->data[i].sat-1].azel[1]*R2D,
                    (double)obs->data[i].SNR[0]*0.001,
                    (double)obs->data[i].SNR[1]*0.001,
                    (double)obs->data[i].SNR[2]*0.001,
                    0,
                    obs->data[i].P[0],
                    obs->data[i].P[1],
                    obs->data[i].P[2],
                    obs->data[i].L[0],
                    obs->data[i].L[1],
                    obs->data[i].L[2],
                    obs->data[i].LLI[0],
                    obs->data[i].LLI[1],
                    obs->data[i].LLI[2]);
            strcat(buff,tmp);
        }
        else {
            sprintf(tmp,"{[sys:%d],[sat:%s],[azel:%.3lf,%.3lf],[snr:%.3lf,%.3lf,%.3lf],[health:%d],[P:%.4lf,%.4lf,%.4lf],[L:%.4lf,%.4lf,%.4lf],[LLI:%d,%d,%d]},",
                    satsys(obs->data[i].sat,NULL),prn,
                    ssat[obs->data[i].sat-1].azel[0]*R2D,
                    ssat[obs->data[i].sat-1].azel[1]*R2D,
                    (double)obs->data[i].SNR[0]*0.001,
                    (double)obs->data[i].SNR[1]*0.001,
                    (double)obs->data[i].SNR[2]*0.001,
                    0,
                    obs->data[i].P[0],
                    obs->data[i].P[1],
                    obs->data[i].P[2],
                    obs->data[i].L[0],
                    obs->data[i].L[1],
                    obs->data[i].L[2],
                    obs->data[i].LLI[0],
                    obs->data[i].LLI[1],
                    obs->data[i].LLI[2]);
            strcat(buff,tmp);
        }
    }
    sprintf(tmp,"}");
    strcat(buff,tmp);
    sprintf(tmp,",{[nsat:%d]},",obs->n);
    strcat(buff,tmp);
    sprintf(tmp,"{[dops:%.3lf,%.3lf,%.3lf,%.3lf]},",sol->dops[0],sol->dops[1],sol->dops[2],sol->dops[3]);
    strcat(buff,tmp);

    double enu[3]={0},xyz[3]={0},dr[3]={0};
    double pos[3]={0};

    if (sta) {
        if (norm(sta->sta.pos,3)) {
            upd_basepos_prc(xyz,&sta->sta);
            ecef2pos(xyz,pos);

            dr[0]=sol->rr[0]-xyz[0];
            dr[1]=sol->rr[1]-xyz[1];
            dr[2]=sol->rr[2]-xyz[2];
            ecef2enu(pos,dr,enu);
        }
    }
    sprintf(tmp,"{[coord:%.3lf,%.3lf,%.3lf]},",enu[0],enu[1],enu[2]);
    strcat(buff,tmp);

    sprintf(tmp,"{[coord2:%.8lf,%.8lf,%.4lf],[address:%s]},",pos[0]*R2D,pos[1]*R2D,pos[2],bsta?bsta->address:m_pnt->site);
    strcat(buff,tmp);

    sprintf(tmp,"{[IP:%s],[port:%d],[user:%s],[password:%s],[mountpoint:%s]},",m_pnt->adrr,m_pnt->port,m_pnt->user,m_pnt->passwd,m_pnt->mntpnt);
    strcat(buff,tmp);

    char sta_type[64]={0};

    if (bsta) {
        if (bsta->type==0) strcpy(sta_type,"physics");
        else if (bsta->type==1) strcpy(sta_type,"virtual");
    }
    sprintf(tmp,"{[data format:RTCM3.x],[receiver type:%s],[antenna type:%s],[source type:%s]},",
            sta?(strcmp(sta->sta.rectype,"")==0?"None":sta->sta.rectype):"None",
            sta?(strcmp(sta->sta.antdes,"")==0?"None":sta->sta.antdes):"None",
            sta_type);
    strcat(buff,tmp);

    sprintf(tmp,"{[epoch:%s],[sample:%d],[obs delay:%d],[eph:%s]},",time_str(sol->time,0),1,0,"GPS+GAL+GLO+BDS");
    strcat(buff,tmp);

    monitor_rtcm_str(monitor,m_pnt->name,tmp1);
    sprintf(tmp,"{[RTCM:%s]},",tmp1);
    strcat(buff,tmp);

    monitor_nav_str(monitor,m_pnt->name,tmp1);
    sprintf(tmp,"{[NAV:%s]}",tmp1);
    strcat(buff,tmp);

    sprintf(tmp,"}\n");
    strcat(buff,tmp);
    return strlen(buff);
}