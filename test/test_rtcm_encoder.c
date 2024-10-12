#include "cors.h"

int main(int argc, const char *argv[])
{
    const char *obs_file="test.obs";
    const char *nav_file="test.nav";
    const char *rtcm_file=".\\rtcm.out";
    char buff[MAXSAT*520]={0};
    FILE *fp_rtcm;
    obs_t obs={0};
    nav_t nav={0};
    sta_t sta={0};
    rtcm_t rtcm={0};
    int i,j,nb;
    int obs_type[5]={1074,1084,1094,1124,1114};
    int nav_type[7]={1019,1020,1046,1042,1044};

    if (!(fp_rtcm=fopen(rtcm_file,"wb"))) {
        return 0;
    }
    readrnx(obs_file,1,"",&obs,NULL,&sta);
    readrnx(nav_file,1,"",NULL,&nav,NULL);

    sortobs(&obs);

    nb=rtcm_encode_sta(1005,&sta,buff);
    if (nb) {
        fwrite(buff,nb,1,fp_rtcm);
        fflush(fp_rtcm);
    }
    nb=rtcm_encode_nav(nav_type,&nav,buff);
    if (nb) {
        fwrite(buff,nb,1,fp_rtcm);
        fflush(fp_rtcm);
    }
    for (i=0;i<obs.n;i=j) {
        for (j=i+1;j<obs.n;j++) {
            if (timediff(obs.data[j].time,obs.data[i].time)>DTTOL) break;
        }
        nb=rtcm_encode_obs(&rtcm,obs_type,5,&nav,obs.data+i,j-i,buff);
        if (nb) {
            fwrite(buff,nb,1,fp_rtcm);
            fflush(fp_rtcm);
        }
    }
    freeobs(&obs); freenav(&nav,0xFF);
    fclose(fp_rtcm);
    return 0;
}
