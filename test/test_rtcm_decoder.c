#include "cors.h"

int main(int argc, const char *argv[])
{
    const char *file="";
    FILE *fp;
    char buff[1024];
    int nr,rlen,i,j,ret,data;
    rtcm_t rtcm={0};

    log_trace_open(".\\rtcm.out");
    log_set_level(1);

    if (!(fp=fopen(file,"rb"))) {
        return 0;
    }
    init_rtcm(&rtcm);

    uint64_t tp=uv_hrtime();

    while (!feof(fp)) {
        for (i=j=0;i<1024;i++) {
            if ((data=fgetc(fp))==EOF) break;
            buff[j++]=data;
        }
#if 1
        for (i=0;i<j;i++) {
            if ((ret=input_rtcm3(&rtcm,buff[i]))) {
                log_trace(3,"%4d: %s\n",ret,rtcm.msgtype);

                if (ret==1) {
                    log_traceobs(1,rtcm.obs.data,rtcm.obs.n);
                }
            }
        }
#else
        for (i=0,rlen=j;i<j;i++) {
            if ((ret=input_rtcm3x(&rtcm,buff+(j-rlen),rlen,&rlen))) {
                log_trace(3,"%4d: %s\n",ret,rtcm.msgtype);
            }
            if (rlen<=0) break;
        }
#endif
    }
    uint64_t tc=uv_hrtime();

    log_trace(1,"elapsed time: %.6lf ms\n",(double)(tc-tp)/1E9*1000.0);

    free_rtcm(&rtcm);
    fclose(fp);
    return 0;
}