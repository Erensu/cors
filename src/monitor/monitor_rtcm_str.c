/*------------------------------------------------------------------------------
 * monitor_rtcm.c: monitor RTCM data functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

extern int monitor_rtcm_str(const cors_monitor_t *monitor, const char *name, char *buff)
{
    cors_t *cors=monitor->cors;
    cors_ntrip_source_info_t *s;

    buff[0]='\0';

    HASH_FIND_STR(cors->ntrip.info_tbl[0],name,s);
    if (!s) {
        return 0;
    }
    int i;
    char **msg=calloc(32,sizeof(*msg));
    for (i=0;i<32;i++) msg[i]=calloc(256,sizeof(char));

    if (cors_monitor_rtcm_msg(&cors->monitor.moni_rtcm,s->ID,msg)<=0) {
        return 0;
    }
    for (i=0;i<32;i++) {
        if (strcmp(msg[i],"")==0) continue;
        strcat(buff,msg[i]);
        if (i<31) {
            strcat(buff,",");
        }
    }
    for (i=0;i<32;i++) free(msg[i]); free(msg);
    return strlen(buff);
}