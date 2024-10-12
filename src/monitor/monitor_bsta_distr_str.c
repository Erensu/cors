/*------------------------------------------------------------------------------
 * monitor_bsta_distr_str.c: monitor base station distribution functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

extern int monitor_get_bstas_province(const cors_monitor_bstas_info_t *bstas, const char *province, int type, cors_monitor_bstas_info_t *bstas_p);
extern void monitor_free_bstas_info(cors_monitor_bstas_info_t *bstas);

extern int monitor_bsta_distr_str(const cors_monitor_bstas_info_t *bstas, const char *province, int type,
                                  char *buff)
{
    cors_monitor_bstas_info_t bstas_p={0};
    cors_monitor_bsta_info_t *s,*t;
    char tmp[1024];

    buff[0]='\0';

    if (monitor_get_bstas_province(bstas,province,type,
            &bstas_p)<=0) {
        return 0;
    }
    sprintf(tmp,"{");
    strcat(buff,tmp);

    HASH_ITER(hh,bstas_p.data,s,t) {
        sprintf(tmp,"{%s,%s,%s,%s,%.8lf,%.8lf,%.4lf,%d,%d},",s->id,s->address,s->province,s->city,s->pos[0]*R2D,
                s->pos[1]*R2D,s->pos[2],s->itrf,s->type);
        strcat(buff,tmp);
    }
    buff[strlen(buff)-1]='}';
    strcat(buff,"\n");
    monitor_free_bstas_info(&bstas_p);
    return strlen(buff);
}