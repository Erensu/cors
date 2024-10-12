/*------------------------------------------------------------------------------
 * monitor_rtcm.c: monitor RTCM data functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

static cors_monitor_rtcm_msg_t* new_monitor_rtcm_msg(cors_monitor_rtcm_msg_t **tbl, int srcid)
{
    cors_monitor_rtcm_msg_t *s=calloc(1,sizeof(cors_monitor_rtcm_msg_t));
    s->srcid=srcid;
    HASH_ADD_INT(*tbl,srcid,s);
    return s;
}

static void upd_rtcm_msg(cors_monitor_rtcm_msgs_t *msgs, const rtcm_t* rtcm, int srcid)
{
    if (!rtcm->moni_msg) return;
    cors_monitor_rtcm_msg_t *data;

    HASH_FIND_INT(msgs->msg,&srcid,data);
    if (!data&&!(data=new_monitor_rtcm_msg(&msgs->msg,srcid))) {
        return;
    }
    strcpy(data->msg[data->n++%MAX_RTCM_MSG],rtcm->msgtype);
}

static cors_monitor_rtcm_sta_t* new_monitor_rtcm_sta(cors_monitor_rtcm_sta_t **tbl, int srcid)
{
    cors_monitor_rtcm_sta_t *s=calloc(1,sizeof(cors_monitor_rtcm_sta_t));
    s->srcid=srcid;
    HASH_ADD_INT(*tbl,srcid,s);
    return s;
}

static void upd_rtcm_sta(cors_monitor_rtcm_stas_t *stas, const rtcm_t* rtcm, int srcid)
{
    if (rtcm->moni_msg<1005||rtcm->moni_msg>1008) return;

    cors_monitor_rtcm_sta_t *data;
    HASH_FIND_INT(stas->sta,&srcid,data);
    if (!data&&!(data=new_monitor_rtcm_sta(&stas->sta,srcid))) {
        return;
    }
    data->sta=rtcm->sta;
}

extern void cors_monitor_rtcm(cors_monitor_rtcm_t *moni_rtcm, const rtcm_t* rtcm, int srcid)
{
    upd_rtcm_msg(&moni_rtcm->msgs,rtcm,srcid);
    upd_rtcm_sta(&moni_rtcm->stas,rtcm,srcid);
}

extern void cors_monitor_initrtcm(cors_monitor_rtcm_t *moni_rtcm)
{
    moni_rtcm->stas.sta=NULL;
    moni_rtcm->msgs.msg=NULL;
}

extern void cors_monitor_freertcm(cors_monitor_rtcm_t *moni_rtcm)
{
    cors_monitor_rtcm_msg_t *d,*tmp_msg;

    HASH_ITER(hh,moni_rtcm->msgs.msg,d,tmp_msg) {
        HASH_DEL(moni_rtcm->msgs.msg,d);
        free(d);
    }

    cors_monitor_rtcm_sta_t *s,*tmp_sta;
    HASH_ITER(hh,moni_rtcm->stas.sta,s,tmp_sta) {
        HASH_DEL(moni_rtcm->stas.sta,s);
        free(s);
    }
}

extern int cors_monitor_rtcm_msg(cors_monitor_rtcm_t *moni_rtcm, int srcid, char **msg_data)
{
    cors_monitor_rtcm_msg_t *d,tmp;

    HASH_FIND_INT(moni_rtcm->msgs.msg,&srcid,d);
    if (!d) return 0;
    tmp=*d;

    int i;
    for (i=0;i<MAX_RTCM_MSG;i++) {
        strcpy(msg_data[i],tmp.msg[i]);
    }
    return 1;
}

extern int cors_monitor_rtcm_sta(cors_monitor_rtcm_t *moni_rtcm, int srcid, sta_t *sta)
{
    cors_monitor_rtcm_sta_t *d;

    HASH_FIND_INT(moni_rtcm->stas.sta,&srcid,d);
    if (!d) return 0;
    *sta=d->sta; return 1;
}




