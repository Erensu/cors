/*------------------------------------------------------------------------------
 * monitor_nav.c: monitor navigation data functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

extern void cors_monitor_initnav(cors_monitor_nav_t *moni_nav)
{
    moni_nav->data=NULL;
}

extern void cors_monitor_freenav(cors_monitor_nav_t *moni_nav)
{
    cors_monitor_navd_t *d,*tmp;
    HASH_ITER(hh,moni_nav->data,d,tmp) {
        freenav(&d->data.data, 0xFF);
    }
}

static cors_monitor_navd_t* new_cors_navd(cors_monitor_navd_t **tbl, int srcid)
{
    cors_monitor_navd_t *s=calloc(1,sizeof(cors_monitor_navd_t));
    if (!s) return NULL;
    s->srcid=srcid;
    cors_initnav(&s->data);
    HASH_ADD_INT(*tbl,srcid,s);
    return s;
}

extern void cors_monitor_nav(cors_monitor_nav_t *moni_nav, const nav_t *nav, int ephsat, int ephset, int srcid)
{
    cors_monitor_navd_t *s;
    HASH_FIND_INT(moni_nav->data,&srcid,s);
    if (!s&&!(s=new_cors_navd(&moni_nav->data,srcid))) {
        return;
    }
    cors_updnav(&s->data,nav,ephsat,ephset);
}