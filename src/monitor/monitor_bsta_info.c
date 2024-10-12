/*------------------------------------------------------------------------------
 * monitor_bsta_info.c: monitor base station infomation functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

static int bsta_kdtree_add(struct kdtree *bstas_tree, cors_monitor_bsta_info_t *bsta)
{
    double rr[3];
    pos2ecef(bsta->pos,rr);
    kd_insert(bstas_tree,rr,bsta);
}

static int add_bsta_info_province(cors_monitor_bsta_info_province_t **province_stas, cors_monitor_bsta_info_t *sta)
{
    cors_monitor_bsta_info_province_t *s;

    HASH_FIND_STR(*province_stas,sta->province,s);
    if (!s) {
        s=calloc(1,sizeof(cors_monitor_bsta_info_province_t));
        strcpy(s->province,sta->province);
        HASH_ADD_STR(*province_stas,province,s);
    }
    cors_monitor_bsta_info_t *b;
    HASH_FIND(pr,s->data,sta->id,strlen(sta->id),b);
    if (b) return 0;

    HASH_ADD(pr,s->data,id,strlen(sta->id),sta);

    if (sta->type==0) {
        HASH_ADD(prpp,s->physic_stas,id,strlen(sta->id),sta);
    }
    else if (sta->type==1) {
        HASH_ADD(prvv,s->virtual_stas,id,strlen(sta->id),sta);
    }
    return 1;
}

extern int monitor_read_bstas_info(const char *file, cors_monitor_bstas_info_t *bstas)
{
    FILE *fp;
    char buff[1024],*p,*q,*val[64];
    int n,cnt=0;

    if (!(fp=fopen(file,"r"))) return -1;

    while (!feof(fp)) {
        if (!fgets(buff,sizeof(buff),fp)) continue;
        for (n=0,p=buff;*p&&n<64;p=q+1) {
            if ((q=strchr(p,','))||(q=strchr(p,'\n'))) {val[n++]=p; *q='\0';}
            else break;
        }
        if (n<9) continue;

        cors_monitor_bsta_info_t *i=calloc(1,sizeof(*i));
        i->type=atoi(val[8]);
        strcpy(i->id,val[0]);
        strcpy(i->province,val[2]);
        strcpy(i->address,val[1]);
        strcpy(i->city,val[3]);
        i->pos[0]=atof(val[4])*D2R;
        i->pos[1]=atof(val[5])*D2R;
        i->pos[2]=atof(val[6]);
        i->itrf=atoi(val[7]);
        HASH_ADD_STR(bstas->data,id,i);

        if (i->type==0) {
            HASH_ADD(pp,bstas->physic_stas,id,strlen(i->id),i);
        }
        else if (i->type==1) {
            HASH_ADD(vv,bstas->virtual_stas,id,strlen(i->id),i);
        }
        add_bsta_info_province(&bstas->province_stas,i);
        bsta_kdtree_add(bstas->bstas_tree,i);
    }
    fclose(fp);
    return HASH_COUNT(bstas->data);
}

extern void monitor_init_bstas_info(cors_monitor_bstas_info_t *bstas)
{
    bstas->data=NULL;
    bstas->province_stas=NULL;
    bstas->physic_stas=NULL;
    bstas->virtual_stas=NULL;
    bstas->bstas_tree=kd_create(3);
}

extern void monitor_free_bstas_info(cors_monitor_bstas_info_t *bstas)
{
    cors_monitor_bsta_info_province_t *pr,*pt;
    cors_monitor_bsta_info_t *sta,*tmp;

    kd_free(bstas->bstas_tree);

    HASH_ITER(pp,bstas->physic_stas,sta,tmp) {
        HASH_DELETE(pp,bstas->physic_stas,sta);
    }
    HASH_ITER(vv,bstas->virtual_stas,sta,tmp) {
        HASH_DELETE(vv,bstas->virtual_stas,sta);
    }
    HASH_ITER(hh,bstas->province_stas,pr,pt) {
        HASH_ITER(prpp,pr->physic_stas,sta,tmp) HASH_DELETE(prpp,pr->physic_stas,sta);
        HASH_ITER(prvv,pr->virtual_stas,sta,tmp) HASH_DELETE(prvv,pr->virtual_stas,sta);
        HASH_ITER(pr,pr->data,sta,tmp) HASH_DELETE(pr,pr->data,sta);
    }
    HASH_ITER(hh,bstas->data,sta,tmp) {
        HASH_DEL(bstas->data,sta);
        free(sta);
    }
}

static int is_in_province(const char *coord_file, double *pos)
{
    double *coord_p,*coord_p2x,*coord_p2y,pc[3]={0};
    double rr0[3],dr[3],rr[3],de[3];
    int i,j,np=0,nmax=1024,ret;
    char buff[1024],*p,*q,*val[64];
    FILE *fp;

    if (!(fp=fopen(coord_file,"r"))) {
        return -1;
    }
    coord_p=mat(3,nmax);

    while (!feof(fp)) {
        if (!fgets(buff,sizeof(buff),fp)) continue;
        if (sscanf(buff,"[%lf,%lf],\n",&pc[0],&pc[1])<2) continue;

        if (nmax<=0||np>=nmax) {
            nmax=nmax==0?1024:2*nmax;
            double *pt=realloc(coord_p,sizeof(double)*nmax*3);
            if (!pt) {free(coord_p); return -1;}
            coord_p=pt;
        }
        coord_p[3*np+0]=pc[1]*D2R;
        coord_p[3*np+1]=pc[0]*D2R;
        coord_p[3*np+2]=0.0;
        np++;
    }
    if (np<=0) {
        fclose(fp); free(coord_p); return -1;
    }
    coord_p2x=mat(1,np);
    coord_p2y=mat(1,np);
    pos2ecef(coord_p,rr0);

    for (i=0;i<np;i++) {
        pos2ecef(&coord_p[3*i],rr);
        for (j=0;j<3;j++) dr[j]=rr[j]-rr0[j];

        ecef2enu(coord_p,dr,de);
        coord_p2x[i]=de[0];
        coord_p2y[i]=de[1];
    }
    pos2ecef(pos,rr);
    for (j=0;j<3;j++) dr[j]=rr[j]-rr0[j];

    ecef2enu(coord_p,dr,de);
    ret=pnpoly(np,coord_p2x,coord_p2y,de[0],de[1]);

    free(coord_p); free(coord_p2x); free(coord_p2y);
    fclose(fp);
    return ret;
}

extern int monitor_upd_bstas_province(const char *coord_dir, cors_monitor_bstas_info_t *bstas)
{
    cors_monitor_bsta_info_t *sta,*tmp;
    char *paths[256],*p,*q,buff[1024];
    int i,nf,cnt=0;

    for (i=0;i<256;i++) {
        paths[i]=malloc(sizeof(char)*1024);
    }
    nf=expath(coord_dir,paths,256);
    if (nf<=0) {
        for (i=0;i<256;i++) free(paths[i]);
        return 0;
    }
    HASH_ITER(hh,bstas->data,sta,tmp) {
        if (sta->type==0) continue;

        for (i=0;i<nf;i++) {
            if (is_in_province(paths[i],sta->pos)<=0) {
                continue;
            }
            cnt++;
            strcpy(buff,paths[i]);
            if (!(p=strrchr(buff,FILEPATHSEP)+1)) continue;
            if (!(q=strrchr(buff,'.'))) continue;
            *q='\0';
            strcpy(sta->province,p);
            break;
        }
    }
    for (i=0;i<256;i++) free(paths[i]);
    return 1;
}

extern int monitor_get_bstas_province(const cors_monitor_bstas_info_t *bstas, const char *province, int type,
                                      cors_monitor_bstas_info_t *bstas_p)
{
    cors_monitor_bsta_info_t *data,*bsta,*tmp;

    if (strcmp(province,"all")==0) {
        if (type==0) data=bstas->physic_stas;
        if (type==1) data=bstas->virtual_stas;
        if (type==2) data=bstas->data;

        if (type==0) {
            HASH_ITER(pp,data,bsta,tmp) {
                cors_monitor_bsta_info_t *t=calloc(1,sizeof(*t));
                *t=*bsta;
                HASH_ADD_STR(bstas_p->data,id,t);
            }
        }
        else if (type==1) {
            HASH_ITER(vv,data,bsta,tmp) {
                cors_monitor_bsta_info_t *t=calloc(1,sizeof(*t));
                *t=*bsta;
                HASH_ADD_STR(bstas_p->data,id,t);
            }
        }
        else if (type==2) {
            HASH_ITER(hh,data,bsta,tmp) {
                cors_monitor_bsta_info_t *t=calloc(1,sizeof(*t));
                *t=*bsta;
                HASH_ADD_STR(bstas_p->data,id,t);
            }
        }
    }
    else {
        cors_monitor_bsta_info_province_t *p;
        HASH_FIND_STR(bstas->province_stas,province,p);
        if (!p) return 0;
        if (type==0) data=p->physic_stas;
        if (type==1) data=p->virtual_stas;
        if (type==2) data=p->data;

        if (type==0) {
            HASH_ITER(prpp,data,bsta,tmp) {
                cors_monitor_bsta_info_t *t=calloc(1,sizeof(*t));
                *t=*bsta;
                HASH_ADD_STR(bstas_p->data,id,t);
            }
        }
        else if (type==1) {
            HASH_ITER(prvv,data,bsta,tmp) {
                cors_monitor_bsta_info_t *t=calloc(1,sizeof(*t));
                *t=*bsta;
                HASH_ADD_STR(bstas_p->data,id,t);
            }
        }
        else if (type==2) {
            HASH_ITER(pr,data,bsta,tmp) {
                cors_monitor_bsta_info_t *t=calloc(1,sizeof(*t));
                *t=*bsta;
                HASH_ADD_STR(bstas_p->data,id,t);
            }
        }
    }
    return HASH_COUNT(bstas_p->data);
}

extern void monitor_add_bsta_info(cors_monitor_bstas_info_t *bstas, const char *province, const char *id, const char *address,
                                  const char *city, const double *pos, int itrf, int type)
{
    cors_monitor_bsta_info_t *bsta;
    HASH_FIND_STR(bstas->data,id,bsta);
    if (!bsta) return;

    bsta=calloc(1,sizeof(*bsta));
    strcpy(bsta->province,province);
    strcpy(bsta->city,city);
    strcpy(bsta->address,address);
    strcpy(bsta->id,id);
    bsta->itrf=itrf;
    bsta->type=type;
    matcpy(bsta->pos,pos,1,3);
    HASH_ADD_STR(bstas->data,id,bsta);

    if (bsta->type==0) {
        HASH_ADD(pp,bstas->physic_stas,id,strlen(bsta->id),bsta);
    }
    else if (bsta->type==1) {
        HASH_ADD(vv,bstas->virtual_stas,id,strlen(bsta->id),bsta);
    }
    add_bsta_info_province(&bstas->province_stas,
            bsta);
}

extern int monitor_bstas_nearest_range(cors_monitor_bstas_info_t *bstas, const double *pos, double range,
                                       cors_monitor_bsta_info_t **bstas_o)
{
    double pos_[3],rr[3];
    struct kdres *res;
    int nb=0;

    pos_[0]=pos[0]*D2R;
    pos_[1]=pos[1]*D2R;
    pos_[2]=pos[2];

    pos2ecef(pos_,rr);
    if (!(res=kd_nearest_range(bstas->bstas_tree,rr,range))) {
        return -1;
    }
    while (!kd_res_end(res)) {
        bstas_o[nb++]=kd_res_item_data(res);
        kd_res_next(res);
    }
    kd_res_free(res);
    return nb;
}




