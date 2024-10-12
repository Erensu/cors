/*------------------------------------------------------------------------------
 * dtrignet.c: Delaunay Triangulate functions for CORS
 *
 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/17 1.0  new
 *-----------------------------------------------------------------------------*/
#include "cors.h"

extern void cors_dtrignet_init(cors_dtrig_net_t *dtrig_net)
{
    dtrig_net->vts_kdtree=kd_create(3);
    dtrig_net->dtrigs=NULL;
    dtrig_net->edges=NULL;
    dtrig_net->vertexs=NULL;
    dtrig_net->state=0;
}

static void vertex_kdtree_insert(struct kdtree *vts_tree, cors_dtrig_vertex_t *vt)
{
    kd_insert(vts_tree,vt->pos,vt);
}

extern void cors_dtrignet_free(cors_dtrig_net_t *dtrig_net)
{
    cors_dtrig_t *s,*t;
    HASH_ITER(hh,dtrig_net->dtrigs,s,t) {
        HASH_DEL(dtrig_net->dtrigs,s);
        free(s);
    }
    cors_dtrig_vertex_q_t *vv,*pp;
    cors_vrs_sta_q_t *vs,*vq;
    cors_dtrig_vertex_t *v,*p;
    cors_dtrig_edge_q_t *eq,*tt;

    HASH_ITER(hh,dtrig_net->vertexs,v,p) {
        HASH_ITER(hh,v->vt_list,vv,pp) {HASH_DEL(v->vt_list,vv); free(vv);}
        HASH_ITER(hh,v->edge_list,eq,tt) {HASH_DEL(v->edge_list,eq); free(eq);}
        HASH_ITER(hh,v->vsta_list,vs,vq) {HASH_DEL(v->vsta_list,vs); free(vs);}
        HASH_DEL(dtrig_net->vertexs,v);
        free(v);
    }
    cors_dtrig_edge_t *e,*q;
    HASH_ITER(hh,dtrig_net->edges,e,q) {
        HASH_DEL(dtrig_net->edges,e);
        free(e);
    }
    kd_free(dtrig_net->vts_kdtree);
    dtrig_net->state=0;
}

static int sort_by_id(cors_dtrig_vertex_t *a, cors_dtrig_vertex_t *b)
{
    if (a->srcid==b->srcid) return 0;
    return (a->srcid<b->srcid)?-1:1;
}

static void upd_vertex(cors_dtrig_vertex_t *tp, cors_dtrig_vertex_t *vt)
{
    cors_dtrig_vertex_q_t *vq;
    HASH_FIND_PTR(tp->vt_list,&vt,vq);
    if (vq) return;
    vq=calloc(1,sizeof(cors_dtrig_vertex_q_t));
    vq->vt=vt;
    HASH_ADD_PTR(tp->vt_list,vt,vq);
}

static cors_dtrig_edge_t* new_add_edge(cors_dtrig_edge_t **edges, cors_dtrig_vertex_t *v1, cors_dtrig_vertex_t *v2)
{
    cors_dtrig_edge_t *edge;
    char id[32];
    sprintf(id,"%d->%d",v1->srcid,v2->srcid);

    HASH_FIND_STR(*edges,id,edge);
    if (edge) return NULL;

    edge=calloc(1,sizeof(*edge));
    strcpy(edge->id,id);
    edge->vt[0]=v1;
    edge->vt[1]=v2;
    HASH_ADD_STR(*edges,id,edge);
    return edge;
}

static void upd_vt_edges(cors_dtrig_vertex_t *vt, cors_dtrig_edge_t *edge)
{
    cors_dtrig_edge_q_t *et;
    HASH_FIND_PTR(vt->edge_list,&edge,et);
    if (et) return;
    et=calloc(1,sizeof(*et));
    et->edge=edge;
    HASH_ADD_PTR(vt->edge_list,edge,et);
}

static void add_edge(cors_dtrig_edge_t **edges, cors_dtrig_vertex_t **vts, cors_dtrig_vertex_t *v1, cors_dtrig_vertex_t *v2,
                     cors_dtrig_edge_t **edge_cur, cors_dtrig_edge_t **edge_add)
{
    cors_dtrig_edge_t *edge;

    edge=new_add_edge(edges,v1,v2);
    if (edge_cur) new_add_edge(edge_cur,v1,v2);
    if (edge_add&&edge) new_add_edge(edge_add,v1,v2);
    if (edge) upd_vt_edges(v1,edge);
}

static void upd_edge(cors_dtrig_edge_t **edges, cors_dtrig_vertex_t **vts, cors_dtrig_vertex_t *v1, cors_dtrig_vertex_t *v2,
                     cors_dtrig_edge_t **edge_cur, cors_dtrig_edge_t **edge_add)
{
    add_edge(edges,vts,v1,v2,edge_cur,edge_add);
    add_edge(edges,vts,v2,v1,edge_cur,edge_add);
    upd_vertex(v1,v2);
    upd_vertex(v2,v1);
}

static void upd_dtrig(cors_dtrig_t **dtrigs, cors_dtrig_vertex_t *v1, cors_dtrig_vertex_t *v2, cors_dtrig_vertex_t *v3,
                      cors_dtrig_t **dtrigs_cur)
{
    char id1[32],id2[32],id3[32];
    int i;

    sprintf(id1,"%d->%d->%d",v1->srcid,v2->srcid,v3->srcid);
    sprintf(id2,"%d->%d->%d",v2->srcid,v3->srcid,v1->srcid);
    sprintf(id3,"%d->%d->%d",v3->srcid,v1->srcid,v2->srcid);

    cors_dtrig_t *dg1,*dg2,*dg3,*dtrig;
    HASH_FIND_STR(*dtrigs,id1,dg1);
    HASH_FIND_STR(*dtrigs,id2,dg2);
    HASH_FIND_STR(*dtrigs,id3,dg3);

    dtrig=calloc(1,sizeof(*dtrig));
    dtrig->vt[0]=v1; dtrig->vt[1]=v2; dtrig->vt[2]=v3;
    strcpy(dtrig->id,id1);
    HASH_ADD_STR(*dtrigs_cur,id,dtrig);

    if (dg1||dg2||dg3) return;
    dtrig=calloc(1,sizeof(*dtrig));
    dtrig->vt[0]=v1; dtrig->vt[1]=v2; dtrig->vt[2]=v3;
    strcpy(dtrig->id,id1);
    HASH_ADD_STR(*dtrigs,id,dtrig);
}

static void del_vertex_q(cors_dtrig_vertex_q_t **vq, cors_dtrig_vertex_t *vt)
{
    cors_dtrig_vertex_q_t *q;
    HASH_FIND_PTR(*vq,&vt,q);
    if (!q) return;
    HASH_DEL(*vq,q); free(q);
}

static void del_edge_q(cors_dtrig_edge_q_t **eq, cors_dtrig_edge_t *et)
{
    cors_dtrig_edge_q_t *q;
    HASH_FIND_PTR(*eq,&et,q);
    if (!q) return;
    HASH_DEL(*eq,q); free(q);
}

static void del_edge_prc(cors_dtrig_vertex_t *vts, cors_dtrig_edge_t **edges, cors_dtrig_edge_t *et,
                         cors_dtrig_vertex_t *v1, cors_dtrig_vertex_t *v2)
{
    if (v1) del_vertex_q(&v1->vt_list,v2);
    if (v2) del_vertex_q(&v2->vt_list,v1);

    if (v1) del_edge_q(&v1->edge_list,et);
    if (v2) del_edge_q(&v2->edge_list,et);
    HASH_DEL(*edges,et);
    free(et);
}

static void do_del_edge(cors_dtrig_vertex_t *vts, cors_dtrig_edge_t **edge_cur, cors_dtrig_edge_t **edges,
                        cors_dtrig_edge_t *et, cors_dtrig_edge_t **edge_del)
{
    cors_dtrig_edge_t *e;
    HASH_FIND_STR(*edge_cur,et->id,e);
    if (e) return;
    if (edge_del) {
        e=calloc(1,sizeof(*e));
        *e=*et;
        HASH_ADD_STR(*edge_del,id,e);
    }
    del_edge_prc(vts,edges,et,et->vt[0],et->vt[1]);
}

static void del_edges(cors_dtrig_vertex_t *vts, cors_dtrig_edge_t **edge_cur, cors_dtrig_edge_t **edges,
                      cors_dtrig_edge_t **edge_del)
{
    cors_dtrig_edge_t *e,*et,*ee;
    HASH_ITER(hh,*edges,e,et) {
        do_del_edge(vts,edge_cur,edges,e,edge_del);
    }
}

static void del_dtrigs(cors_dtrig_t **dtrigs, cors_dtrig_t **dtrigs_cur)
{
    char id1[32],id2[32],id3[32];
    cors_dtrig_t *dg1,*dg2,*dg3;
    cors_dtrig_t *d,*t;
    int i;

    HASH_ITER(hh,*dtrigs,d,t) {
        sprintf(id1,"%d->%d->%d",d->vt[0]->srcid,d->vt[1]->srcid,d->vt[2]->srcid);
        sprintf(id2,"%d->%d->%d",d->vt[1]->srcid,d->vt[2]->srcid,d->vt[0]->srcid);
        sprintf(id3,"%d->%d->%d",d->vt[2]->srcid,d->vt[0]->srcid,d->vt[1]->srcid);
        HASH_FIND_STR(*dtrigs_cur,id1,dg1);
        HASH_FIND_STR(*dtrigs_cur,id2,dg2);
        HASH_FIND_STR(*dtrigs_cur,id3,dg3);
        if (dg1||dg2||dg3) continue;
        HASH_DEL(*dtrigs,d); free(d);
    }
}

extern int cors_dtrignet_build(cors_dtrig_net_t *dtrignet, cors_dtrig_edge_t **edge_add, cors_dtrig_edge_t **edge_del)
{
    struct triangulateio in,out;
    int i,j=0,k=0,cnt=HASH_COUNT(dtrignet->vertexs),p[3],l=0;
    double de[3],r0[3]={0},pos0[3],dr[3];
    char parameters[]="zQB",id1[32],id2[32],id3[32];
    cors_dtrig_vertex_t *v,*t,**vs,*vtss[2];
    cors_dtrig_vertex_t *vts=dtrignet->vertexs;
    cors_dtrig_edge_t *edge_cur=NULL,*e,*et;
    cors_dtrig_t *dtrig_cur=NULL,*dt,*tt;

    if (cnt<3) {
        if (cnt<=1) return 0;
        HASH_ITER(hh,dtrignet->vertexs,v,t) vtss[l++]=v;
        upd_edge(&dtrignet->edges,&vts,vtss[0],vtss[1],NULL,edge_add);
        upd_edge(&dtrignet->edges,&vts,vtss[1],vtss[0],NULL,edge_add);
        return 1;
    }
    HASH_SORT(dtrignet->vertexs,sort_by_id);

    in.numberofpoints=cnt;
    in.pointlist=mat(2,cnt);
    vs=calloc(cnt,sizeof(cors_dtrig_vertex_t*));

    HASH_ITER(hh,dtrignet->vertexs,v,t) {
        r0[0]+=v->pos[0];
        r0[1]+=v->pos[1];
        r0[2]+=v->pos[2];
    }
    for (i=0;i<3;i++) r0[i]/=cnt;
    ecef2pos(r0,pos0);

    HASH_ITER(hh,dtrignet->vertexs,v,t) {
        for (i=0;i<3;i++) dr[i]=v->pos[i]-r0[i];
        ecef2enu(pos0,dr,de);
        in.pointlist[k++]=de[0];
        in.pointlist[k++]=de[1];
        vs[j++]=v;
    }
    in.pointattributelist     =NULL;
    in.pointmarkerlist        =NULL;
    in.numberofpointattributes=0;
    in.numberofsegments       =0;
    in.numberofholes          =0;
    in.numberofregions        =0;
    in.numberofpoints         =cnt;
    in.regionlist             =NULL;
    out.pointlist             =NULL;
    out.pointattributelist    =NULL;
    out.pointmarkerlist       =NULL;
    out.trianglelist          =NULL;
    out.triangleattributelist =NULL;
    out.neighborlist          =NULL;
    out.segmentlist           =NULL;
    out.segmentmarkerlist     =NULL;
    out.edgelist              =NULL;
    out.edgemarkerlist        =NULL;
    triangulate(parameters,&in,&out,NULL);

    for (i=0;i<out.numberoftriangles;i++) {
        for (j=0;j<3;j++) p[j]=out.trianglelist[i*3+j];

        upd_edge(&dtrignet->edges,&vts,vs[p[0]],vs[p[1]],&edge_cur,edge_add);
        upd_edge(&dtrignet->edges,&vts,vs[p[1]],vs[p[2]],&edge_cur,edge_add);
        upd_edge(&dtrignet->edges,&vts,vs[p[2]],vs[p[0]],&edge_cur,edge_add);
        upd_dtrig(&dtrignet->dtrigs,vs[p[0]],vs[p[1]],vs[p[2]],&dtrig_cur);
    }
    del_edges(vts,&edge_cur,&dtrignet->edges,edge_del);
    del_dtrigs(&dtrignet->dtrigs,&dtrig_cur);
    HASH_ITER(hh,edge_cur,e,et) {
        HASH_DEL(edge_cur,e); free(e);
    }
    HASH_ITER(hh,dtrig_cur,dt,tt) {
        HASH_DEL(dtrig_cur,dt); free(dt);
    }
    free(out.trianglelist);
    free(vs);
    free(out.pointlist);
    free(in.pointlist);
    return out.numberoftriangles;
}

static void upd_dtrig_edge(cors_dtrig_net_t *dtrig_net)
{
    cors_dtrig_edge_t *e1,*e2,*e3;
    cors_dtrig_t *d,*t;
    static int j[3][2]={{1,2},{0,2},{0,1}};
    char id1[32],id2[32],id3[32];
    int i;

    HASH_ITER(hh,dtrig_net->dtrigs,d,t) {
        sprintf(id1,"%d->%d",d->vt[0]->srcid,d->vt[1]->srcid);
        sprintf(id2,"%d->%d",d->vt[1]->srcid,d->vt[2]->srcid);
        sprintf(id3,"%d->%d",d->vt[2]->srcid,d->vt[0]->srcid);

        HASH_FIND_STR(dtrig_net->edges,id1,e1);
        HASH_FIND_STR(dtrig_net->edges,id2,e2);
        HASH_FIND_STR(dtrig_net->edges,id3,e3);
        d->edge[0]=e1;
        d->edge[1]=e2;
        d->edge[2]=e3;
    }
    HASH_ITER(hh,dtrig_net->dtrigs,d,t) {
        for (i=0;i<3;i++) {
            sprintf(id1,"%d->%d",d->vt[i]->srcid,d->vt[j[i][0]]->srcid);
            sprintf(id2,"%d->%d",d->vt[i]->srcid,d->vt[j[i][1]]->srcid);
            HASH_FIND_STR(dtrig_net->edges,id1,e1);
            HASH_FIND_STR(dtrig_net->edges,id2,e2);
            d->edge_f[i][0]=e1;
            d->edge_f[i][1]=e2;
        }
    }
}

extern int cors_dtrignet_add_vertex(cors_dtrig_net_t *dtrig_net, const double *pos, int srcid, cors_dtrig_edge_t **edge_add,
                                    cors_dtrig_edge_t **edge_del)
{
    if (edge_del) *edge_del=NULL;
    if (edge_add) *edge_add=NULL;

    if (norm(pos,3)<=0) return 0;

    cors_dtrig_vertex_t *s;
    HASH_FIND_INT(dtrig_net->vertexs,&srcid,s);
    if (s) return 0;
    s=calloc(1,sizeof(*s));
    s->srcid=srcid;
    matcpy(s->pos,pos,1,3);

    HASH_ADD_INT(dtrig_net->vertexs,srcid,s);
    cors_dtrignet_build(dtrig_net,edge_add,edge_del);
    vertex_kdtree_insert(dtrig_net->vts_kdtree,s);
    upd_dtrig_edge(dtrig_net);
    return 1;
}

extern void cors_dtrignet_upd_vertex(cors_dtrig_net_t *dtrig_net, const double *pos, int srcid)
{
    cors_dtrig_vertex_t *s;
    HASH_FIND_INT(dtrig_net->vertexs,&srcid,s);
    if (!s) return;
    if (pos) matcpy(s->pos,pos,1,3);
}

extern void cors_dtrignet_del_vertex(cors_dtrig_net_t *dtrig_net, int srcid, cors_dtrig_edge_t **edge_add,
                                     cors_dtrig_edge_t **edge_del)
{
    cors_dtrig_vertex_t *s,*t;

    HASH_FIND_INT(dtrig_net->vertexs,&srcid,s);
    if (!s) return;
    HASH_DEL(dtrig_net->vertexs,s);
    cors_dtrignet_build(dtrig_net,edge_add,edge_del);

    kd_clear(dtrig_net->vts_kdtree);
    HASH_ITER(hh,dtrig_net->vertexs,s,t) {
        vertex_kdtree_insert(dtrig_net->vts_kdtree,s);
    }
    upd_dtrig_edge(dtrig_net);
}

extern void cors_dtrignet_del_edge(cors_dtrig_net_t *dtrignet, int srcid1, int srcid2)
{
    cors_dtrig_vertex_t *v1,*v2;
    HASH_FIND_INT(dtrignet->vertexs,&srcid1,v1);
    HASH_FIND_INT(dtrignet->vertexs,&srcid2,v2);
    if (!v1||!v2) return;

    cors_dtrig_edge_t *e;
    char id[32];
    sprintf(id,"%d->%d",srcid1,srcid2);
    HASH_FIND_STR(dtrignet->edges,id,e);
    if (!e) return;
    del_edge_prc(dtrignet->vertexs,&dtrignet->edges,e,v1,v2);
}

extern void cors_dtrignet_upd_edge(cors_dtrig_net_t *dtrig_net, cors_baseline_t *bl, int base_srcid, int rover_srcid)
{
    cors_dtrig_edge_t *e;
    char id[32];

    sprintf(id,"%d->%d",base_srcid,rover_srcid);
    HASH_FIND_STR(dtrig_net->edges,id,e);
    if (e) e->bl=bl;
}

extern void cors_dtrignet_add_edge(cors_dtrig_net_t *dtrig_net, int srcid1, int srcid2)
{
    cors_dtrig_vertex_t *v1,*v2;
    cors_dtrig_edge_t *e;
    char id[32];

    sprintf(id,"%d->%d",srcid1,srcid2);
    HASH_FIND_STR(dtrig_net->edges,id,e);
    if (e) return;

    HASH_FIND_INT(dtrig_net->vertexs,&srcid1,v1);
    HASH_FIND_INT(dtrig_net->vertexs,&srcid2,v2);
    if (!v1||!v2) return;

    e=calloc(1,sizeof(*e));
    strcpy(e->id,id);
    e->vt[0]=v1; e->vt[1]=v2;
    HASH_ADD_STR(dtrig_net->edges,id,e);

    upd_vertex(v1,v2); upd_vertex(v2,v1);
    upd_vt_edges(v1,e);
}


