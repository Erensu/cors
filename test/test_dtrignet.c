#include "cors.h"

extern int monitor_read_bstas_info(const char *file, cors_monitor_bstas_info_t *bstas);
extern void monitor_init_bstas_info(cors_monitor_bstas_info_t *bstas);
extern int monitor_upd_bstas_province(const char *coord_dir, cors_monitor_bstas_info_t *bstas);
extern int monitor_get_bstas_province(const cors_monitor_bstas_info_t *bstas, const char *province, int type,
                                      cors_monitor_bstas_info_t *bstas_p);
extern void monitor_free_bstas_info(cors_monitor_bstas_info_t *bstas);

int main(int argc, const char *argv[])
{
    const char *file="cors\\conf\\base-stations.info";
    cors_monitor_bstas_info_t bstas={0};
    cors_dtrig_net_t dtrig_net;
    int srcid=0;

    log_trace_open(".\\dtrig.out");
    log_set_level(1);

    monitor_init_bstas_info(&bstas);
    monitor_read_bstas_info(file,&bstas);

    cors_dtrignet_init(&dtrig_net);

    cors_monitor_bstas_info_t bstas_p={0};
    cors_monitor_bsta_info_t *s,*t;

    monitor_get_bstas_province(&bstas,"BeiJing",0,&bstas_p);
    HASH_ITER(hh,bstas_p.data,s,t) {
        cors_dtrig_edge_t *edge_del=NULL,*edge_add=NULL;
        cors_dtrig_edge_t *e,*et;
        double rr[3];
        pos2ecef(s->pos,rr);
        cors_dtrignet_add_vertex(&dtrig_net,rr,++srcid,&edge_add,&edge_del);

        fprintf(stdout,"%s %d %d\n",s->id,srcid,HASH_COUNT(dtrig_net.edges));

        HASH_ITER(hh,edge_del,e,et) {
            fprintf(stdout,"del: %s\n",e->id);
            HASH_DEL(edge_del,e);
            free(e);
        }
        HASH_ITER(hh,edge_add,e,et) {
            fprintf(stdout,"add: %s\n",e->id);
            HASH_DEL(edge_add,e);
            free(e);
        }
        cors_dtrig_vertex_t *v,*vt;
        HASH_ITER(hh,dtrig_net.vertexs,v,vt) {
            fprintf(stdout,"vertex %d: ",v->srcid);
            cors_dtrig_vertex_q_t *q,*qt;
            HASH_ITER(hh,v->vt_list,q,qt) fprintf(stdout,"%d ",q->vt->srcid);
            fprintf(stdout," | ");

            cors_dtrig_edge_q_t *ee,*eet;
            HASH_ITER(hh,v->edge_list,ee,eet) fprintf(stdout,"%s ",ee->edge->id);
            fprintf(stdout,"\n");
        }
        fprintf(stdout,"######################\n");
    }
    monitor_free_bstas_info(&bstas);

    HASH_ITER(hh,bstas_p.data,s,t) {
        HASH_DEL(bstas_p.data,s);
        free(s);
    }
    cors_dtrignet_free(&dtrig_net);
    return 0;
}