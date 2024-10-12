#include "cors.h"

extern int monitor_read_bstas_info(const char *file, cors_monitor_bstas_info_t *bstas);
extern void monitor_init_bstas_info(cors_monitor_bstas_info_t *bstas);
extern int monitor_bstas_nearest_range(cors_monitor_bstas_info_t *bstas, const double *pos, double range,
                                       cors_monitor_bsta_info_t **bstas_o);
extern void monitor_free_bstas_info(cors_monitor_bstas_info_t *bstas);

int main(int argc, const char *argv[])
{
    const char *file="cors\\conf\\base-stations.info";
    cors_monitor_bstas_info_t bstas={0};
    double pos[3]={36.83564207,116.05735844,60.3575};
    double rr[3],rr_[3],dr[3];
    cors_monitor_bsta_info_t **bsta_infos=calloc(1024,sizeof(cors_monitor_bsta_info_t*));
    int i,j,nb;

    monitor_init_bstas_info(&bstas);
    monitor_read_bstas_info(file,&bstas);

    nb=monitor_bstas_nearest_range(&bstas,pos,30.0*1000.0,bsta_infos);
    pos[0]*=D2R;
    pos[1]*=D2R;
    pos2ecef(pos,rr);

    for (i=0;i<nb;i++) {
        cors_monitor_bsta_info_t *s=bsta_infos[i];
        pos2ecef(s->pos,rr_);

        for (j=0;j<3;j++) dr[j]=rr_[j]-rr[j];

        fprintf(stdout,"%s,%s,%s,%s,%.8lf,%.8lf,%.4lf,%d,%d,%.3lf\n",s->id,s->address,s->province,s->city,s->pos[0]*R2D,
                s->pos[1]*R2D,s->pos[2],s->itrf,s->type,norm(dr,3));
    }
    free(bsta_infos);
    monitor_free_bstas_info(&bstas);
    return 0;
}