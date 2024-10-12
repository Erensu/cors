#include "cors.h"

static int nextobsf(const obs_t *obs, int *i, int rcv)
{
    double tt;
    int n;

    for (;*i<obs->n;(*i)++) if (obs->data[*i].rcv==rcv) break;
    for (n=0;*i+n<obs->n;n++) {
        tt=timediff(obs->data[*i+n].time,obs->data[*i].time);
        if (obs->data[*i+n].rcv!=rcv||tt>DTTOL) break;
    }
    return n;
}

static int inputobs(obs_t *obss, obsd_t *obs, const prcopt_t *popt)
{
    static int iobsu=0;
    static int iobsr=0;
    int i,nu,nr,n=0;

    if ((nu=nextobsf(obss,&iobsu,1))<=0) return -1;
    for (i=iobsr;(nr=nextobsf(obss,&i,2))>0;iobsr=i,i+=nr) {
        if (timediff(obss->data[i].time,obss->data[iobsu].time)>DTTOL) break;
    }
    nr=nextobsf(obss,&iobsr,2);
    if (nr<=0) {
        nr=nextobsf(obss,&iobsr,2);
    }
    for (i=0;i<nu&&n<MAXOBS*2;i++) obs[n++]=obss->data[iobsu+i];
    for (i=0;i<nr&&n<MAXOBS*2;i++) obs[n++]=obss->data[iobsr+i];
    iobsu+=nu;
    return n;
}

int main(int argc, const char *argv[])
{
    const char *rover_obs_file="A001.obs";
    const char *base_obs_file="A002.obs";
    const char *nav_file="A002.nav";
    obsd_t obsd[MAXOBS*2]={0};
    obsd_t obst[MAXOBS*2]={0};
    obs_t rover_obs={0,0,obsd},base_obs={0,0,obsd+MAXOBS},obss={0};
    nav_t nav={0};
    rtk_t rtk={0};
    sta_t base_sta={0},rover_sta={0};
    int nobs,i,nr,nu;
    char tbuf_r[32]={0},tbuf_b[32]={0};
    double dr[3];
    FILE *fp_sol=fopen(".\\rtksol.out","w");

    log_trace_open(".\\rtk_trace.out");
    log_set_level(1);

    readrnx(nav_file,0,"",NULL,&nav,NULL);
    readrnx(rover_obs_file,1,"",&obss,NULL,&rover_sta);
    readrnx(base_obs_file,2,"",&obss,NULL,&base_sta);

    sortobs(&obss);

    log_trace(1,"rover ant=%s rcvtype=%s\n",rover_sta.antdes,rover_sta.rectype);
    log_trace(1,"base ant=%s rcvtype=%s\n",base_sta.antdes,base_sta.rectype);

    rtkinit(&rtk,&prcopt_default_rtk);
    matcpy(rtk.rb,base_sta.pos,1,3);
    matcpy(rtk.rr,rover_sta.pos,1,3);

    while ((nobs=inputobs(&obss,obst,&prcopt_default_rtk))>=0) {

        for (nu=nr=i=0;i<nobs;i++) {
            if (obst[i].rcv==1) rover_obs.data[nu++]=obst[i];
            if (obst[i].rcv==2) base_obs.data[nr++]=obst[i];
        }
        double sow=time2gpst(rover_obs.data[0].time,NULL);

        rover_obs.n=nu;
        base_obs.n=nr;
        rtkpos(&rtk,&rover_obs,&base_obs,&nav);

        for (i=0;i<3;i++) {
            dr[i]=rtk.rb[i]-rtk.sol.rr[i];
        }
        time2str(rover_obs.data[0].time,tbuf_r,2);
        time2str(base_obs.data[0].time,tbuf_b,2);
        log_trace(1,"baselen=%6.3lf sow=%6.3lf time=%s %s ratio=%7.2lf stat=%d dxyz_float=%6.3lf %6.3lf %6.3lf dxyz_fix=%6.3lf"
                    " %6.3lf %6.3lf nb=%2d tropr=%6.3lf tropu=%6.3lf\n",
                    norm(dr,3)/1000.0,sow,tbuf_b,tbuf_r,rtk.sol.ratio,rtk.sol.stat,
                    rtk.x[0]-rover_sta.pos[0],
                    rtk.x[1]-rover_sta.pos[1],
                    rtk.x[2]-rover_sta.pos[2],
                    rtk.sol.rr[0]-rover_sta.pos[0],
                    rtk.sol.rr[1]-rover_sta.pos[1],
                    rtk.sol.rr[2]-rover_sta.pos[2],
                    rtk.nb,rtk.x[3],rtk.x[6]);
        outsol(fp_sol,&rtk.sol,rtk.rb,&solopt_default);
    }
    fclose(fp_sol);
    rtkfree(&rtk);
    freenav(&nav,0xFF);
    freeobs(&obss);
    return 0;
}