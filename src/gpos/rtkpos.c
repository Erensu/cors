/*------------------------------------------------------------------------------
 * rtkpos.c : precise positioning

 * author  : sujinglan
 * version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
 * history : 2022/11/21 1.0  new
 *-----------------------------------------------------------------------------*/
#include "rtklib.h"
#include "log.h"

/* constants/macros ----------------------------------------------------------*/
#define SQR(x)      ((x)*(x))
#define SQRT(x)     ((x)<=0.0||(x)!=(x)?0.0:sqrt(x))
#define MIN(x,y)    ((x)<=(y)?(x):(y))
#define MAX(x,y)    ((x)>(y)?(x):(y))
#define ROUND(x)    (int)floor((x)+0.5)

#define VAR_POS     SQR(60.0) /* initial variance of receiver pos (m^2) */
#define VAR_GRA     SQR(0.001)/* initial variance of gradient (m^2) */
#define INIT_ZWD    0.15      /* initial zwd (m) */

#define GAP_RESION  120       /* gap to reset ionosphere parameters (epochs) */
#define VAR_HOLDAMB 0.001     /* constraint to hold ambiguity (cycle^2) */

#define CONST_FIX_INHERIT_AMB  0
#define INHERIT_AMB            1
#define THRES_MW_JUMP          0.3
#define ADJ_AR_RATIO           1

/* number of parameters (pos,ionos,tropos,hw-bias,phase-bias,real,estimated) */
#define NF(opt)     ((opt)->nf)
#define NP(opt)     (3)
#define NI(opt)     ((opt)->ionoopt!=IONOOPT_EST?0:MAXSAT)
#define NT(opt)     ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt<TROPOPT_ESTG?2:6))
#define NB(opt)     ((opt)->mode<=PMODE_DGPS?0:MAXSAT*NF(opt))
#define NR(opt)     (NP(opt)+NI(opt)+NT(opt))
#define NX(opt)     (NR(opt)+NB(opt))

/* state variable index */
#define II(s,opt)   (NP(opt)+(s)-1)                 /* ionos (s:satellite no) */
#define IT(r,opt)   (NP(opt)+NI(opt)+NT(opt)/2*(r)) /* tropos (r:0=rov,1:ref) */
#define IB(s,f,opt) (NR(opt)+MAXSAT*(f)+(s)-1)      /* phase bias (s:satno,f:freq) */

#define DD_BSAT(val) ((val>>16)&0xFF)
#define DD_RSAT(val) ((val>> 8)&0xFF)
#define DD_TYPE(val) ((val>> 4)&0x0F)
#define DD_FREQ(val) ((val    )&0x0F)

/* adjust AR ratio based on # of ambiguity------------------------------------*/
static double adj_arratio(const prcopt_t *opt, int namb)
{
    /* poly coeffs used to adjust AR ratio by # of ambiguity, derived by fitting to example from:
     * https://www.tudelft.nl/citg/over-faculteit/afdelingen/geoscience-remote-sensing/research/lambda/lambda
     * */
    static double ar_poly_coeffs[3][5] = {
            {-1.94058448e-01,-7.79023476e+00, 1.24231120e+02,-4.03126050e+02, 3.50413202e+02},
            { 6.42237302e-01,-8.39813962e+00, 2.92107285e+01,-2.37577308e+01,-1.14307128e+00},
            {-2.22600390e-02, 3.23169103e-01,-1.39837429e+00, 2.19282996e+00,-5.34583971e-02}};
    int nb=namb<50?namb:50,i,j;
    double coeff[5]={0},thres,min_ratio=1.2,max_ratio=3.0;

    /* generate poly coeffs based on nominal AR ratio */
    for ((i=0);i<3;i++) {
        coeff[i]=ar_poly_coeffs[i][0];
        for ((j=1);j<5;j++)coeff[i]=coeff[i]*opt->thresar[0]+ar_poly_coeffs[i][j];
    }
    /* generate adjusted AR ratio based on # of sat pairs */
    thres=coeff[0];
    for (i=1;i<3;i++) thres=thres*1/(nb+1)+coeff[i];
    thres=MIN(MAX(min_ratio,thres),max_ratio);
    return thres;
}
/* single-differenced observable ---------------------------------------------*/
static double sdobs(const obsd_t *obs, int i, int j, int k)
{
    double pi=(k<NFREQ)?obs[i].L[k]:obs[i].P[k-NFREQ];
    double pj=(k<NFREQ)?obs[j].L[k]:obs[j].P[k-NFREQ];
    return pi==0.0||pj==0.0?0.0:pi-pj;
}
/* single-differenced geometry-free linear combination of phase --------------*/
static double gfobs(const obsd_t *obs, int i, int j, int k, const nav_t *nav)
{
    double freq1,freq2,L1,L2;

    freq1=sat2freq(obs[i].sat,obs[i].code[0],nav);
    freq2=sat2freq(obs[i].sat,obs[i].code[k],nav);
    L1=sdobs(obs,i,j,0);
    L2=sdobs(obs,i,j,k);
    if (freq1==0.0||freq2==0.0||L1==0.0||L2==0.0) return 0.0;
    return L1*CLIGHT/freq1-L2*CLIGHT/freq2;
}
/* Melbourne-Wubbena linear combination --------------------------------------*/
static double mwobs(const obsd_t *obs, int iu, int ir, int j, int k, const nav_t *nav, double *lam)
{
    double L1,L2,P1,P2,lam1,lam2;

    L1=sdobs(obs,iu,ir,j);
    L2=sdobs(obs,iu,ir,k);
    P1=sdobs(obs,iu,ir,j+NFREQ);
    P2=sdobs(obs,iu,ir,k+NFREQ);

    if (!L1||!L2||!P1||!P2) return 0.0;

    lam1=CLIGHT/sat2freq(obs[iu].sat,obs[iu].code[j],nav);
    lam2=CLIGHT/sat2freq(obs[iu].sat,obs[iu].code[k],nav);
    if (lam) *lam=lam1*lam2/(lam2-lam1);
    return lam1*lam2*(L1-L2)/(lam2-lam1)-(lam2*P1+lam1*P2)/(lam2+lam1);
}
/* single-differenced measurement error variance -----------------------------*/
static double varerr(int sat, int sys, double el, double bl, double dt, int f,
                     const prcopt_t *opt)
{
    double a,b,c=opt->err[3]*bl/1E4,d=CLIGHT*opt->sclkstab*dt,fact=1.0;
    double sinel=sin(el);
    int nf=NF(opt);

    if (f>=nf) fact=opt->eratio[f-nf];
    if (fact<=0.0) fact=opt->eratio[0];
    fact*=sys==SYS_GLO?EFACT_GLO:(sys==SYS_SBS?EFACT_SBS:(sys==SYS_CMP?EFACT_CMP:EFACT_GPS));
    a=fact*opt->err[1];
    b=fact*opt->err[2];
    return 2.0*(opt->ionoopt==IONOOPT_IFLC?3.0:1.0)*(a*a+b*b/sinel/sinel+c*c)+d*d;
}
/* baseline length -----------------------------------------------------------*/
static double baseline(const double *ru, const double *rb, double *dr)
{
    int i;
    for (i=0;i<3;i++) dr[i]=ru[i]-rb[i];
    return norm(dr,3);
}
/* initialize state and covariance -------------------------------------------*/
static void initx(rtk_t *rtk, double xi, double var, int i)
{
    int j;
    rtk->x[i]=xi;
    for (j=0;j<rtk->nx;j++) {
        rtk->P[i+j*rtk->nx]=rtk->P[j+i*rtk->nx]=i==j?var:0.0;
    }
}
/* select common satellites between rover and reference station --------------*/
static int selsat(const rtk_t *rtk, const obsd_t *obs, const double *azel, int nu, int nr,
                  const prcopt_t *opt, int *sat, int *iu, int *ir)
{
    int i,j,k=0;
    for (i=0,j=nu;i<nu&&j<nu+nr;i++,j++) {
        if (satexclude(obs[i].sat,rtk->ssat[obs[i].sat-1].var,rtk->ssat[obs[i].sat-1].svh,&rtk->opt)) {
            continue;
        }
        if      (obs[i].sat<obs[j].sat) j--;
        else if (obs[i].sat>obs[j].sat) i--;
        else if (azel[1+j*2]>=opt->elmin) { /* elevation at base station */
            sat[k]=obs[i].sat; iu[k]=i; ir[k++]=j;
            log_trace(4,"(%2d) sat=%3d iu=%2d ir=%2d\n",k-1,obs[i].sat,i,j);
        }
    }
    return k;
}
/* temporal update of position/velocity/acceleration -------------------------*/
static void udpos(int reset, rtk_t *rtk, double tt)
{
    int i,j,nx;

    /* fix mode */
    if (rtk->opt.mode==PMODE_FIXED) {
        for (i=0;i<3;i++) {
            rtk->x[i]=rtk->rr[i]; rtk->P[i+i*rtk->nx]=0.0;
        }
        return;
    }
    if (norm(rtk->x,3)<=0.0||reset) {
        for (i=0;i<3;i++) initx(rtk,rtk->sol.rr[i],VAR_POS,i);
        return;
    }
    if (rtk->opt.mode==PMODE_STATIC) return;

    for (i=0;i<3;i++) {
        if (rtk->x[i]) rtk->P[i+i*rtk->nx]+=VAR_POS;
        else {
            rtk->x[i]=rtk->sol.rr[i];
            rtk->P[i+i*rtk->nx]+=VAR_POS;
        }
    }
}
/* temporal update of ionospheric parameters ---------------------------------*/
static void udion(int reset, rtk_t *rtk, double tt, double bl, const int *sat, int ns)
{
    double el,fact;
    int i,j;

    for (i=1;i<=MAXSAT;i++) {
        j=II(i,&rtk->opt);
        if (rtk->x[j]!=0.0&&rtk->ssat[i-1].outc[0]>GAP_RESION&&rtk->ssat[i-1].outc[1]>GAP_RESION)
            rtk->x[j]=0.0;
    }
    for (i=0;i<ns;i++) {
        j=II(sat[i],&rtk->opt);
        if (rtk->x[j]==0.0||reset) {
            initx(rtk,1E-6,SQR(rtk->opt.std[1]*bl/1E4),j);
        }
        else {
            /* elevation dependent factor of process noise */
            el=rtk->ssat[sat[i]-1].azel[1];
            fact=cos(el);
            rtk->P[j+j*rtk->nx]+=SQR(rtk->opt.prn[1]*bl/1E4*fact)*fabs(tt);
        }
    }
}
/* temporal update of tropospheric parameters --------------------------------*/
static void udtrop(int reset, rtk_t *rtk, double tt, double bl)
{
    int i,j,k;

    for (i=0;i<2;i++) {
        j=IT(i,&rtk->opt);
        if (rtk->x[j]==0.0||reset) {
            initx(rtk,INIT_ZWD,SQR(rtk->opt.std[2]),j); /* initial zwd */
            if (rtk->opt.tropopt>=TROPOPT_ESTG) {
                for (k=0;k<2;k++) initx(rtk,1E-6,VAR_GRA,++j);
            }
        }
        else {
            rtk->P[j+j*rtk->nx]+=SQR(rtk->opt.prn[2])*fabs(tt);
            if (rtk->opt.tropopt>=TROPOPT_ESTG) {
                for (k=0;k<2;k++) {
                    rtk->P[++j*(1+rtk->nx)]+=SQR(rtk->opt.prn[2]*0.3)*fabs(tt);
                }
            }
        }
    }
}
/* detect cycle slip by LLI --------------------------------------------------*/
static void detslp_ll(rtk_t *rtk, const obsd_t *obs, int i, int rcv)
{
    uint32_t slip,LLI;
    int f,sat=obs[i].sat;

    for (f=0;f<rtk->opt.nf;f++) {
        if (obs[i].L[f]==0.0) continue;

        /* restore previous LLI */
        if (rcv==1) LLI=getbitu(&rtk->ssat[sat-1].slip[f],0,2); /* rover */
        else        LLI=getbitu(&rtk->ssat[sat-1].slip[f],2,2); /* base  */

        /* detect slip by cycle slip flag in LLI */
        if (rtk->tt>=0.0) {
            if (obs[i].LLI[f]&1) {
                log_trace(2,"slip detected  (sat=%2d rcv=%d F=%d LLI=%x)\n",
                        sat,rcv,f+1,obs[i].LLI[f]);
            }
            slip=obs[i].LLI[f];
        }
        /* detect slip by parity unknown flag transition in LLI */
        if (((LLI&2)&&!(obs[i].LLI[f]&2))||(!(LLI&2)&&(obs[i].LLI[f]&2))) {
            log_trace(2,"slip detected half-cyc (sat=%2d rcv=%d F=%d LLI=%x->%x)\n",
                    sat,rcv,f+1,LLI,obs[i].LLI[f]);
            slip|=1;
        }
        /* save current LLI */
        if (rcv==1) setbitu(&rtk->ssat[sat-1].slip[f],0,2,obs[i].LLI[f]);
        else        setbitu(&rtk->ssat[sat-1].slip[f],2,2,obs[i].LLI[f]);

        /* save slip and half-cycle valid flag */
        rtk->ssat[sat-1].slip[f]|=(uint8_t)slip;
        rtk->ssat[sat-1].half[f]=(obs[i].LLI[f]&2)?0:1;
    }
}
/* detect cycle slip by geometry free phase jump -----------------------------*/
static void detslp_gf(rtk_t *rtk, const obsd_t *obs, int i, int j,
                      const nav_t *nav)
{
    int k,sat=obs[i].sat;
    double g0,g1;

    for (k=1;k<rtk->opt.nf;k++) {
        if ((g1=gfobs(obs,i,j,k,nav))==0.0) return;

        g0=rtk->ssat[sat-1].gf[k-1];
        rtk->ssat[sat-1].gf[k-1]=g1;

        if (g0!=0.0&&fabs(g1-g0)>rtk->opt.thresslip) {
            rtk->ssat[sat-1].slip[0]|=1;
            rtk->ssat[sat-1].slip[k]|=1;
            log_trace(2,"slip detected GF jump (sat=%2d L1-L%d GF=%.3f %.3f)\n",sat,k+1,g0,g1);
        }
    }
}
/* detect slip by Melbourne-Wubbena linear combination jump ------------------*/
static void detslp_mw(rtk_t *rtk, const obsd_t *obs, int iu, int ir, const nav_t *nav)
{
    int j,k,sat=obs[iu].sat;
    double w0,w1;

    for (k=1;k<rtk->opt.nf;k++) {
        if ((w1=mwobs(obs,iu,ir,0,k,nav,NULL))==0.0) return;

        w0=rtk->ssat[sat-1].mw[k-1];
        rtk->ssat[sat-1].mw[k-1]=w1;

        log_trace(3,"detslip_mw: sat=%3d f=%d mw0=%12.3f mw1=%12.3f\n",sat,k,w0,w1);

        if (w0!=0.0&&fabs(w1-w0)>THRES_MW_JUMP) {
            log_trace(2,"detslip_mw: slip detected sat=%2d f=%d mw=%8.3f->%8.3f\n",sat,k,w0,w1);
            rtk->ssat[sat-1].slip[0]|=1;
            rtk->ssat[sat-1].slip[k]|=1;
        }
    }
}
/* temporal update of phase biases -------------------------------------------*/
static void udbias(int reset_flag, rtk_t *rtk, double tt, const obsd_t *obs, const int *sat,
                   const int *iu, const int *ir, int ns, const nav_t *nav)
{
    double cp,pr,*bias,freqi,cp1,cp2,pr1,pr2,freq1,freq2,C1,C2;
    int i,j,k,slip,reset,nf=NF(&rtk->opt),f;

    for (i=0;i<ns;i++) {

        /* detect cycle slip by LLI */
        for (k=0;k<rtk->opt.nf;k++) rtk->ssat[sat[i]-1].slip[k]&=0xFC;
        detslp_ll(rtk,obs,iu[i],1);
        detslp_ll(rtk,obs,ir[i],2);

        /* detect cycle slip by geometry-free phase jump */
        detslp_gf(rtk,obs,iu[i],ir[i],nav);

        /* detect cycle slip by MW phase jump */
        detslp_mw(rtk,obs,iu[i],ir[i],nav);

        /* update half-cycle valid flag */
        for (k=0;k<nf;k++) {
            rtk->ssat[sat[i]-1].half[k]=!((obs[iu[i]].LLI[k]&2)||(obs[ir[i]].LLI[k]&2));
            if (rtk->ssat[sat[i]-1].slip[k]&1) {
                rtk->ssat[sat[i]-1].slipc[k]++;
            }
        }
    }
    for (k=0;k<nf;k++) {
        /* reset phase-bias if instantaneous AR or expire obs outage counter */
        for (i=0;i<ns;i++) {
            reset=0;
            if (!rtk->ssat[sat[i]-1].vs) {
                rtk->ssat[sat[i]-1].fix[k]=0;
                reset|=1;
            }
            reset|=reset_flag;
            reset|=(rtk->opt.modear==ARMODE_INST);

            if (reset&&rtk->x[IB(sat[i],k,&rtk->opt)]!=0.0) initx(rtk,0.0,0.0,IB(sat[i],k,&rtk->opt));
            if (reset) {
                rtk->ssat[sat[i]-1].lock[k]=-rtk->opt.minlock;
                rtk->ssat[sat[i]-1].fix[k]=0;
            }
        }
        /* reset phase-bias if detecting cycle slip */
        for (i=0;i<ns;i++) {
            j=IB(sat[i],k,&rtk->opt);
            rtk->P[j+j*rtk->nx]+=rtk->opt.prn[0]*rtk->opt.prn[0]*fabs(tt);
            slip=rtk->ssat[sat[i]-1].slip[k];
            if (rtk->opt.ionoopt==IONOOPT_IFLC) slip|=rtk->ssat[sat[i]-1].slip[0];
            if (rtk->opt.modear==ARMODE_INST||!(slip&1)) continue;
            rtk->x[j]=0.0;

            rtk->ssat[sat[i]-1].lock[k]=-rtk->opt.minlock;
            rtk->ssat[sat[i]-1].fix[k]=0;
        }
        bias=zeros(ns,1);

        /* estimate approximate phase-bias by phase - code */
        for (i=j=0;i<ns;i++) {

            if (rtk->opt.ionoopt!=IONOOPT_IFLC) {
                cp=sdobs(obs,iu[i],ir[i],k); /* cycle */
                pr=sdobs(obs,iu[i],ir[i],k+NFREQ);
                freqi=sat2freq(sat[i],obs[iu[i]].code[k],nav);
                if (cp==0.0||pr==0.0||freqi==0.0) continue;
                bias[i]=cp-pr*freqi/CLIGHT;
            }
            else {
                if (k==0) continue;
                cp1=sdobs(obs,iu[i],ir[i],0);
                cp2=sdobs(obs,iu[i],ir[i],k);
                pr1=sdobs(obs,iu[i],ir[i],NFREQ);
                pr2=sdobs(obs,iu[i],ir[i],NFREQ+k);
                freq1=sat2freq(sat[i],obs[iu[i]].code[0],nav);
                freq2=sat2freq(sat[i],obs[iu[i]].code[k],nav);
                if (cp1==0.0||cp2==0.0||pr1==0.0||pr2==0.0||freq1==0.0||freq2<=0.0) continue;

                C1= SQR(freq1)/(SQR(freq1)-SQR(freq2));
                C2=-SQR(freq2)/(SQR(freq1)-SQR(freq2));
                bias[i]=(C1*cp1*CLIGHT/freq1+C2*cp2*CLIGHT/freq2)-(C1*pr1+C2*pr2);
            }
        }
        /* set initial states of phase-bias */
        for (i=0;i<ns;i++) {
            f=(rtk->opt.ionoopt==IONOOPT_IFLC?k-1:k);
            if (!bias[i]||rtk->x[IB(sat[i],f,&rtk->opt)]!=0.0) continue;
            initx(rtk,bias[i],SQR(rtk->opt.std[0]),IB(sat[i],f,&rtk->opt));
        }
        free(bias);
    }
}
/* temporal update of states --------------------------------------------------*/
static void udstate(int reset, rtk_t *rtk, const obsd_t *obs, const int *sat,
                    const int *iu, const int *ir, int ns, const nav_t *nav)
{
    double tt=rtk->tt,bl,dr[3];

    /* temporal update of position/velocity/acceleration */
    udpos(reset,rtk,tt);

    /* temporal update of ionospheric parameters */
    if (rtk->opt.ionoopt>=IONOOPT_EST) {
        bl=baseline(rtk->x,rtk->rb,dr);
        udion(reset,rtk,tt,bl,sat,ns);
    }
    /* temporal update of tropospheric parameters */
    if (rtk->opt.tropopt>=TROPOPT_EST) {
        udtrop(reset,rtk,tt,bl);
    }
    /* temporal update of phase-bias */
    if (rtk->opt.mode>PMODE_DGPS) {
        udbias(reset,rtk,tt,obs,sat,iu,ir,ns,nav);
    }
}
/* UD (undifferenced) phase/code residual for satellite ----------------------*/
static void zdres_sat(int base, double r, const obsd_t *obs, const nav_t *nav,
                      const double *azel, const double *dant,
                      const prcopt_t *opt, double *y, double *freq)
{
    double freq1,freq2,C1,C2,dant_if;
    int i,nf=NF(opt);

    /* iono-free linear combination */
    if (opt->ionoopt==IONOOPT_IFLC) {
        for (i=1;i<nf;i++) {
            freq1=sat2freq(obs->sat,obs->code[0],nav);
            freq2=sat2freq(obs->sat,obs->code[i],nav);
            if (freq1==0.0||freq2==0.0) return;

            if (testsnr(base,0,azel[1],obs->SNR[0]*SNR_UNIT,&opt->snrmask)||
                testsnr(base,1,azel[1],obs->SNR[i]*SNR_UNIT,&opt->snrmask)) return;

            C1= SQR(freq1)/(SQR(freq1)-SQR(freq2));
            C2=-SQR(freq2)/(SQR(freq1)-SQR(freq2));
            dant_if=C1*dant[0]+C2*dant[i];

            if (obs->L[0]!=0.0&&obs->L[i]!=0.0) {
                y[i-1]=C1*obs->L[0]*CLIGHT/freq1+C2*obs->L[i]*CLIGHT/freq2-r-dant_if;
            }
            if (obs->P[0]!=0.0&&obs->P[i]!=0.0) {
                y[i-1+nf]=C1*obs->P[0]+C2*obs->P[i]-r-dant_if;
            }
            freq[i-1]=1.0;
        }
    }
    else {
        for (i=0;i<nf;i++) {
            if ((freq[i]=sat2freq(obs->sat,obs->code[i],nav))==0.0) continue;

            /* check SNR mask */
            if (testsnr(base,i,azel[1],obs->SNR[i]*SNR_UNIT,&opt->snrmask)) continue;

            /* residuals=observable-pseudorange */
            if (obs->L[i]!=0.0) y[i   ]=obs->L[i]*CLIGHT/freq[i]-r-dant[i];
            if (obs->P[i]!=0.0) y[i+nf]=obs->P[i]               -r-dant[i];
        }
    }
}
/* UD (undifferenced) phase/code residuals -----------------------------------*/
static int zdres(int base, const obsd_t *obs, int n, const double *rs, const double *dts,
                 const double *var, const int *svh, const nav_t *nav, const double *rr,
                 const prcopt_t *opt, int index, double *y, double *e, double *azel, double *freq)
{
    double r,rr_[3],pos[3],dant[NFREQ]={0},disp[3],zhd,zazel[]={0.0,90.0*D2R};
    int i,nf=NF(opt);

    if (norm(rr,3)<=0.0) return 0; /* no receiver position */

    for (i=0;i<3;i++) rr_[i]=rr[i];

    /* earth tide correction */
    if (opt->tidecorr) {
        tidedisp(gpst2utc(obs[0].time),rr_,opt->tidecorr,&nav->erp,opt->odisp[base],disp);
        for (i=0;i<3;i++) rr_[i]+=disp[i];
    }
    ecef2pos(rr_,pos);

    for (i=0;i<n;i++) {
        y[i*nf*2]=y[i*nf*2+1]=0.0;

        /* compute geometric-range and azimuth/elevation angle */
        if ((r=geodist(rs+i*6,rr_,e+i*3))<=0.0) continue;
        if (satazel(pos,e+i*3,azel+i*2)<opt->elmin) continue;

        /* excluded satellite? */
        if (satexclude(obs[i].sat,var[i],svh[i],opt)) continue;

        /* satellite clock-bias */
        r+=-CLIGHT*dts[i*2];

        /* troposphere delay model (hydrostatic) */
        zhd=tropmodel(obs[0].time,pos,zazel,0.0);
        r+=tropmapf(obs[i].time,pos,azel+i*2,NULL)*zhd;

        /* receiver antenna phase center correction */
        antmodel(opt->pcvr+index,opt->antdel[index],azel+i*2,opt->posopt[1],dant);

        /* UD phase/code residual for satellite */
        zdres_sat(base,r,obs+i,nav,azel+i*2,dant,opt,y+i*nf*2,freq+i*nf);
    }
    return 1;
}
/* test valid observation data -----------------------------------------------*/
static int validobs(int i, int j, int f, int nf, const double *y)
{
    /* if no phase observable, psudorange is also unusable */
    return y[f+i*nf*2]!=0.0&&y[f+j*nf*2]!=0.0&&(f<nf||(y[f-nf+i*nf*2]!=0.0&&y[f-nf+j*nf*2]!=0.0));
}
/* DD (double-differenced) measurement error covariance ----------------------*/
static void ddcov(const int *nb, int n, const double *Ri, const double *Rj,
                  int nv, double *R)
{
    int i,j,k=0,b;

    memset(R,0,sizeof(double)*nv*nv);
    for (b=0;b<n;k+=nb[b++]) {
        for (i=0;i<nb[b];i++) {
            for (j=0;j<nb[b];j++) R[k+i+(k+j)*nv]=Ri[k+i]+(i==j?Rj[k+i]:0.0);
        }
    }
}
/* precise tropspheric model -------------------------------------------------*/
static double prectrop(gtime_t time, const double *pos, int r,
                       const double *azel, const prcopt_t *opt, const double *x,
                       double *dtdx)
{
    double m_w=0.0,cotz,grad_n,grad_e;
    int i=IT(r,opt);

    /* wet mapping function */
    tropmapf(time,pos,azel,&m_w);

    if (opt->tropopt>=TROPOPT_ESTG&&azel[1]>0.0) {

        /* m_w=m_0+m_0*cot(el)*(Gn*cos(az)+Ge*sin(az)): ref [6] */
        cotz=1.0/tan(azel[1]);
        grad_n=m_w*cotz*cos(azel[0]);
        grad_e=m_w*cotz*sin(azel[0]);
        m_w+=grad_n*x[i+1]+grad_e*x[i+2];
        dtdx[1]=grad_n*x[i];
        dtdx[2]=grad_e*x[i];
    }
    else dtdx[1]=dtdx[2]=0.0;
    dtdx[0]=m_w;
    return m_w*x[i];
}
/* test satellite system (m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN) ---------*/
static int test_sys(int sys, int m)
{
    switch (sys) {
        case SYS_GPS: return m==0;
        case SYS_SBS: return m==0;
        case SYS_GLO: return m==1;
        case SYS_GAL: return m==2;
        case SYS_CMP: return m==3;
        case SYS_QZS: return m==4;
        case SYS_IRN: return m==5;
    }
    return 0;
}
/* detect measurement outlier on MAD------------------------------------------*/
static int detoutl_MAD(const double *v, int nv, int *outl_ind, double thres)
{
    int i,outl_n=0;
    double m,med;

    if (nv<3) return 0;

    med=median(v,nv); m=mad(v,nv);
    for (i=0;i<nv;i++) {
        if (fabs((v[i]-med)/(1.4826*m))<thres) continue;
        outl_ind[outl_n++]=i;
    }
    return outl_n;
}
/* outlier detect for DD measurement------------------------------------------*/
static void detect_outl(rtk_t *rtk, const double *v, const int *vflg, int nv,
                        double *Ri, double *Rj)
{
    int i,np,nc,n,ip[NFREQ*MAXOBS],ic[NFREQ*MAXOBS],io[NFREQ*MAXOBS];
    double vp[NFREQ*MAXOBS],vc[NFREQ*MAXOBS];

    for (np=nc=i=0;i<nv;i++) {
        if (DD_TYPE(vflg[i])==0) {vp[np]=v[i]; ip[np++]=i;}
        if (DD_TYPE(vflg[i])==1) {vc[nc]=v[i]; ic[nc++]=i;}
    }
    if (np) {
        n=detoutl_MAD(vp,np,io,3.0);
        for (i=0;i<n;i++) {
            if (rtk->ssat[DD_RSAT(vflg[ip[io[i]]])-1].slip[DD_FREQ(vflg[ip[io[i]]])]) continue;
            rtk->ssat[DD_RSAT(vflg[ip[io[i]]])-1].vsat[DD_FREQ(vflg[ip[io[i]]])]=0;
            Rj[ip[io[i]]]=SQR(30.0);
        }
    }
    if (nc) {
        n=detoutl_MAD(vc,nc,io,3.0);
        for (i=0;i<n;i++) Rj[ic[io[i]]]=SQR(30.0);
    }
}
/* generate valid states index------------------------------------------------*/
static int valix(const prcopt_t *opt, const int *vflg, int nv, int *ix)
{
    int i,nx=0,rsat=0,rfrq=0;

    for (i=0;i<NR(opt);i++) ix[nx++]=i;
    for (i=0;i<nv;i++) {
        if (DD_TYPE(vflg[i])!=0) continue;
        ix[nx++]=IB(DD_RSAT(vflg[i]),DD_FREQ(vflg[i]),opt);
        if (DD_BSAT(vflg[i])==rsat&&DD_FREQ(vflg[i])==rfrq) continue;
        rsat=DD_BSAT(vflg[i]);
        rfrq=DD_FREQ(vflg[i]);
        ix[nx++]=IB(DD_BSAT(vflg[i]),DD_FREQ(vflg[i]),opt);
    }
    return nx;
}
/* DD (double-differenced) phase/code residuals ------------------------------*/
static int ddres(rtk_t *rtk, const nav_t *nav, const obsd_t *obs, double dt, const double *x,
                 const double *P, const int *sat, double *y, const double *e,
                 double *azel, const double *freq, const int *iu, const int *ir,
                 int ns, double *v, double *H, double *R, int *vflg, int *ix, int *nx)
{
    prcopt_t *opt=&rtk->opt;
    double bl,dr[3],posu[3],posr[3],didxi=0.0,didxj=0.0,*im,df;
    double tropr[MAXOBS]={0},tropu[MAXOBS]={0},dtdxr[MAXOBS*3]={0},dtdxu[MAXOBS*3]={0};
    double Ri[MAXOBS*NFREQ*2]={0},Rj[MAXOBS*NFREQ*2]={0},frq,*Hi=NULL;
    int i,j,k,m,f,nv=0,nb[NFREQ*4*2+2]={0},b=0,sysi,sysj,nf=NF(opt),bi,bj;

    log_trace(3,"ddres: dt=%.1f nx=%d ns=%d\n",dt,rtk->nx,ns);

    bl=baseline(x,rtk->rb,dr);
    ecef2pos(x,posu); ecef2pos(rtk->rb,posr);

    /* compute factors of ionospheric and tropospheric delay */
    for (i=0;i<ns;i++) {
        if (opt->ionoopt==IONOOPT_EST) {
            im[i]=(ionmapf(posu,azel+iu[i]*2)+ionmapf(posr,azel+ir[i]*2))/2.0;
        }
        if (opt->tropopt>=TROPOPT_EST) {
            tropu[i]=prectrop(rtk->sol.time,posu,0,azel+iu[i]*2,opt,x,dtdxu+i*3);
            tropr[i]=prectrop(rtk->sol.time,posr,1,azel+ir[i]*2,opt,x,dtdxr+i*3);
        }
    }
    for (m=0;m<6;m++) /* m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN */

    for (f=opt->mode>PMODE_DGPS?0:nf;f<nf*2;f++) {

        /* search reference satellite with highest elevation */
        for (i=-1,j=0;j<ns;j++) {
            sysi=satsys(sat[j],NULL);
            if (!test_sys(sysi,m)) continue;
            if (!validobs(iu[j],ir[j],f,nf,y)) continue;
            if (i<0||azel[1+iu[j]*2]>=azel[1+iu[i]*2]) i=j;
        }
        if (i<0) continue;

        /* make DD (double difference) */
        for (j=0;j<ns;j++) {
            if (i==j) continue;
            sysi=satsys(sat[i],NULL); sysj=satsys(sat[j],NULL);
            frq=freq[f%nf+iu[i]*nf];

            if (!test_sys(sysj,m)) continue;
            if (!validobs(iu[j],ir[j],f,nf,y)) continue;
            if (!rtk->ssat[sat[j]-1].vs) continue;

            if (f<nf) {
                rtk->ssat[sat[j]-1].dd[f]=x[IB(sat[i],f,opt)]-x[IB(sat[j],f,opt)];
            }
            rtk->ssat[sat[i]-1].refsat[f%nf]=sat[i];
            rtk->ssat[sat[j]-1].refsat[f%nf]=sat[i];

            if (H) {
                Hi=H+nv*rtk->nx;
                memset(Hi,0,sizeof(double)*rtk->nx);
            }
            /* DD residual */
            v[nv]=(y[f+iu[i]*nf*2]-y[f+ir[i]*nf*2])-(y[f+iu[j]*nf*2]-y[f+ir[j]*nf*2]);

            /* partial derivatives by rover position */
            if (H) {
                for (k=0;k<3;k++) {
                    Hi[k]=-e[k+iu[i]*3]+e[k+iu[j]*3];
                }
            }
            /* DD ionospheric delay term */
            if (opt->ionoopt==IONOOPT_EST) {
                didxi=(f<nf?-1.0:1.0)*im[i]*SQR(FREQ1/frq);
                didxj=(f<nf?-1.0:1.0)*im[j]*SQR(FREQ1/frq);
                v[nv]-=didxi*x[II(sat[i],opt)]-didxj*x[II(sat[j],opt)];
                if (H) {
                    Hi[II(sat[i],opt)]= didxi;
                    Hi[II(sat[j],opt)]=-didxj;
                }
            }
            /* DD tropospheric delay term */
            if (opt->tropopt==TROPOPT_EST||opt->tropopt==TROPOPT_ESTG) {
                v[nv]-=(tropu[i]-tropu[j])-(tropr[i]-tropr[j]);
                for (k=0;k<(opt->tropopt<TROPOPT_ESTG?1:3);k++) {
                    if (!H) continue;
                    Hi[IT(0,opt)+k]= (dtdxu[k+i*3]-dtdxu[k+j*3]);
                    Hi[IT(1,opt)+k]=-(dtdxr[k+i*3]-dtdxr[k+j*3]);
                }
            }
            /* DD phase-bias term */
            if (f<nf) {
                if (opt->ionoopt!=IONOOPT_IFLC) {
                    v[nv]-=CLIGHT/freq[f%nf+iu[i]*nf]*x[IB(sat[i],f,opt)]-CLIGHT/freq[f%nf+iu[j]*nf]*x[IB(sat[j],f,opt)];
                    if (H) {
                        Hi[IB(sat[i],f,opt)]= CLIGHT/freq[f%nf+iu[i]*nf];
                        Hi[IB(sat[j],f,opt)]=-CLIGHT/freq[f%nf+iu[j]*nf];
                    }
                }
                else {
                    v[nv]-=x[IB(sat[i],f,opt)]-x[IB(sat[j],f,opt)];
                    if (H) {
                        Hi[IB(sat[i],f,opt)]= 1.0;
                        Hi[IB(sat[j],f,opt)]=-1.0;
                    }
                }
            }
            if (f<nf) rtk->ssat[sat[j]-1].resc[f%nf]=v[nv];
            else      rtk->ssat[sat[j]-1].resp[f%nf]=v[nv];

            /* test innovation */
            if (opt->maxinno>0.0&&fabs(v[nv])>opt->maxinno) {
                if (f<nf) {
                    rtk->ssat[sat[i]-1].rejc[f]++;
                    rtk->ssat[sat[j]-1].rejc[f]++;
                }
                log_trace(2,"outlier rejected (sat=%3d-%3d %s%d v=%.3f)\n",sat[i],sat[j],f<nf?"L":"P",f%nf+1,v[nv]);
                continue;
            }
            /* SD (single-differenced) measurement error variances */
            Ri[nv]=varerr(sat[i],sysi,azel[1+iu[i]*2],bl,dt,f,opt);
            Rj[nv]=varerr(sat[j],sysj,azel[1+iu[j]*2],bl,dt,f,opt);

            /* set valid data flags */
            if (opt->mode>PMODE_DGPS) {
                if (f<nf) rtk->ssat[sat[i]-1].vsat[f]=rtk->ssat[sat[j]-1].vsat[f]=1;
            }
            else {
                rtk->ssat[sat[i]-1].vsat[f-nf]=rtk->ssat[sat[j]-1].vsat[f-nf]=1;
            }
            char bsat[8],rsat[8];
            satno2id(sat[i],bsat);
            satno2id(sat[j],rsat);

            log_trace(3,"sat=%3d-%3d %4s-%4s %s%d v=%13.3f R=%8.6f %8.6f el=%5.1lf SNR=%5.1lf DD-AMB: %10.3lf fix=%d\n",sat[i],
                      sat[j],bsat,rsat,f<nf?"L":"P",f%nf+1,v[nv],Ri[nv],Rj[nv],rtk->ssat[sat[j]-1].azel[1]*R2D,rtk->ssat[sat[j]-1].snr[f%nf]*SNR_UNIT,
                      f<nf?x[IB(sat[i],f,opt)]-x[IB(sat[j],f,opt)]:0.0,rtk->ssat[sat[j]-1].fix[f%nf]);

            vflg[nv++]=(sat[i]<<16)|(sat[j]<<8)|((f<nf?0:1)<<4)|(f%nf);
            nb[b]++;
        }
        b++;
    }
    /* end of system loop */

    /* detect measurement outlier */
    detect_outl(rtk,v,vflg,nv,Ri,Rj);

    /* generate valid states index */
    if (ix) *nx=valix(opt,vflg,nv,ix);

    /* DD measurement error covariance */
    if (R) ddcov(nb,b,Ri,Rj,nv,R);
    return nv;
}
/* fix double-differnce ambiguity?--------------------------------------------*/
static int is_fixamb(const rtk_t *rtk, const int vflg, double elmask)
{
    int r,b,f;

    if (DD_TYPE(vflg)!=0) return 0;
    b=DD_BSAT(vflg);
    r=DD_RSAT(vflg);
    f=DD_FREQ(vflg);

    if (!rtk->ssat[r-1].vsat[f]) return 0;
    if (!rtk->ssat[r-1].half[f]||!rtk->ssat[b-1].half[f]) return 0;
    if (rtk->ssat[r-1].lock[f]<0) return 0;
    if (rtk->ssat[r-1].azel[1]<elmask||rtk->ssat[r-1].snr[f]*SNR_UNIT<rtk->opt.elmasksnr) {
        return 0;
    }
    return 1;
}
/* index for SD to DD transformation matrix D --------------------------------*/
static int ddidx(rtk_t *rtk, const int *vflg, int nv, int *ix, int *ind)
{
    double elmask=rtk->opt.elmaskar*D2R;
    int i,nb=0;

    for (i=0;i<nv;i++) {
        if (!is_fixamb(rtk,vflg[i],elmask)) continue;
        ix[nb*2+0]=IB(DD_BSAT(vflg[i]),DD_FREQ(vflg[i]),&rtk->opt);
        ix[nb*2+1]=IB(DD_RSAT(vflg[i]),DD_FREQ(vflg[i]),&rtk->opt);
        ind[nb++]=i;
        rtk->ssat[DD_RSAT(vflg[i])-1].fix[DD_FREQ(vflg[i])]=1;
        rtk->ssat[DD_BSAT(vflg[i])-1].fix[DD_FREQ(vflg[i])]=1;
    }
    return nb;
}
/* restore SD (single-differenced) ambiguity ---------------------------------*/
static int restamb(rtk_t *rtk, const int *vflg, const double *bias, const int *ind, int nb, double *xa)
{
    int b,r,f,f1,f2,i,k;
    matcpy(xa,rtk->xa,1,rtk->na);
    matcpy(xa,rtk->x,1,rtk->nx);

    for (i=0;i<nb;i++) {
        b=DD_BSAT(vflg[ind[i]]);
        r=DD_RSAT(vflg[ind[i]]);
        f=DD_FREQ(vflg[ind[i]]);
        xa[IB(r,f,&rtk->opt)]=xa[IB(b,f,&rtk->opt)]-bias[i];
        rtk->ssat[r-1].fix[f]=2;
        rtk->ssat[b-1].fix[f]=2;
    }
    return nb;
}
/* hold integer ambiguity ----------------------------------------------------*/
static void holdamb(rtk_t *rtk, const double *xa, const int *vflg, const int *ind, int nb, const int *ix, int nx)
{
    prcopt_t *opt=&rtk->opt;
    double *v,*H,*R,*var,b1,b2;
    int i,n,nv,info,b,r,f;

    v=mat(nb,1); H=zeros(nb,rtk->nx); var=mat(nb,1);

    for (i=nv=0;i<nb;i++) {
        b=DD_BSAT(vflg[ind[i]]);
        r=DD_RSAT(vflg[ind[i]]);
        f=DD_FREQ(vflg[ind[i]]);
        v[nv]=(xa[IB(b,f,&rtk->opt)]-xa[IB(r,f,&rtk->opt)])-(rtk->x[IB(b,f,&rtk->opt)]-rtk->x[IB(r,f,&rtk->opt)]);

        H[IB(b,f,&rtk->opt)+nv*rtk->nx]= 1.0;
        H[IB(r,f,&rtk->opt)+nv*rtk->nx]=-1.0;
        var[nv]=rtk->ssat[r-1].lflg[f]?3.0*VAR_HOLDAMB:VAR_HOLDAMB;
        rtk->ssat[r-1].fix[f]=3;
        rtk->ssat[b-1].fix[f]=3;
        nv++;
    }
    if (nv>0) {
        R=zeros(nv,nv);
        for (i=0;i<nv;i++) R[i+i*nv]=var[i];

        /* update states with constraints */
        if ((info=filter(rtk->x,rtk->P,H,v,R,rtk->nx,nv,ix,nx))) {
            log_trace(1,"filter error (info=%d)\n",info);
        }
        free(R);
    }
    free(v); free(H); free(var);
}
/* extract double-difference ambiguity Qb/y/Qab-------------------------------*/
static void resamb_Qy(rtk_t *rtk, const int *ix, int nb, double *y, double *Qb, double *Qab)
{
    int i,j,nx=rtk->nx,na=rtk->na;
    double *DP;

    DP=mat(nb,nx-na);

    for (i=0;y&&i<nb;i++) y[i]=rtk->x[ix[i*2]]-rtk->x[ix[i*2+1]];

    for (j=0;j<nx-na;j++) for (i=0;i<nb;i++) {
        DP[i+j*nb]=rtk->P[ix[i*2]+(na+j)*nx]-rtk->P[ix[i*2+1]+(na+j)*nx];
    }
    for (j=0;Qb&&j<nb;j++) for (i=0;i<nb;i++) {
        Qb[i+j*nb]=DP[i+(ix[j*2]-na)*nb]-DP[i+(ix[j*2+1]-na)*nb];
    }
    for (j=0;Qab&&j<nb;j++) for (i=0;i<na;i++) {
        Qab[i+j*na]=rtk->P[i+ix[j*2]*nx]-rtk->P[i+ix[j*2+1]*nx];
    }
    free(DP);
}
/* fixamb on LAMBDA-----------------------------------------------------------*/
static double fixamb(rtk_t *rtk, const int *vflg, const int *ind, const int *ix, int nb,
                     double *bias, double *y, double *Qb, double *Qab, double thresar)
{
    int i,j,info,nx=rtk->nx,na=rtk->na;
    double *b=mat(nb,2),s[2],ratio=0.0;

    resamb_Qy(rtk,ix,nb,y,Qb,Qab);

    log_trace(3,"y=\n"); log_tracemat(3,y,1,nb,12,5);
    log_trace(3,"Qb=\n"); log_tracemat(3,Qb,nb,nb,12,5);

    /* LAMBDA/MLAMBDA ILS (integer least-square) estimation */
    if ((info=lambda(nb,2,y,Qb,b,s))) {
        log_trace(1,"lambda error (info=%d)\n",info);
        free(b);
        return 0.0;
    }
    ratio=s[0]>0?(float)(s[1]/s[0]):0.0f;
    ratio=ratio>999.0?999.0:ratio;

    log_trace(3,"b0=\n"); log_tracemat(3,b,1,nb,10,3);
    log_trace(3,"b1=\n"); log_tracemat(3,b+nb,1,nb,10,3);

    /* validation by popular ratio-test */
    if (s[0]>0.0&&s[1]/s[0]>=thresar) {
        matcpy(bias,b,1,nb);
        log_trace(3,"fixamb: validation ok (nb=%d ratio=%.2f s=%.2f/%.2f)\n",
                  nb,s[0]==0.0?0.0:s[1]/s[0],s[0],s[1]);
    }
    else {
        /* validation failed */
        log_trace(2,"ambiguity validation failed (nb=%d ratio=%.2f s=%.2f/%.2f)\n",
                  nb,s[1]/s[0],s[0],s[1]);
    }
    free(b); return ratio;
}
/* inherit double-differenced ambiguity---------------------------------------*/
static int inherit_amb(rtk_t *rtk, const nav_t *nav, const obsd_t *obs, const int *ir, const int *iu,
                       const int *vflg, int nv, int *amb_ind, double *bias, double *xa, int *ix)
{
    int i,nb,na=rtk->na,m,ind[2*MAXOBS],ixb[2*MAXOBS];

    if (rtk->opt.modear!=ARMODE_FIXHOLD||rtk->sol.ratio<rtk->opt.thresar[0]) {
        return 0;
    }
    for (nb=i=0;i<nv;i++) {
        if (rtk->ssat[DD_RSAT(vflg[i])-1].fix[DD_FREQ(vflg[i])]!=3) continue;
        if (DD_TYPE(vflg[i])) continue;

        ix[nb*2+0]=IB(DD_BSAT(vflg[i]),DD_FREQ(vflg[i]),&rtk->opt);
        ix[nb*2+1]=IB(DD_RSAT(vflg[i]),DD_FREQ(vflg[i]),&rtk->opt);
        bias[nb]=ROUND(rtk->x[ix[nb*2]]-rtk->x[ix[nb*2+1]]);
        amb_ind[nb++]=i;
    }
    if (nb<=0) return 0;

    for (m=i=0;i<nv;i++) {
        if (rtk->ssat[DD_RSAT(vflg[i])-1].fix[DD_FREQ(vflg[i])]==3) continue;
        if (!is_fixamb(rtk,vflg[i],rtk->opt.elmin)) continue;

        ixb[m*2+0]=IB(DD_BSAT(vflg[i]),DD_FREQ(vflg[i]),&rtk->opt);
        ixb[m*2+1]=IB(DD_RSAT(vflg[i]),DD_FREQ(vflg[i]),&rtk->opt);
        ind[m++]=i;
    }
    if (m) {
        double *b=mat(1,m),*y=mat(m,1),*Qb=mat(m,m);
        double ratio,thresar=rtk->opt.thresar[0];
#if ADJ_AR_RATIO
        thresar=adj_arratio(&rtk->opt,m);
#endif
        if ((ratio=fixamb(rtk,vflg,ind,ixb,m,b,y,Qb,NULL,thresar))>thresar) {
            for (i=0;i<m;i++) {
                ix[2*nb+0]=ixb[2*i+0];
                ix[2*nb+1]=ixb[2*i+1];
                bias[nb]=b[i]; amb_ind[nb++]=ind[i];
            }
        }
        rtk->sol.ratio=MAX(rtk->sol.ratio,ratio);
        free(b); free(y); free(Qb);
    }
    return nb;
}
/* transform float to fixed solution (xa=xa-Qab*Qb\(b0-b))--------------------*/
static int float2fix(int hold, rtk_t *rtk, const int *vflg, int nb, int *amb_ind, int *ix, double *bias, double *xa,
                     double *y, double *Qb, double *Qab)
{
    int i,j,na=rtk->na,nx=rtk->nx;
    double *db,*QQ;

    for (i=0;i<na;i++) {
        rtk->xa[i]=rtk->x[i];
        for (j=0;j<na;j++) rtk->Pa[i+j*na]=rtk->P[i+j*nx];
    }
    /* restore SD ambiguity */
    nb=restamb(rtk,vflg,bias,amb_ind,nb,xa);
    if (hold) return nb;

    if (rtk->opt.mode==PMODE_FIXED) return nb;
    for (i=0;i<nb;i++) y[i]-=bias[i];

    QQ=mat(na,nb); db=mat(nb,1);

    if (matinv(Qb,nb)) {
        free(db); free(QQ);
        return 0;
    }
    matmul("NN",nb,1,nb, 1.0,Qb ,y,0.0,db);
    matmul("NN",na,1,nb,-1.0,Qab,db,1.0,rtk->xa);

    /* covariance of fixed solution (Qa=Qa-Qab*Qb^-1*Qab') */
    matmul("NN",na,nb,nb, 1.0,Qab,Qb ,0.0,QQ);
    matmul("NT",na,na,nb,-1.0,QQ ,Qab,1.0,rtk->Pa);
    free(db); free(QQ);
    return nb;
}
/* resolve integer ambiguity by LAMBDA ---------------------------------------*/
static int resamb_LAMBDA(rtk_t *rtk, const nav_t *nav, const obsd_t *obs, const int *ir, const int *iu,
                         const int *vflg, int nv, int *amb_ind, double *bias, double *xa)
{
    prcopt_t *opt=&rtk->opt;
    int i,j,nb,info,nx=rtk->nx,na=rtk->na,*ix;
    double *DP,*y,*b,*Qb,*Qab,s[2],ratio,thresar=rtk->opt.thresar[0];

    if (rtk->opt.mode<=PMODE_DGPS||rtk->opt.modear==ARMODE_OFF||rtk->opt.ionoopt==IONOOPT_IFLC||
        thresar<1.0) {
        return 0;
    }
    ix=imat(nx,2);

#if INHERIT_AMB
    if ((nb=inherit_amb(rtk,nav,obs,ir,iu,vflg,nv,amb_ind,bias,xa,ix))) {
#if CONST_FIX_INHERIT_AMB
        y=mat(nb,1); Qb=mat(nb,nb); Qab=mat(na,nb);

        resamb_Qy(rtk,ix,nb,y,Qb,Qab);
        nb=float2fix(0,rtk,vflg,nb,amb_ind,bias,xa,y,Qb,Qab);
        rtk->nb=nb;

        free(y); free(Qb); free(Qab); free(ix);
        log_trace(3,"inherit double-differenced ambiguity: nb=%d\n",nb);
        return nb;
#else
        nb=float2fix(1,rtk,vflg,nb,amb_ind,ix,bias,xa,y,Qb,Qab);
        rtk->nb=nb;

        free(ix);
        log_trace(3,"inherit double-differenced ambiguity: nb=%d\n",nb);
        return nb;
#endif
    }
#endif
    rtk->sol.ratio=0.0f;

    /* index of SD to DD transformation matrix D */
    if ((nb=ddidx(rtk,vflg,nv,ix,amb_ind))<=0) {
        log_trace(2,"no valid double-difference\n");
        free(ix);
        return 0;
    }
#if ADJ_AR_RATIO
    thresar=adj_arratio(&rtk->opt,nb);
#endif
    y=mat(nx,1); b=mat(nx,2); Qb=mat(nx,nx);
    Qab=mat(na,nx);

    if ((ratio=fixamb(rtk,vflg,amb_ind,ix,nb,bias,y,Qb,Qab,thresar))>thresar) {
        nb=float2fix(0,rtk,vflg,nb,amb_ind,ix,bias,xa,y,Qb,Qab);
        rtk->sol.ratio=(float)ratio;
    }
    else nb=0;

    rtk->nb=nb;
    free(ix); free(y);
    free(b); free(Qb); free(Qab);
    return nb; /* number of ambiguities */
}
/* validation of solution ----------------------------------------------------*/
static int valpos(rtk_t *rtk, const double *v, const double *R, const int *vflg,
                  int nv, double thres)
{
    int i,n=0,sat1,sat2,type,freq;
    double fact=thres*thres;

    log_trace(3,"valpos  : nv=%d thres=%.1f\n",nv,thres);

    /* post-fit residual test */
    for (i=0;i<nv;i++) {
        if (v[i]*v[i]<=fact*R[i+i*nv]) continue;
        sat1=DD_BSAT(vflg[i]);
        sat2=DD_RSAT(vflg[i]);
        type=DD_TYPE(vflg[i]);
        freq=DD_FREQ(vflg[i]);

        log_trace(2,"large residual (sat=%2d-%2d %s%d v=%6.3f sig=%.3f)\n",
                  sat1,sat2,type==0?"L":(type==1?"P":"C"),freq+1,v[i],SQRT(R[i+i*nv]));

        if (type==0) {
            rtk->ssat[sat1-1].lflg[freq]=1;
            rtk->ssat[sat2-1].lflg[freq]=1;
            n++;
        }
    }
    return n<(nv/3*2);
}
/* update satellite position/velocity/clcok-----------------------------------*/
static void udsatpos(gtime_t time, rtk_t *rtk, const obsd_t *obs, int nu, int nr, const nav_t *nav,
                     double *rs, double *dts, double *var, int *svh)
{
    int i;
    for (i=0;i<nu;i++) {
        matcpy(rs+6*i,rtk->ssat[obs[i].sat-1].rs,1,6);
        matcpy(dts+2*i,rtk->ssat[obs[i].sat-1].dts,1,2);
        var[i]=rtk->ssat[obs[i].sat-1].var;
        svh[i]=rtk->ssat[obs[i].sat-1].svh;
    }
    satposs(time,obs+nu,nr,nav,rtk->opt.sateph,rs+6*nu,dts+2*nu,var+nu,svh+nu);
}
/* update rtk solution ------------------------------------------------------*/
static void udrtksol(rtk_t *rtk, const int *sat, int ns, int stat)
{
    int i,na=rtk->na,nx=rtk->nx,f,nf=NF(&rtk->opt);

    rtk->sol.ns=0;
    for (i=0;i<ns;i++) for (f=0;f<nf;f++) {
        if (!rtk->ssat[sat[i]-1].vsat[f]) continue;
        rtk->ssat[sat[i]-1].lock[f]++;
        if (f==0) rtk->sol.ns++; /* valid satellite count by L1 */
    }
    /* lack of valid satellites */
    if (rtk->sol.ns<4) stat=SOLQ_NONE;

    if (stat==SOLQ_FIX) {
        for (i=0;i<3;i++) {
            rtk->sol.rr[i]=rtk->xa[i];
            rtk->sol.qr[i]=(float)rtk->Pa[i+i*na];
        }
        rtk->sol.qr[3]=(float)rtk->Pa[1];
        rtk->sol.qr[4]=(float)rtk->Pa[1+2*na];
        rtk->sol.qr[5]=(float)rtk->Pa[2];
        rtk->sol.stat=SOLQ_FIX;
        rtk->nfix++;
    }
    else if (stat==SOLQ_FLOAT||stat==SOLQ_DGPS) {
        for (i=0;i<3;i++) {
            rtk->sol.rr[i]=rtk->x[i];
            rtk->sol.qr[i]=(float)rtk->P[i+i*nx];
        }
        rtk->sol.qr[3]=(float)rtk->P[1];
        rtk->sol.qr[4]=(float)rtk->P[1+2*nx];
        rtk->sol.qr[5]=(float)rtk->P[2];
        rtk->sol.stat=stat;
    }
    if (stat!=SOLQ_FIX) rtk->nfix=0;
    if (stat!=SOLQ_NONE) {
        rtk->sol.stat=stat;
        matcpy(rtk->rr,rtk->sol.rr,1,3);
        for (i=0;i<3;i++) rtk->bl[i]=rtk->rr[i]-rtk->rb[i];
    }
}
/* update satellite status----------------------------------------------------*/
static void udssat(rtk_t *rtk, const obsd_t *obs, int nu)
{
    ssat_t ssat[MAXSAT];
    int i;

    memcpy(ssat,rtk->ssat,sizeof(ssat_t)*MAXSAT);
    memset(rtk->ssat,0,sizeof(ssat_t)*MAXSAT);
    for (i=0;i<nu;i++) {
        rtk->ssat[obs[i].sat-1]=ssat[obs[i].sat-1];
    }
}
/* relative positioning ------------------------------------------------------*/
static int relpos(rtk_t *rtk, const obsd_t *obs, int nu, int nr, const nav_t *nav)
{
    prcopt_t *opt=&rtk->opt;
    double rs[MAXOBS*2*6]={0},dts[MAXOBS*2*2]={0},var[MAXOBS*2]={0},y[MAXOBS*4*NFREQ]={0};
    double e[MAXOBS*2*3]={0},azel[MAXOBS*2*2]={0},freq[MAXOBS*2*NFREQ]={0};
    double *v,*H,*R,*xp,*Pp,*xa,*bias,dt;
    int i,j,f,nb,r,b,n=nu+nr,ns,nv,sat[MAXOBS*2],iu[MAXOBS*2],ir[MAXOBS*2],amb_ind[MAXOBS*NF(opt)],niter;
    int info,vflg[MAXOBS*NFREQ*2*2+1],svh[MAXOBS*2],reset=0,nf=NF(opt),ix[NX(opt)],nx;
    int stat=rtk->opt.mode<=PMODE_DGPS?SOLQ_DGPS:SOLQ_FLOAT;

    log_trace(3,"relpos: nx=%d nu=%d nr=%d\n",rtk->nx,nu,nr);

    for (i=0;i<nu+nr;i++) {
        for (j=0;j<NFREQ;j++) rtk->ssat[obs[i].sat-1].vsat[j]=0;
        for (j=0;j<NFREQ;j++) rtk->ssat[obs[i].sat-1].lflg[j]=0;
        for (j=0;j<NFREQ;j++) rtk->ssat[obs[i].sat-1].slip[j]=0;
    }
    /* check base observation data */
    if (nr==0) {
        log_trace(2,"no base station observation data for rtk\n");
        return 1;
    }
    /* check age of differential */
    rtk->sol.age=dt=timediff(obs[0].time,obs[nu].time);

    if (fabsf(rtk->sol.age)>rtk->opt.maxtdiff) {
        log_trace(2,"age of differential error (age=%.1f)\n",rtk->sol.age);
        return 1;
    }
    /* satellite positions/clocks */
    udsatpos(obs[0].time,rtk,obs,nu,nr,nav,rs,dts,var,svh);

    /* UD (undifferenced) residuals for base station */
    if (!zdres(1,obs+nu,nr,rs+nu*6,dts+nu*2,var+nu,svh+nu,nav,rtk->rb,opt,1,y+nu*nf*2,e+nu*3,azel+nu*2,freq+nu*nf)) {
        log_trace(1,"initial base station position error\n");
        return 0;
    }
    /* select common satellites between rover and base-station */
    if ((ns=selsat(rtk,obs,azel,nu,nr,opt,sat,iu,ir))<=0) {
        log_trace(2,"no common satellite\n");
        return 0;
    }
    xp=rtk->xp; Pp=rtk->Pp; xa=rtk->xl;
    v=rtk->v; H=rtk->H; R=rtk->R; bias=rtk->bias;

__reset_rtk:
    /* temporal update of states */
    udstate(reset,rtk,obs,sat,iu,ir,ns,nav);
    matcpy(xp,rtk->x,rtk->nx,1);

    for (i=0;i<opt->niter;i++) {
        /* UD (undifferenced) residuals for rover */
        if (!zdres(0,obs,nu,rs,dts,var,svh,nav,xp,opt,0,y,e,azel,freq)) {
            log_trace(1,"rover initial position error\n");
            stat=SOLQ_NONE;
            break;
        }
        /* DD (double-differenced) residuals and partial derivatives */
        if ((nv=ddres(rtk,nav,obs,dt,xp,Pp,sat,y,e,azel,freq,iu,ir,ns,v,H,R,vflg,ix,&nx))<1) {
            log_trace(1,"no double-differenced residual\n");
            stat=SOLQ_NONE;
            break;
        }
        /* Kalman filter measurement update */
        matcpy(Pp,rtk->P,rtk->nx,rtk->nx);
        if ((info=filter(xp,Pp,H,v,R,rtk->nx,nv,ix,nx))) {
            log_trace(1,"filter error (info=%d)\n",info);
            stat=SOLQ_NONE;
            break;
        }
    }
    if (stat!=SOLQ_NONE&&zdres(0,obs,nu,rs,dts,var,svh,nav,xp,opt,0,y,e,azel,freq)) {

        /* post-fit residuals for float solution */
        nv=ddres(rtk,nav,obs,dt,xp,Pp,sat,y,e,azel,freq,iu,ir,ns,v,NULL,R,vflg,NULL,NULL);

        /* validation of float solution */
        if (valpos(rtk,v,R,vflg,nv,4.0)) {

            /* update state and covariance matrix */
            matcpy(rtk->x,xp,rtk->nx,1);
            matcpy(rtk->P,Pp,rtk->nx,rtk->nx);
        }
        else {
            if (++reset<=1) goto __reset_rtk;
            stat=SOLQ_NONE;
        }
    }
    /* resolve integer ambiguity by LAMBDA */
    if (stat!=SOLQ_NONE&&(nb=resamb_LAMBDA(rtk,nav,obs,ir,iu,vflg,nv,amb_ind,bias,xa))>1) {

        if (zdres(0,obs,nu,rs,dts,var,svh,nav,xa,opt,0,y,e,azel,freq)) {

            /* post-fit residuals for fixed solution */
            nv=ddres(rtk,nav,obs,dt,xa,NULL,sat,y,e,azel,freq,iu,ir,ns,v,NULL,R,vflg,NULL,NULL);

            /* validation of fixed solution */
            if (valpos(rtk,v,R,vflg,nv,4.0)) {

                /* hold integer ambiguity */
                if (++rtk->nfix>=rtk->opt.minfix&&rtk->opt.modear==ARMODE_FIXHOLD) {
                    holdamb(rtk,xa,vflg,amb_ind,nb,ix,nx);
                }
                stat=SOLQ_FIX;
            }
        }
    }
    /* update RTK solution status */
    udrtksol(rtk,sat,ns,stat);

    /* update satellite status */
    udssat(rtk,obs,nu);
    return stat;
}
/* initialize RTK control ------------------------------------------------------
* initialize RTK control struct
* args   : rtk_t    *rtk    IO  TKk control/result struct
*          prcopt_t *opt    I   positioning options (see rtklib.h)
* return : none
*-----------------------------------------------------------------------------*/
extern void rtkinit(rtk_t *rtk, const prcopt_t *opt)
{
    sol_t sol0={{0}};
    ambc_t ambc0={{{0}}};
    ssat_t ssat0={0};
    int i;

    rtk->sol=sol0;
    for (i=0;i<6;i++) rtk->rb[i]=0.0;
    rtk->nx=opt->mode<=PMODE_FIXED?NX(opt):pppnx(opt);
    rtk->na=opt->mode<=PMODE_FIXED?NR(opt):pppnx(opt);
    rtk->tt=0.0;
    rtk->x =zeros(rtk->nx,1);
    rtk->P =zeros(rtk->nx,rtk->nx);
    rtk->xa=zeros(rtk->na,1);
    rtk->Pa=zeros(rtk->na,rtk->na);
    rtk->xp=zeros(rtk->nx,1);
    rtk->Pp=zeros(rtk->nx,rtk->nx);
    rtk->xl=zeros(rtk->nx,1);
    rtk->bias=zeros(rtk->nx,1);
    rtk->v=zeros(MAXOBS*NFREQ*4,1);
    rtk->H=zeros(MAXOBS*NFREQ*4,rtk->nx);
    rtk->R=zeros(MAXOBS*NFREQ*4,MAXOBS*NFREQ*2);
    rtk->nfix=0;
    for (i=0;i<MAXSAT;i++) {
        rtk->ambc[i]=ambc0; rtk->ssat[i]=ssat0;
    }
    rtk->opt=*opt;
}
/* free rtk control ------------------------------------------------------------
* free memory for rtk control struct
* args   : rtk_t    *rtk    IO  rtk control/result struct
* return : none
*-----------------------------------------------------------------------------*/
extern void rtkfree(rtk_t *rtk)
{
    rtk->nx=rtk->na=0;
    free(rtk->x ); rtk->x =NULL;
    free(rtk->P ); rtk->P =NULL;
    free(rtk->xa); rtk->xa=NULL;
    free(rtk->Pa); rtk->Pa=NULL;
    free(rtk->xl); rtk->xl=NULL;
    free(rtk->Pp); rtk->Pp=NULL;
    free(rtk->v ); rtk->v =NULL;
    free(rtk->H ); rtk->H =NULL;
    free(rtk->R ); rtk->R =NULL;
    free(rtk->bias); rtk->bias=NULL;
}
/* update RTK time-------------------------------------------------------------*/
static void udrtktime(rtk_t *rtk, gtime_t tr, gtime_t tb)
{
    if (timediff(tr,tb)>0.0) rtk->time=tr;
    else rtk->time=tb;
}
/* precise positioning ---------------------------------------------------------
 * input observation data and navigation message, compute rover position by
 * RTK positioning
 * args   : rtk_t *rtk       IO  RTK control/result struct
 *          obs_t *robs      I   rover observation data
 *          obs_t *bobs      I   base observation data
 *          nav_t  *nav      I   navigation messages
 * return : status (0:no solution,1:valid solution)
 * notes  : before calling function, base station position rtk->sol.rb[] should
 *          be properly set for relative mode except for moving-baseline
 *-----------------------------------------------------------------------------*/
extern int rtkpos(rtk_t *rtk, const obs_t *robs, const obs_t *bobs, const nav_t *nav)
{
    obsd_t obs[2*MAXOBS]={0};
    gtime_t time=rtk->sol.time,tr={0},tb={0};
    char msg[128]="";
    int i,j,nu=0,nr=0,stat;

    /* update rover observation data */
    if (robs&&robs->n) {
        tr=robs->data[0].time;
        nu=robs->n;
        memcpy(obs,robs->data,sizeof(obsd_t)*nu);
    }
    /* update base observation data */
    if (bobs&&bobs->n) {
        tb=bobs->data[0].time;
        nr=bobs->n;
        memcpy(obs+nu,bobs->data,sizeof(obsd_t)*nr);
    }
    /* rover position by single point positioning */
    if (!pntpos(obs,nu,nav,&rtk->opt,&rtk->sol,NULL,rtk->ssat,msg)) {
        log_trace(1,"point pos error (%s)\n",msg);
        udrtktime(rtk,tr,tb);
        return 0;
    }
    if (time.time!=0) rtk->tt=timediff(rtk->sol.time,time);

    /* single point positioning */
    if (rtk->opt.mode==PMODE_SINGLE) {
        udrtktime(rtk,tr,tb);
        return 1;
    }
    /* relative positioning */
    stat=relpos(rtk,obs,nu,nr,nav);

    /* update RTK solution time */
    udrtktime(rtk,tr,tb);
    return stat;
}
