/* rtk algorithm helpers -----------------------------------------------------*/
#include "rtklib.h"

#ifndef USE_PAR_FFRT
#define USE_PAR_FFRT 1      /* temporary switch: 0=LAMBDA, 1=PAR+B-FFRT */
#endif

#define REFSEL_ELMIN (45.0*D2R) /* min elevation for geometry reference sat */
#define REFSEL_SNRMIN 35.0   /* min rover/base snr for geometry reference */
#define REFSEL_ALPHA 0.2     /* variance term weight for geometry score */
#define REFSEL_ETA   0.1     /* condition number switching margin */
#define REFSEL_DJ    0.1     /* score switching margin */
#define REFSEL_EPS   1E-12   /* singularity threshold */

#ifndef MIN
#define MIN(x,y)    ((x)<=(y)?(x):(y))
#endif
#ifndef MAX
#define MAX(x,y)    ((x)>=(y)?(x):(y))
#endif

extern double varerr(int sat, int sys, double el, double snr_rover,
                     double snr_base, double bl, double dt, int f,
                     const prcopt_t *opt, const obsd_t *obs);
extern double baseline(const double *ru, const double *rb, double *dr);
extern int validobs(int i, int j, int f, int nf, double *y);
extern int test_sys(int sys, int m);

#ifdef RTKPOS_INCLUDE_RTKALG

#define PAR_SUCCESS_RATE_THRES 0.995 /* bootstrapping success-rate threshold */
#define PAR_MIN_AMB_DIM       8      /* min fixed DD ambiguity subset size */
#define PAR_BPD_MAX           10.0   /* max baseline precision defect */
#define PAR_FFRT_CMIN         2.0    /* bounded FFRT minimum ratio */
#define PAR_RATIO_EPS         1E-12  /* ratio denominator protection */
#define PAR_FFRT_XMIN         1E-6   /* minimum ILS failure-rate estimate */
#define PAR_FFRT_XMAX         0.2    /* failure-rate rejection threshold */

static void errmsg(rtk_t *rtk, const char *format, ...);
static int ddidx(rtk_t *rtk, int *ix, int gps, int glo, int sbs);

static const double par_ffrt_coeffs[62][3]={
    {0.3364,-0.2968,-0.3335},{0.4401,-0.2435,-0.3686},
    {0.3794,-0.2521,-0.2291},{0.2904,-0.2793,-0.0730},
    {0.2874,-0.2702,-0.0146},{0.1797,-0.3314, 0.1593},
    {0.1569,-0.3439, 0.2290},{0.1310,-0.3615, 0.2998},
    {0.0793,-0.4428, 0.3928},{0.0839,-0.4222, 0.4166},
    {0.0721,-0.4411, 0.4563},{0.0700,-0.4381, 0.4825},
    {0.0664,-0.4378, 0.5096},{0.0645,-0.4339, 0.5321},
    {0.0674,-0.4175, 0.5449},{0.0683,-0.4074, 0.5598},
    {0.0647,-0.4090, 0.5783},{0.0659,-0.3980, 0.5912},
    {0.0661,-0.3910, 0.6039},{0.0514,-0.4286, 0.6342},
    {0.0519,-0.4202, 0.6435},{0.0529,-0.4098, 0.6531},
    {0.0425,-0.4442, 0.6762},{0.0381,-0.4575, 0.6916},
    {0.0458,-0.4183, 0.6885},{0.0386,-0.4443, 0.7059},
    {0.0387,-0.4380, 0.7124},{0.0385,-0.4329, 0.7204},
    {0.0384,-0.4287, 0.7267},{0.0393,-0.4191, 0.7318},
    {0.0360,-0.4300, 0.7419},{0.0392,-0.4103, 0.7426},
    {0.0345,-0.4277, 0.7549},{0.0323,-0.4356, 0.7627},
    {0.0300,-0.4443, 0.7704},{0.0286,-0.4493, 0.7759},
    {0.0264,-0.4594, 0.7842},{0.0245,-0.4695, 0.7904},
    {0.0267,-0.4501, 0.7905},{0.0254,-0.4545, 0.7966},
    {0.0249,-0.4550, 0.8004},{0.0249,-0.4505, 0.8036},
    {0.0269,-0.4332, 0.8037},{0.0237,-0.4527, 0.8119},
    {0.0250,-0.4390, 0.8129},{0.0255,-0.4322, 0.8148},
    {0.0259,-0.4265, 0.8167},{0.0231,-0.4418, 0.8240},
    {0.0217,-0.4504, 0.8280},{0.0220,-0.4457, 0.8305},
    {0.0253,-0.4180, 0.8279},{0.0211,-0.4461, 0.8367},
    {0.0193,-0.4585, 0.8414},{0.0166,-0.4850, 0.8472},
    {0.0243,-0.4120, 0.8373},{0.0179,-0.4638, 0.8492},
    {0.0205,-0.4360, 0.8478},{0.0195,-0.4434, 0.8505},
    {0.0145,-0.4951, 0.8605},{0.0166,-0.4634, 0.8581},
    {0.0149,-0.4873, 0.8628},{0.0071,-0.6131, 0.8773}
};

/* standard normal cdf approximation ----------------------------------------*/
static double par_norm_cdf(double x)
{
    double t,d,prob;
    int neg=x<0.0;

    if (neg) x=-x;
    t=1.0/(1.0+0.2316419*x);
    d=0.3989422804014327*exp(-0.5*x*x);
    prob=1.0-d*t*(0.319381530+t*(-0.356563782+
         t*(1.781477937+t*(-1.821255978+t*1.330274429))));
    return neg?1.0-prob:prob;
}

/* bootstrapping success rate from reduced z-space conditional variances ------*/
static double par_bootstrap_success(int p, const double *D)
{
    double ps=1.0,arg,prob;
    int i;

    for (i=0;i<p;i++) {
        if (D[i]<=0.0) return 0.0;
        arg=0.5/sqrt(D[i]);
        prob=2.0*par_norm_cdf(arg)-1.0;
        if (prob<0.0) prob=0.0;
        else if (prob>1.0) prob=1.0;
        ps*=prob;
    }
    return ps;
}

/* B-FFRT threshold for ratio definition R=s2/s1 ----------------------------*/
static int par_bffrt_threshold(int p, double ps, double *thres)
{
    const double *c;
    double x,mu;

    if (p<PAR_MIN_AMB_DIM) return 0;
    if (p>65) p=65;

    x=1.0-ps;
    if (x<PAR_FFRT_XMIN) x=PAR_FFRT_XMIN;
    if (x>=PAR_FFRT_XMAX) return 0;

    if (x<0.01) mu=1.0;
    else {
        c=par_ffrt_coeffs[p-4];
        mu=c[0]*pow(x,c[1])+c[2];
    }
    if (mu<=0.0) return 0;
    if (mu>1.0) mu=1.0;

    *thres=MAX(1.0/mu,PAR_FFRT_CMIN);
    return 1;
}

/* build full DD float ambiguities and covariance matrices -------------------*/
static void par_build_full_dd_mats(const rtk_t *rtk, const int *ix, int nb,
                                   double *y, double *DP, double *Qb,
                                   double *Qab)
{
    int i,j,nx=rtk->nx,na=rtk->na;

    for (i=0;i<nb;i++) {
        y[i]=rtk->x[ix[i*2]]-rtk->x[ix[i*2+1]];
    }
    for (j=0;j<nx-na;j++) for (i=0;i<nb;i++) {
        DP[i+j*nb]=rtk->P[ix[i*2]+(na+j)*nx]-
                   rtk->P[ix[i*2+1]+(na+j)*nx];
    }
    for (j=0;j<nb;j++) for (i=0;i<nb;i++) {
        Qb[i+j*nb]=DP[i+(ix[j*2]-na)*nb]-
                   DP[i+(ix[j*2+1]-na)*nb];
    }
    for (j=0;j<nb;j++) for (i=0;i<na;i++) {
        Qab[i+j*na]=rtk->P[i+ix[j*2]*nx]-
                    rtk->P[i+ix[j*2+1]*nx];
    }
}

/* copy a tail subset in the reduced z-space --------------------------------*/
static void par_build_z_subset(int n, int start, const double *z,
                               const double *Qzz, const double *L,
                               const double *D, const double *Z,
                               double *zp, double *Qp, double *Lp,
                               double *Dp, double *Zp)
{
    int i,j,p=n-start;

    for (i=0;i<p;i++) {
        zp[i]=z[start+i];
        Dp[i]=D[start+i];
        for (j=0;j<p;j++) {
            Qp[i+j*p]=Qzz[start+i+(start+j)*n];
            Lp[i+j*p]=L[start+i+(start+j)*n];
        }
    }
    for (j=0;j<p;j++) for (i=0;i<n;i++) {
        Zp[i+j*n]=Z[i+(start+j)*n];
    }
}

/* trace of fixed position covariance for a selected z-space ambiguity set ---*/
static int par_fixed_pos_trace_z(const rtk_t *rtk, const double *Qbz,
                                 const double *Qzz, int p, double *tr)
{
    int i,j,nx=rtk->nx,na=rtk->na;
    double *Qzi,*QQ,*Pa;

    if (na<3||p<=0) return 0;

    Qzi=mat(p,p); QQ=mat(na,p); Pa=mat(na,na);
    matcpy(Qzi,Qzz,p,p);
    for (j=0;j<na;j++) for (i=0;i<na;i++) {
        Pa[i+j*na]=rtk->P[i+j*nx];
    }
    if (matinv(Qzi,p)) {
        free(Qzi); free(QQ); free(Pa);
        return 0;
    }
    matmul("NN",na,p,p,Qbz,Qzi,QQ);
    matmulm("NT",na,na,p,QQ,Qbz,Pa);
    *tr=Pa[0]+Pa[1+na]+Pa[2+2*na];

    free(Qzi); free(QQ); free(Pa);
    return *tr>0.0;
}

/* baseline precision defect check for a selected z-space subset ------------*/
static int par_bpd_z_ok(const rtk_t *rtk, const double *Qbz,
                        const double *Qzz, int p, double tr_full,
                        double *bpd)
{
    double tr_float,tr_part;
    int nx=rtk->nx;

    if (tr_full<=0.0) return 0;
    tr_float=rtk->P[0]+rtk->P[1+nx]+rtk->P[2+2*nx];
    if (tr_float<=0.0) return 0;
    if (!par_fixed_pos_trace_z(rtk,Qbz,Qzz,p,&tr_part)) return 0;
    *bpd=sqrt(tr_float/tr_full)-sqrt(tr_float/tr_part);
    return *bpd<=PAR_BPD_MAX;
}

/* update real-valued states using a fixed reduced z-space subset ------------*/
static int par_update_real_z(rtk_t *rtk, const double *zp,
                             const double *zfix, const double *Qp,
                             const double *Qbz, int p)
{
    int i,j,nx=rtk->nx,na=rtk->na;
    double *Qpi,*dz,*db,*QQ;

    Qpi=mat(p,p); dz=mat(p,1); db=mat(p,1); QQ=mat(na,p);

    for (i=0;i<na;i++) {
        rtk->xa[i]=rtk->x[i];
        for (j=0;j<na;j++) rtk->Pa[i+j*na]=rtk->P[i+j*nx];
    }
    for (i=0;i<p;i++) dz[i]=zp[i]-zfix[i];

    matcpy(Qpi,Qp,p,p);
    if (matinv(Qpi,p)) {
        free(Qpi); free(dz); free(db); free(QQ);
        return 0;
    }
    matmul("NN",p,1,p,Qpi,dz,db);
    matmulm("NN",na,1,p,Qbz,db,rtk->xa);
    matmul("NN",na,p,p,Qbz,Qpi,QQ);
    matmulm("NT",na,na,p,QQ,Qbz,rtk->Pa);

    free(Qpi); free(dz); free(db); free(QQ);
    return 1;
}

/* restore complete state vector for residual validation after z-space PAR ---*/
static int par_restamb_z(rtk_t *rtk, const int *ix, int n, const double *Z,
                         const double *z, const double *Qzz, int start,
                         const double *zfix, int p, double *xa)
{
    double *zmix,*dz,*Qpi,*work,*amix;
    int i,j,na=rtk->na;

    zmix=mat(n,1); amix=mat(n,1);
    for (i=0;i<n;i++) zmix[i]=z[i];

    if (start>0) {
        dz=mat(p,1); Qpi=mat(p,p); work=mat(p,1);
        for (i=0;i<p;i++) {
            dz[i]=z[start+i]-zfix[i];
            for (j=0;j<p;j++) {
                Qpi[i+j*p]=Qzz[start+i+(start+j)*n];
            }
        }
        if (matinv(Qpi,p)) {
            free(zmix); free(amix); free(dz); free(Qpi); free(work);
            return 0;
        }
        matmul("NN",p,1,p,Qpi,dz,work);
        for (i=0;i<start;i++) {
            double corr=0.0;
            for (j=0;j<p;j++) corr+=Qzz[i+(start+j)*n]*work[j];
            zmix[i]=z[i]-corr;
        }
        free(dz); free(Qpi); free(work);
    }
    for (i=0;i<p;i++) zmix[start+i]=zfix[i];

    if (solve("T",Z,zmix,n,1,amix)) {
        free(zmix); free(amix);
        return 0;
    }

    for (i=0;i<rtk->nx;i++) xa[i]=rtk->x[i];
    for (i=0;i<na;i++) xa[i]=rtk->xa[i];

    for (i=0;i<n;i++) {
        xa[ix[i*2]]=rtk->x[ix[i*2]];
    }
    for (i=0;i<n;i++) {
        xa[ix[i*2+1]]=xa[ix[i*2]]-amix[i];
    }

    free(zmix); free(amix);
    return 1;
}

/* try to fix a tail subset in the reduced z-space --------------------------*/
static int par_try_z_subset(rtk_t *rtk, const int *ix, int n, int start,
                            const double *Z, const double *z,
                            const double *Qzz, const double *L,
                            const double *D, const double *Qab,
                            double tr_full, double *xa)
{
    int p=n-start,info,na=rtk->na;
    double *zp,*Qp,*Lp,*Dp,*Zp,*Qbz,*zfix,s[2],ps,ratio,thres,bpd;

    zp=mat(p,1); Qp=mat(p,p); Lp=mat(p,p); Dp=mat(p,1);
    Zp=mat(n,p); Qbz=mat(na,p); zfix=mat(p,2);

    par_build_z_subset(n,start,z,Qzz,L,D,Z,zp,Qp,Lp,Dp,Zp);
    ps=par_bootstrap_success(p,Dp);
    if (ps<PAR_SUCCESS_RATE_THRES) {
        trace(3,"PAR SRC failed (start=%d p=%d ps=%.6f thres=%.6f)\n",
              start,p,ps,PAR_SUCCESS_RATE_THRES);
        free(zp); free(Qp); free(Lp); free(Dp); free(Zp); free(Qbz); free(zfix);
        return 0;
    }
    if (!par_bffrt_threshold(p,ps,&thres)) {
        trace(3,"PAR FFRT threshold failed (start=%d p=%d ps=%.6f)\n",
              start,p,ps);
        free(zp); free(Qp); free(Lp); free(Dp); free(Zp); free(Qbz); free(zfix);
        return 0;
    }
    rtk->sol.thres=(float)thres;

    if ((info=lambda_search_LD(p,2,zp,Lp,Dp,zfix,s))) {
        errmsg(rtk,"PAR z-space lambda error (info=%d start=%d p=%d)\n",
               info,start,p);
        free(zp); free(Qp); free(Lp); free(Dp); free(Zp); free(Qbz); free(zfix);
        return 0;
    }
    ratio=s[0]>PAR_RATIO_EPS?s[1]/s[0]:999.9;
    rtk->sol.ratio=(float)MIN(ratio,999.9);

    trace(3,"PAR z(1)=     "); tracemat(3,zfix  ,1,p,7,2);
    trace(3,"PAR z(2)=     "); tracemat(3,zfix+p,1,p,7,2);

    if (ratio<thres) {
        errmsg(rtk,"PAR FFRT failed (start=%d p=%d ratio=%.2f thresh=%.2f ps=%.6f s=%.2f/%.2f)\n",
               start,p,ratio,thres,ps,s[0],s[1]);
        free(zp); free(Qp); free(Lp); free(Dp); free(Zp); free(Qbz); free(zfix);
        return 0;
    }

    matmul("NN",na,p,n,Qab,Zp,Qbz);
    if (!par_bpd_z_ok(rtk,Qbz,Qp,p,tr_full,&bpd)) {
        errmsg(rtk,"PAR BPD failed (start=%d p=%d ratio=%.2f thresh=%.2f ps=%.6f)\n",
               start,p,ratio,thres,ps);
        free(zp); free(Qp); free(Lp); free(Dp); free(Zp); free(Qbz); free(zfix);
        return -1; /* BPD failure means smaller subsets are not useful here */
    }

    if (!par_update_real_z(rtk,zp,zfix,Qp,Qbz,p)||
        !par_restamb_z(rtk,ix,n,Z,z,Qzz,start,zfix,p,xa)) {
        free(zp); free(Qp); free(Lp); free(Dp); free(Zp); free(Qbz); free(zfix);
        return -1;
    }

    trace(3,"PAR validation ok (nb=%d ratio=%.2f thresh=%.2f ps=%.6f bpd=%.2f s=%.2f/%.2f)\n",
          p,ratio,thres,ps,bpd,s[0],s[1]);

    free(zp); free(Qp); free(Lp); free(Dp); free(Zp); free(Qbz); free(zfix);
    return 1;
}

/* resolve integer ambiguity by PAR with SRC, B-FFRT and BPD -----------------*/
static int resamb_PAR(rtk_t *rtk, double *bias, double *xa,
                      int gps, int glo, int sbs)
{
    int nb,nb0,start,minamb,nx=rtk->nx,na=rtk->na,*ix,stat;
    double *DP,*a,*Qaa,*Qab,*Z,*z,*Qzz,*L,*D,*Qbz,tr_full=0.0;

    trace(3,"resamb_PAR : nx=%d\n",nx);

    (void)bias;
    rtk->sol.ratio=0.0;
    rtk->sol.thres=(float)PAR_FFRT_CMIN;
    rtk->nb_ar=0;

    ix=imat(nx,2);
    if ((nb0=ddidx(rtk,ix,gps,glo,sbs))<(rtk->opt.minfixsats-1)) {
        errmsg(rtk,"not enough valid double-differences for PAR\n");
        free(ix);
        return -1;
    }
    rtk->nb_ar=nb0;
    /* In PAR, ssat[].fix==2 means the ambiguity passed the original AR
       screening and participates in z-space PAR. It does not mean the
       corresponding satellite/frequency ambiguity is individually fixed. */
    minamb=MAX(PAR_MIN_AMB_DIM,rtk->opt.minfixsats-1);
    if (nb0<minamb) {
        errmsg(rtk,"not enough PAR ambiguities (nb=%d min=%d)\n",nb0,minamb);
        free(ix);
        return -1;
    }

    DP=mat(nb0,nx-na); a=mat(nb0,1); Qaa=mat(nb0,nb0);
    Qab=mat(na,nb0); Z=mat(nb0,nb0); z=mat(nb0,1);
    Qzz=mat(nb0,nb0); L=mat(nb0,nb0); D=mat(nb0,1);
    Qbz=mat(na,nb0);

    par_build_full_dd_mats(rtk,ix,nb0,a,DP,Qaa,Qab);
    if (lambda_reduction_info(nb0,a,Qaa,Z,z,Qzz,L,D)) {
        errmsg(rtk,"PAR lambda reduction failed (nb=%d)\n",nb0);
        free(ix); free(DP); free(a); free(Qaa); free(Qab);
        free(Z); free(z); free(Qzz); free(L); free(D); free(Qbz);
        return 0;
    }
    matmul("NN",na,nb0,nb0,Qab,Z,Qbz);
    if (!par_fixed_pos_trace_z(rtk,Qbz,Qzz,nb0,&tr_full)) {
        errmsg(rtk,"PAR full fixed covariance failed (nb=%d)\n",nb0);
    }

#ifdef TRACE
    {
        int i;
        double *QQb=mat(nb0,1);
        for (i=0;i<nb0;i++) QQb[i]=1000*Qaa[i+i*nb0];
        trace(3,"PAR a(0)=     "); tracemat(3,a,1,nb0,7,2);
        trace(3,"PAR z(0)=     "); tracemat(3,z,1,nb0,7,2);
        trace(3,"PAR Qb*1000=  "); tracemat(3,QQb,1,nb0,7,4);
        trace(3,"PAR D=        "); tracemat(3,D,1,nb0,10,4);
        free(QQb);
    }
#endif

    nb=0;
    /* TC-PAR subset selection is performed in the reduced z-space. LAMBDA
       reduction orders D so the low-precision ambiguities are at the front;
       each failed attempt removes the next leading z ambiguity and keeps the
       tail subset z[start:n-1]. */
    for (start=0;start<=nb0-minamb;start++) {
        stat=par_try_z_subset(rtk,ix,nb0,start,Z,z,Qzz,L,D,Qab,tr_full,xa);
        if (stat>0) {
            nb=nb0-start;
            rtk->nb_ar=nb;
            break;
        }
        if (stat<0) break;
        trace(3,"PAR drop reduced ambiguity z-index=%d\n",start);
    }
    if (nb<=0) {
        errmsg(rtk,"PAR ambiguity validation failed (nb=%d ratio=%.2f thresh=%.2f)\n",
               nb0,rtk->sol.ratio,rtk->sol.thres);
    }

    free(ix); free(DP); free(a); free(Qaa); free(Qab);
    free(Z); free(z); free(Qzz); free(L); free(D); free(Qbz);
    return nb;
}

/* resolve integer ambiguity by PAR using same management policy as LAMBDA ----*/
static int manage_amb_PAR(rtk_t *rtk, double *bias, double *xa,
                          const int *sat, int nf, int ns)
{
    int gps1=-1,glo1=-1,sbas1=-1,nb,rerun,dly;
    float ratio1,posvar=0;

    for (int i=0;i<3;i++) posvar+=rtk->P[i+i*rtk->nx];
    posvar/=3.0;

    trace(3,"PAR posvar=%.6f\n",posvar);
    trace(3,"PAR prevRatios= %.3f %.3f\n",
          rtk->sol.prev_ratio1,rtk->sol.prev_ratio2);
    trace(3,"PAR num ambiguities used last AR: %d\n",rtk->nb_ar);

    if (rtk->opt.mode<=PMODE_DGPS||rtk->opt.modear==ARMODE_OFF||
        rtk->opt.thresar[0]<1.0||posvar>rtk->opt.thresar[1]) {
        trace(3,"Skip PAR AR\n");
        rtk->sol.ratio=0.0;
        rtk->sol.prev_ratio1=rtk->sol.prev_ratio2=0.0;
        rtk->nb_ar=0;
        return 0;
    }

    int lockc[NFREQ],excsat=0;
    if (rtk->sol.prev_ratio2<rtk->sol.thres&&rtk->nb_ar>=rtk->opt.mindropsats) {
        int i=0;
        if (rtk->excsat!=0) {
            for (;i<ns;i++) {
                if (rtk->excsat==sat[i]) {
                    i++;
                    break;
                }
            }
            if (i>=ns) i=0;
        }
        for (;i<ns;i++) {
            for (int f=0;f<nf;f++) {
                if (rtk->ssat[sat[i]-1].vsat[f]&&
                    rtk->ssat[sat[i]-1].lock[f]>=0&&
                    rtk->ssat[sat[i]-1].azel[1]>=rtk->opt.elmin) {
                    excsat=sat[i];
                    break;
                }
            }
            if (excsat) break;
        }
        if (excsat) {
            for (int f=0;f<nf;f++) {
                lockc[f]=rtk->ssat[excsat-1].lock[f];
                rtk->ssat[excsat-1].lock[f]=-rtk->nb_ar;
            }
            trace(3,"PAR AR: exclude sat %d\n",excsat);
        }
        rtk->excsat=excsat;
    }

    gps1=1;
    /* PAR fix-and-hold is not implemented yet; treat GLO fix-and-hold as
       continuous AR here instead of waiting for a hold event that will not
       be generated in PAR mode. */
    glo1=(rtk->opt.navsys&SYS_GLO)?1:0;
    sbas1=(rtk->opt.navsys&SYS_GLO)?glo1:((rtk->opt.navsys&SYS_SBS)?1:0);

    nb=resamb_PAR(rtk,bias,xa,gps1,glo1,sbas1);
    ratio1=rtk->sol.ratio;

    if (rtk->opt.arfilter) {
        rerun=0;
        if (nb>=0&&rtk->sol.prev_ratio2>=rtk->sol.thres&&
            ((rtk->sol.ratio<rtk->sol.thres)||
             (rtk->sol.ratio<rtk->opt.thresar[0]*1.1&&
              rtk->sol.ratio<rtk->sol.prev_ratio1/2.0))) {
            trace(3,"PAR low ratio: check for new sat\n");
            dly=2;
            for (int i=0;i<ns;i++) for (int f=0;f<nf;f++) {
                if (rtk->ssat[sat[i]-1].fix[f]!=2) continue;
                if (rtk->ssat[sat[i]-1].lock[f]==0) {
                    trace(3,"PAR remove sat %d:%d lock=%d\n",
                          sat[i],f,rtk->ssat[sat[i]-1].lock[f]);
                    rtk->ssat[sat[i]-1].lock[f]=-rtk->opt.minlock-dly;
                    dly+=2;
                    rerun=1;
                }
            }
        }
        if (rerun) {
            trace(3,"rerun PAR AR with new sats removed\n");
            nb=resamb_PAR(rtk,bias,xa,gps1,glo1,sbas1);
        }
    }
    rtk->sol.prev_ratio1=ratio1;
    if (excsat&&(rtk->sol.ratio<rtk->sol.thres)&&
        (rtk->sol.ratio<(1.5*rtk->sol.prev_ratio2))) {
        for (int f=0;f<nf;f++) rtk->ssat[excsat-1].lock[f]=lockc[f];
        trace(3,"PAR AR: restore sat %d\n",excsat);
    }

    rtk->sol.prev_ratio1=ratio1>0?ratio1:rtk->sol.ratio;
    rtk->sol.prev_ratio2=rtk->sol.ratio;

    return nb;
}

#endif /* RTKPOS_INCLUDE_RTKALG */

/* select reference satellites by highest elevation --------------------------*/
static void select_sat_elmax(rtk_t *rtk, const int *sat, double *y,
                             double *azel, const int *iu, const int *ir,
                             int ns, int nf, int refsat[6][NFREQ*2])
{
    prcopt_t *opt=&rtk->opt;
    int i,j,m,f,frq,sysi;

    for (m=0;m<6;m++) for (f=0;f<NFREQ*2;f++) refsat[m][f]=0;

    /* step through sat systems: m=0:gps/sbs,1:glo,2:gal,3:bds 4:qzs 5:irn*/
    for (m=0;m<6;m++) {

        /* step through phases/codes */
        for (f=opt->mode>PMODE_DGPS?0:nf;f<nf*2;f++) {
            frq=f%nf;

            /* find reference satellite with highest elevation */
            for (i=-1,j=0;j<ns;j++) {
                sysi=rtk->ssat[sat[j]-1].sys;
                if (!test_sys(sysi,m) || sysi==SYS_SBS) continue;
                if (!validobs(iu[j],ir[j],f,nf,y)) continue;
                /* skip sat with slip unless no other valid sat */
                if (i>=0&&rtk->ssat[sat[j]-1].slip[frq]&LLI_SLIP) continue;
                if (i<0||azel[1+iu[j]*2]>=azel[1+iu[i]*2]) i=j;
            }
            if (i>=0) {
                refsat[m][f]=sat[i];
                trace(3,"refsat elmax: sys=%d frq=%d type=%s sat=%d el=%.1f\n",
                      m,frq+1,f<nf?"L":"P",sat[i],azel[1+iu[i]*2]*R2D);
            }
        }
    }
}
/* select reference satellites by double-difference geometry -----------------*/
static void select_sat_geom(rtk_t *rtk, const obsd_t *obs, double dt,
                            const int *sat, double *y, double *e,
                            double *azel, double *freq, const int *iu,
                            const int *ir, int ns, int nf,
                            const double *x, int refsat[6][NFREQ*2])
{
    prcopt_t *opt=&rtk->opt;
    double dr[3],bl=baseline(x,rtk->rb,dr);
    int m,frq,i,j,k,q,n,sys,prev,badprev,f2=0;

    for (m=0;m<6;m++) for (frq=0;frq<NFREQ*2;frq++) refsat[m][frq]=0;
    for (m=0;m<6;m++) for (frq=0;frq<NFREQ;frq++) {
        rtk->refsat_used[m][frq]=rtk->refsat_rej[m][frq]=0;
    }

    for (m=0;m<6;m++) for (frq=0;frq<nf;frq++) {
        int sats[MAXSAT],cands[MAXSAT],nsat=0,ncand=0;
        int best=-1,best_degraded=0,prev_ok=0,kept_prev=0,best_ndd=0;
        double best_score=1E99,best_kappa=1E99,prev_score=1E99,prev_kappa=1E99;
        double cand_var[MAXSAT],cand_kappa[MAXSAT],cand_score[MAXSAT];
        double var_sort[MAXSAT],median=0.0;
        int cand_ok[MAXSAT];

        prev=rtk->refsat[m][frq];
        badprev=prev&&rtk->refsat_bad[m][frq];
        if (badprev) {
            trace(2,"refsat geom: previous reference marked bad sys=%d frq=%d sat=%d\n",
                  m,frq+1,prev);
        }

        for (i=0;i<ns;i++) {
            int ok=1,candidate=1;

            sys=rtk->ssat[sat[i]-1].sys;
            if (!test_sys(sys,m)||sys==SYS_SBS) continue;
            if (azel[1+iu[i]*2]<opt->elmin) continue;
            if (!validobs(iu[i],ir[i],frq,nf,y)||
                !validobs(iu[i],ir[i],frq+nf,nf,y)) continue;
            if (freq[frq+iu[i]*nf]<=0.0) continue;

            if (opt->ionoopt==IONOOPT_IFLC) {
                f2=seliflc(opt->nf,sys);
                if (f2>=opt->nf||f2>=NFREQ) ok=0;
                else if (obs[iu[i]].L[0]==0.0||obs[ir[i]].L[0]==0.0||
                         obs[iu[i]].L[f2]==0.0||obs[ir[i]].L[f2]==0.0||
                         obs[iu[i]].P[0]==0.0||obs[ir[i]].P[0]==0.0||
                         obs[iu[i]].P[f2]==0.0||obs[ir[i]].P[f2]==0.0) ok=0;
            }
            if (!ok) continue;

            sats[nsat++]=i;

            if (azel[1+iu[i]*2]<MAX(opt->elmin,REFSEL_ELMIN)) candidate=0;
            if (rtk->ssat[sat[i]-1].snr_rover[frq]<REFSEL_SNRMIN||
                rtk->ssat[sat[i]-1].snr_base [frq]<REFSEL_SNRMIN) candidate=0;
            if (rtk->ssat[sat[i]-1].slip[frq]&LLI_SLIP) candidate=0;
            if (!rtk->ssat[sat[i]-1].half[frq]) candidate=0;
            if ((obs[iu[i]].LLI[frq]&LLI_HALFC)||
                (obs[ir[i]].LLI[frq]&LLI_HALFC)) candidate=0;
            if (opt->ionoopt==IONOOPT_IFLC) {
                f2=seliflc(opt->nf,sys);
                if (obs[iu[i]].SNR[f2]<REFSEL_SNRMIN||
                    obs[ir[i]].SNR[f2]<REFSEL_SNRMIN) candidate=0;
                if (rtk->ssat[sat[i]-1].slip[f2]&LLI_SLIP) candidate=0;
                if (!rtk->ssat[sat[i]-1].half[f2]) candidate=0;
                if ((obs[iu[i]].LLI[f2]&LLI_HALFC)||
                    (obs[ir[i]].LLI[f2]&LLI_HALFC)) candidate=0;
            }
            if (badprev&&sat[i]==prev) candidate=0;

            if (candidate) {
                cands[ncand]=i;
                cand_var[ncand++]=varerr(sat[i],sys,azel[1+iu[i]*2],
                    rtk->ssat[sat[i]-1].snr_rover[frq],
                    rtk->ssat[sat[i]-1].snr_base [frq],
                    bl,dt,frq,opt,&obs[iu[i]]);
            }
        }
        if (nsat<=0) continue;

        if (ncand<=0) { /* degrade to highest elevation in the available set */
            for (best=sats[0],i=1;i<nsat;i++) {
                if (azel[1+iu[sats[i]]*2]>=azel[1+iu[best]*2]) best=sats[i];
            }
            best_ndd=nsat-1;
            best_degraded=1;
            trace(2,"refsat geom: no candidate sys=%d frq=%d nsat=%d, use elmax sat=%d\n",
                  m,frq+1,nsat,sat[best]);
        }
        else {
            for (i=0;i<ncand;i++) {
                cand_kappa[i]=cand_score[i]=1E99;
                cand_ok[i]=0;
            }
            for (i=0;i<ncand;i++) var_sort[i]=cand_var[i];
            for (i=0;i<ncand-1;i++) for (j=i+1;j<ncand;j++) {
                if (var_sort[j]<var_sort[i]) {
                    double tmp=var_sort[i]; var_sort[i]=var_sort[j]; var_sort[j]=tmp;
                }
            }
            median=ncand%2?var_sort[ncand/2]:
                   (var_sort[ncand/2-1]+var_sort[ncand/2])*0.5;

            for (int ci=0;ci<ncand;ci++) {
                double *B,*Rg,*BR,*N,varp,score,kappa=1E99;
                double eval[3]={0},A[9],off,app,aqq,apq,phi,c,s,tmp;
                int ndd=nsat-1,row,col,info=0,ref=cands[ci],ip,iq,ii,jj;

                if (ndd<3) continue;
                B=mat(3,ndd); Rg=zeros(ndd,ndd);
                BR=mat(3,ndd); N=zeros(3,3);

                varp=varerr(sat[ref],rtk->ssat[sat[ref]-1].sys,
                    azel[1+iu[ref]*2],
                    rtk->ssat[sat[ref]-1].snr_rover[frq],
                    rtk->ssat[sat[ref]-1].snr_base [frq],
                    bl,dt,frq,opt,&obs[iu[ref]]);

                for (row=0,n=0;n<nsat;n++) {
                    double varq;
                    q=sats[n];
                    if (q==ref) continue;

                    for (k=0;k<3;k++) B[k+row*3]=-e[k+iu[ref]*3]+e[k+iu[q]*3];
                    varq=varerr(sat[q],rtk->ssat[sat[q]-1].sys,
                        azel[1+iu[q]*2],
                        rtk->ssat[sat[q]-1].snr_rover[frq],
                        rtk->ssat[sat[q]-1].snr_base [frq],
                        bl,dt,frq,opt,&obs[iu[q]]);
                    for (col=0;col<ndd;col++) Rg[row+col*ndd]=varp;
                    Rg[row+row*ndd]+=varq;
                    row++;
                }
                if (matinv(Rg,ndd)) info=1;
                else {
                    matmul("NN",3,ndd,ndd,B,Rg,BR);
                    matmul("NT",3,3,ndd,BR,B,N);
                }
                if (!info) {
                    for (ii=0;ii<9;ii++) A[ii]=N[ii];
                    for (ii=0;ii<24;ii++) {
                        ip=0; iq=1; off=fabs(A[0+1*3]);
                        if (fabs(A[0+2*3])>off) {ip=0; iq=2; off=fabs(A[0+2*3]);}
                        if (fabs(A[1+2*3])>off) {ip=1; iq=2; off=fabs(A[1+2*3]);}
                        if (off<REFSEL_EPS) break;
                        app=A[ip+ip*3]; aqq=A[iq+iq*3]; apq=A[ip+iq*3];
                        phi=0.5*atan2(2.0*apq,aqq-app);
                        c=cos(phi); s=sin(phi);
                        for (k=0;k<3;k++) {
                            tmp=A[k+ip*3];
                            A[k+ip*3]=c*tmp-s*A[k+iq*3];
                            A[k+iq*3]=s*tmp+c*A[k+iq*3];
                        }
                        for (k=0;k<3;k++) {
                            tmp=A[ip+k*3];
                            A[ip+k*3]=c*tmp-s*A[iq+k*3];
                            A[iq+k*3]=s*tmp+c*A[iq+k*3];
                        }
                    }
                    eval[0]=A[0]; eval[1]=A[4]; eval[2]=A[8];
                    for (ii=0;ii<2;ii++) for (jj=ii+1;jj<3;jj++) {
                        if (eval[jj]<eval[ii]) {
                            tmp=eval[ii]; eval[ii]=eval[jj]; eval[jj]=tmp;
                        }
                    }
                    if (eval[0]>REFSEL_EPS) kappa=eval[2]/eval[0];
                    else info=1;
                }
                if (!info) {
                    score=log(kappa)+REFSEL_ALPHA*varp/(median+REFSEL_EPS);
                    cand_ok[ci]=1;
                    cand_kappa[ci]=kappa;
                    cand_score[ci]=score;
                    if (sat[ref]==prev) {
                        prev_ok=1; prev_score=score; prev_kappa=kappa;
                    }
                    if (score<best_score) {
                        best_score=score; best_kappa=kappa; best=ref; best_ndd=ndd;
                    }
                }
                if (!info) {
                    trace(5,"refsat geom detail: sys=%d frq=%d sat=%d ndd=%d var=%.3e kappa=%.3e score=%.3f\n",
                          m,frq+1,sat[ref],ndd,varp,kappa,score);
                    trace(5,"refsat geom N:\n");
                    tracemat(5,N,3,3,13,6);
                    trace(5,"refsat geom eval=%.6e %.6e %.6e\n",
                          eval[0],eval[1],eval[2]);
                }
                else {
                    trace(5,"refsat geom detail: sys=%d frq=%d sat=%d invalid geometry\n",
                          m,frq+1,sat[ref]);
                }
                free(B); free(Rg); free(BR); free(N);
            }
            for (i=0;i<ncand&&i<5;i++) {
                int isat=cands[i];
                trace(4,"refsat geom cand: sys=%d frq=%d sat=%d el=%.1f snr=%.1f var=%.3e kappa=%.3e score=%.3f%s\n",
                      m,frq+1,sat[isat],azel[1+iu[isat]*2]*R2D,
                      MIN(rtk->ssat[sat[isat]-1].snr_rover[frq],
                          rtk->ssat[sat[isat]-1].snr_base [frq]),
                      cand_var[i],cand_kappa[i],cand_score[i],
                      cand_ok[i]?"":" invalid");
            }
            if (best<0) {
                for (best=sats[0],i=1;i<nsat;i++) {
                    if (azel[1+iu[sats[i]]*2]>=azel[1+iu[best]*2]) best=sats[i];
                }
                best_ndd=nsat-1;
                best_degraded=1;
                trace(2,"refsat geom: all geometry candidates invalid sys=%d frq=%d ncand=%d, use elmax sat=%d\n",
                      m,frq+1,ncand,sat[best]);
            }
            else if (prev_ok&&prev!=sat[best]&&
                     (prev_kappa/best_kappa<=1.0+REFSEL_ETA||
                      prev_score-best_score<REFSEL_DJ)) {
                for (i=0;i<ns;i++) if (sat[i]==prev) {best=i; break;}
                best_score=prev_score;
                best_kappa=prev_kappa;
                best_ndd=nsat-1;
                kept_prev=1;
            }
        }
        rtk->refsat[m][frq]=sat[best];
        rtk->refsat_bad[m][frq]=0;
        refsat[m][frq]=refsat[m][frq+nf]=sat[best];
        if (best_degraded) {
            trace(3,"refsat geom: sys=%d frq=%d sat=%d ndd=%d degraded\n",m,frq+1,sat[best],best_ndd);
        }
        else {
            trace(3,"refsat geom: sys=%d frq=%d sat=%d ndd=%d kappa=%.3e score=%.3f%s\n",
                  m,frq+1,sat[best],best_ndd,best_kappa,best_score,kept_prev?" kept":"");
        }
    }
}
