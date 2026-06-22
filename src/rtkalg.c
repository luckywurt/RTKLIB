/* rtk algorithm helpers -----------------------------------------------------*/
#include "rtklib.h"

#define REFSEL_ELMAX 0       /* reference satellite selection: elevation */
#define REFSEL_GEOM  1       /* reference satellite selection: geometry */
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

#ifdef RTKPOS_INCLUDE_RTKALG

#define PAR_TDCP_MAX_AGE   5.0   /* max age of previous SD cache (s) */
#define PAR_RATIO_IMPROVE  1.10  /* required relative ratio/thres improvement */
#define PAR_MIN_TOTAL_DD   12    /* min total DD count before PAR attempts */
#define PAR_MIN_SYS_DD     3     /* min DD count per system after exclusion */
#define PAR_MAX_DROP_FRAC  0.50  /* max excluded DD fraction */
#define PAR_FINAL_DROP_RATE 0.20 /* final carry-forward exclusions per system */

typedef struct {        /* PAR sat/frequency exclusion candidate */
    int sat;
    int freq;
    int sys;
    int m;
    int refsat;
    int prior;
    int has_tdcp;
    double tdcp;
    double ratio;
    double thres;
    double norm;
    double improve;
} par_cand_t;

typedef struct {        /* PAR trial state snapshot */
    uint8_t fix[MAXSAT][NFREQ];
    float ratio;
    float thres;
    int nb_ar;
} par_snapshot_t;

static int par_sys_index(int sys)
{
    switch (sys) {
        case SYS_GPS: return 0;
        case SYS_GLO: return 1;
        case SYS_GAL: return 2;
        case SYS_CMP: return 3;
        case SYS_QZS: return 4;
        case SYS_IRN: return 5;
    }
    return -1;
}

static double par_ratio_norm(double ratio, double thres)
{
    return thres>0.0?ratio/thres:ratio;
}

static int par_ratio_improved(double norm0, double norm1)
{
    if (norm1<=norm0) return 0;
    return norm0<=0.0||norm1/norm0>=PAR_RATIO_IMPROVE;
}

static void par_clear_excl(uint8_t excl[MAXSAT][NFREQ])
{
    int i,j;

    for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ;j++) excl[i][j]=0;
}

static void par_clear_prev_excl(rtk_t *rtk)
{
    int i,j;

    for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ;j++) {
        rtk->par_excl_prev[i][j]=0;
    }
}

static void par_store_prev_excl(rtk_t *rtk, const uint8_t excl[MAXSAT][NFREQ])
{
    int i,j;

    for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ;j++) {
        rtk->par_excl_prev[i][j]=excl[i][j];
    }
}

static void par_save_snapshot(rtk_t *rtk, par_snapshot_t *snap)
{
    int i,j;

    for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ;j++) {
        snap->fix[i][j]=rtk->ssat[i].fix[j];
    }
    snap->ratio=rtk->sol.ratio;
    snap->thres=rtk->sol.thres;
    snap->nb_ar=rtk->nb_ar;
}

static void par_restore_snapshot(rtk_t *rtk, const par_snapshot_t *snap)
{
    int i,j;

    for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ;j++) {
        rtk->ssat[i].fix[j]=snap->fix[i][j];
    }
    rtk->sol.ratio=snap->ratio;
    rtk->sol.thres=snap->thres;
    rtk->nb_ar=snap->nb_ar;
}

static int par_is_current_refsat(const int refsat[6][NFREQ*2], int sat, int f)
{
    int m;

    for (m=0;m<6;m++) {
        if (refsat[m][f]==sat) return 1;
    }
    return 0;
}

/* index for PAR single to double-difference transformation matrix (D') ------*/
static int ddidx_PAR(rtk_t *rtk, int *ix, int gps, int glo, int sbs,
                     const uint8_t par_excl[MAXSAT][NFREQ])
{
    int i,j,k,m,f,n,nb=0,na=rtk->na,nf=NF(&rtk->opt),nofix;
    double fix[MAXSAT],ref[MAXSAT];

    trace(3,"ddidx_PAR: gps=%d/%d glo=%d/%d sbs=%d\n",
          gps,rtk->opt.gpsmodear,glo,rtk->opt.glomodear,sbs);

    for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ;j++) {
        rtk->ssat[i].fix[j]=0;
    }
    for (m=0;m<6;m++) {
        nofix=(m==0&&gps==0)||(m==1&&glo==0)||(m==3&&rtk->opt.bdsmodear==0);

        for (f=0,k=na;f<nf;f++,k+=MAXSAT) {
            for (i=k;i<k+MAXSAT;i++) {
                int sat_i=i-k+1;

                if (par_excl[sat_i-1][f]) {
                    rtk->ssat[sat_i-1].fix[f]=1;
                    continue;
                }
                if (sbs==0&&satsys(sat_i,NULL)==SYS_SBS) continue;
                if (rtk->x[i]==0.0||!test_sys(rtk->ssat[sat_i-1].sys,m)||
                    !rtk->ssat[sat_i-1].vsat[f]) {
                    continue;
                }
                if (rtk->ssat[sat_i-1].lock[f]>=0&&
                    !(rtk->ssat[sat_i-1].slip[f]&(LLI_SLIP|LLI_HALFC))&&
                    rtk->ssat[sat_i-1].azel[1]>=rtk->opt.elmaskar&&!nofix) {
                    rtk->ssat[sat_i-1].fix[f]=2;
                    break;
                }
                else rtk->ssat[sat_i-1].fix[f]=1;
            }
            if (i>=k+MAXSAT||rtk->ssat[i-k].fix[f]!=2) continue;

            for (n=0,j=k;j<k+MAXSAT;j++) {
                int sat_j=j-k+1;

                if (par_excl[sat_j-1][f]) {
                    rtk->ssat[sat_j-1].fix[f]=1;
                    continue;
                }
                if (i==j||rtk->x[j]==0.0||!test_sys(rtk->ssat[sat_j-1].sys,m)||
                    !rtk->ssat[sat_j-1].vsat[f]) {
                    continue;
                }
                if (sbs==0&&satsys(sat_j,NULL)==SYS_SBS) continue;
                if (rtk->ssat[sat_j-1].lock[f]>=0&&
                    !(rtk->ssat[sat_j-1].slip[f]&(LLI_SLIP|LLI_HALFC))&&
                    rtk->ssat[sat_j-1].azel[1]>=rtk->opt.elmaskar&&!nofix) {
                    ix[nb*2  ]=i;
                    ix[nb*2+1]=j;
                    ref[nb]=i-k+1;
                    fix[nb++]=sat_j;
                    rtk->ssat[sat_j-1].fix[f]=2;
                    n++;
                }
                else rtk->ssat[sat_j-1].fix[f]=1;
            }
            if (n==0) rtk->ssat[i-k].fix[f]=1;
        }
    }
    if (nb>0) {
        trace(3,"PAR refSats=");tracemat(3,ref,1,nb,7,0);
        trace(3,"PAR fixSats=");tracemat(3,fix,1,nb,7,0);
    }
    return nb;
}

/* resolve integer ambiguity for PAR ----------------------------------------*/
static int resamb_PAR(rtk_t *rtk, double *bias, double *xa, int gps, int glo,
                      int sbs, const uint8_t par_excl[MAXSAT][NFREQ],
                      int commit)
{
    prcopt_t *opt=&rtk->opt;
    int i,j,nb,nb1,info,nx=rtk->nx,na=rtk->na;
    double *DP,*y,*b,*db,*Qb,*Qab,*QQ,s[2],coeff[3];
    int *ix;

    trace(3,"resamb_PAR : nx=%d commit=%d\n",nx,commit);

    rtk->sol.ratio=0.0;
    rtk->nb_ar=0;
    ix=imat(nx,2);
    if ((nb=ddidx_PAR(rtk,ix,gps,glo,sbs,par_excl))<(rtk->opt.minfixsats-1)) {
        if (commit) errmsg(rtk,"not enough valid PAR double-differences\n");
        free(ix);
        return -1;
    }
    rtk->nb_ar=nb;

    y=mat(nb,1); DP=mat(nb,nx-na); b=mat(nb,2); db=mat(nb,1); Qb=mat(nb,nb);
    Qab=mat(na,nb); QQ=mat(na,nb);

    for (i=0;i<nb;i++) {
        y[i]=rtk->x[ix[i*2]]-rtk->x[ix[i*2+1]];
    }
    for (j=0;j<nx-na;j++) for (i=0;i<nb;i++) {
        DP[i+j*nb]=rtk->P[ix[i*2]+(na+j)*nx]-rtk->P[ix[i*2+1]+(na+j)*nx];
    }
    for (j=0;j<nb;j++) for (i=0;i<nb;i++) {
        Qb[i+j*nb]=DP[i+(ix[j*2]-na)*nb]-DP[i+(ix[j*2+1]-na)*nb];
    }
    for (j=0;j<nb;j++) for (i=0;i<na;i++) {
        Qab[i+j*na]=rtk->P[i+ix[j*2]*nx]-rtk->P[i+ix[j*2+1]*nx];
    }

    if (!(info=lambda(nb,2,y,Qb,b,s))) {
        rtk->sol.ratio=s[0]>0?(float)(s[1]/s[0]):0.0f;
        if (rtk->sol.ratio>999.9) rtk->sol.ratio=999.9f;

        if (opt->thresar[5]!=opt->thresar[6]) {
            nb1=nb<50?nb:50;
            for (i=0;i<3;i++) {
                coeff[i]=ar_poly_coeffs[i][0];
                for (j=1;j<5;j++) {
                    coeff[i]=coeff[i]*opt->thresar[0]+ar_poly_coeffs[i][j];
                }
            }
            rtk->sol.thres=(float)coeff[0];
            for (i=1;i<3;i++) {
                rtk->sol.thres=(float)(rtk->sol.thres*1.0/(nb1+1.0)+coeff[i]);
            }
            rtk->sol.thres=(float)MIN(MAX(rtk->sol.thres,opt->thresar[5]),opt->thresar[6]);
        }
        else {
            rtk->sol.thres=(float)opt->thresar[0];
        }

        if (s[0]<=0.0||s[1]/s[0]>=rtk->sol.thres) {
            if (!commit) {
                trace(3,"resamb_PAR : trial validation ok (nb=%d ratio=%.2f thresh=%.2f)\n",
                      nb,rtk->sol.ratio,rtk->sol.thres);
            }
            else {
                for (i=0;i<na;i++) {
                    rtk->xa[i]=rtk->x[i];
                    for (j=0;j<na;j++) rtk->Pa[i+j*na]=rtk->P[i+j*nx];
                }
                for (i=0;i<nb;i++) {
                    bias[i]=b[i];
                    y[i]-=b[i];
                }
                if (!matinv(Qb,nb)) {
                    matmul("NN",nb,1,nb,Qb ,y,db);
                    matmulm("NN",na,1,nb,Qab,db,rtk->xa);
                    matmul("NN",na,nb,nb,Qab,Qb ,QQ);
                    matmulm("NT",na,na,nb,QQ ,Qab,rtk->Pa);
                    trace(3,"resamb_PAR : validation ok (nb=%d ratio=%.2f thresh=%.2f s=%.2f/%.2f)\n",
                          nb,s[0]==0.0?0.0:s[1]/s[0],rtk->sol.thres,s[0],s[1]);
                    restamb(rtk,bias,nb,xa);
                }
                else nb=0;
            }
        }
        else {
            if (commit) {
                errmsg(rtk,"PAR ambiguity validation failed (nb=%d ratio=%.2f thresh=%.2f s=%.2f/%.2f)\n",
                       nb,s[1]/s[0],rtk->sol.thres,s[0],s[1]);
            }
            nb=0;
        }
    }
    else {
        if (commit) errmsg(rtk,"PAR lambda error (info=%d)\n",info);
        nb=0;
    }
    free(ix);
    free(y); free(DP); free(b); free(db); free(Qb); free(Qab); free(QQ);

    return nb;
}

static int par_build_sd(rtk_t *rtk, const obsd_t *obs, const int *sat,
                        const int *iu, const int *ir, int ns, int nf,
                        const int refsat[6][NFREQ*2], const double *y,
                        double sd[MAXSAT][NFREQ],
                        uint8_t valid[MAXSAT][NFREQ])
{
    int i,f,n=0;

    for (i=0;i<MAXSAT;i++) for (f=0;f<NFREQ;f++) {
        sd[i][f]=0.0;
        valid[i][f]=0;
    }
    for (i=0;i<ns;i++) for (f=0;f<nf&&f<NFREQ;f++) {
        int s=sat[i],used;

        used=rtk->ssat[s-1].fix[f]==2||par_is_current_refsat(refsat,s,f);
        if (!used||!rtk->ssat[s-1].vsat[f]) continue;
        if (y[f+iu[i]*nf*2]==0.0||y[f+ir[i]*nf*2]==0.0) continue;
        if (rtk->ssat[s-1].slip[f]&(LLI_SLIP|LLI_HALFC)) continue;
        if ((obs[iu[i]].LLI[f]|obs[ir[i]].LLI[f])&(LLI_SLIP|LLI_HALFC)) continue;

        /* SD is built from zdres() zero-difference phase residuals, not ddres(). */
        sd[s-1][f]=y[f+iu[i]*nf*2]-y[f+ir[i]*nf*2];
        valid[s-1][f]=1;
        n++;
    }
    return n;
}

static void par_store_sd_cache(rtk_t *rtk, gtime_t time,
                               const double sd[MAXSAT][NFREQ],
                               const uint8_t valid[MAXSAT][NFREQ])
{
    int i,f,n=0;

    rtk->par_sd_time=time;
    for (i=0;i<MAXSAT;i++) for (f=0;f<NFREQ;f++) {
        rtk->par_sd[i][f]=valid[i][f]?sd[i][f]:0.0;
        rtk->par_sd_valid[i][f]=valid[i][f];
        if (valid[i][f]) n++;
    }
    rtk->par_sd_n=n;
}

static int par_collect_candidates(rtk_t *rtk, const int *sat, int ns, int nf,
                                  const int refsat[6][NFREQ*2],
                                  const double sd[MAXSAT][NFREQ],
                                  const uint8_t valid[MAXSAT][NFREQ],
                                  par_cand_t *cand, int *sys_count)
{
    int i,f,n=0,m;

    for (m=0;m<6;m++) sys_count[m]=0;
    for (i=0;i<ns;i++) for (f=0;f<nf&&f<NFREQ;f++) {
        int s=sat[i],sys=rtk->ssat[s-1].sys,ref;
        par_cand_t *c;

        if (rtk->ssat[s-1].fix[f]!=2) continue;
        if (sys==SYS_SBS) continue; /* TODO: SBAS PAR support is intentionally omitted. */
        if ((m=par_sys_index(sys))<0) continue;
        ref=refsat[m][f];
        if (ref<=0||ref==s||!valid[s-1][f]||!valid[ref-1][f]) continue;

        c=cand+n++;
        c->sat=s;
        c->freq=f;
        c->sys=sys;
        c->m=m;
        c->refsat=ref;
        c->prior=rtk->par_excl_prev[s-1][f]?1:0;
        c->ratio=c->thres=c->norm=0.0;
        c->improve=-1E99;
        if (rtk->par_sd_valid[s-1][f]&&rtk->par_sd_valid[ref-1][f]) {
            c->has_tdcp=1;
            c->tdcp=(sd[s-1][f]-sd[ref-1][f])-
                    (rtk->par_sd[s-1][f]-rtk->par_sd[ref-1][f]);
        }
        else {
            c->has_tdcp=0;
            c->tdcp=0.0;
        }
        sys_count[m]++;
    }
    return n;
}

static void par_sort_candidates(par_cand_t *cand, int n)
{
    int i,j;

    for (i=0;i<n-1;i++) for (j=i+1;j<n;j++) {
        int swap=0;

        if (cand[j].prior!=cand[i].prior) swap=cand[j].prior>cand[i].prior;
        else if (cand[j].has_tdcp!=cand[i].has_tdcp) swap=!cand[j].has_tdcp;
        else swap=fabs(cand[j].tdcp)>fabs(cand[i].tdcp);

        if (swap) {
            par_cand_t tmp=cand[i];
            cand[i]=cand[j];
            cand[j]=tmp;
        }
    }
}

static void par_select_final_excl(const par_cand_t *cand, int n,
                                  uint8_t excl[MAXSAT][NFREQ])
{
    int i,m,limit[6]={0},used[6]={0},count[6]={0},selected[MAXSAT*NFREQ]={0};

    par_clear_excl(excl);
    for (i=0;i<n;i++) count[cand[i].m]++;
    for (m=0;m<6;m++) {
        if (count[m]>0) limit[m]=(int)(count[m]*PAR_FINAL_DROP_RATE+0.999999);
    }
    for (;;) {
        int best=-1;
        double best_improve=0.0;

        for (i=0;i<n;i++) {
            if (selected[i]||cand[i].improve<=0.0) continue;
            if (used[cand[i].m]>=limit[cand[i].m]) continue;
            if (best<0||cand[i].improve>best_improve) {
                best=i;
                best_improve=cand[i].improve;
            }
        }
        if (best<0) break;
        excl[cand[best].sat-1][cand[best].freq]=1;
        selected[best]=1;
        used[cand[best].m]++;
    }
}

/* resolve integer ambiguity by PAR -----------------------------------------*/
static int manage_amb_PAR(rtk_t *rtk, const obsd_t *obs, const int *sat,
                          const int *iu, const int *ir, int ns, int nf,
                          const int refsat[6][NFREQ*2], const double *y,
                          double *bias, double *xa)
{
    uint8_t par_excl[MAXSAT][NFREQ]={{0}},final_excl[MAXSAT][NFREQ]={{0}};
    uint8_t sd_valid[MAXSAT][NFREQ];
    double sd[MAXSAT][NFREQ],ratio0,thres0,norm0;
    par_cand_t cand[MAXSAT*NFREQ];
    int sys_count[6],sys_left[6],i,nb,full_nb,ncand,sd_n,prev_ok;
    int gps1=1,glo1,sbas1=0,drops=0,max_drop,total_left;
    float posvar=0.0f;

    for (i=0;i<3;i++) posvar+=(float)rtk->P[i+i*rtk->nx];
    posvar/=3.0f;

    trace(3,"manage_amb_PAR: posvar=%.6f\n",posvar);
    trace(3,"manage_amb_PAR: prevRatios= %.3f %.3f\n",
          rtk->sol.prev_ratio1,rtk->sol.prev_ratio2);

    if (rtk->opt.mode<=PMODE_DGPS||rtk->opt.modear==ARMODE_OFF||
        rtk->opt.thresar[0]<1.0||posvar>rtk->opt.thresar[1]) {
        trace(3,"Skip PAR AR\n");
        rtk->sol.ratio=0.0;
        rtk->sol.prev_ratio1=rtk->sol.prev_ratio2=0.0;
        rtk->nb_ar=0;
        return 0;
    }
    /* TODO: move PAR constants to processing options after validation. */
    /* TODO: verify GLONASS fix-hold/autocal edge cases before special handling. */
    glo1=(rtk->opt.navsys&SYS_GLO)?(((rtk->opt.glomodear==GLO_ARMODE_FIXHOLD)&&!rtk->holdamb)?0:1):0;

    nb=resamb_PAR(rtk,bias,xa,gps1,glo1,sbas1,par_excl,1);
    full_nb=rtk->nb_ar;
    ratio0=rtk->sol.ratio;
    thres0=rtk->sol.thres;
    norm0=par_ratio_norm(ratio0,thres0);

    sd_n=par_build_sd(rtk,obs,sat,iu,ir,ns,nf,refsat,y,sd,sd_valid);
    if (nb>1) {
        par_store_sd_cache(rtk,obs[0].time,sd,sd_valid);
        par_clear_prev_excl(rtk);
        rtk->sol.prev_ratio1=(float)ratio0;
        rtk->sol.prev_ratio2=rtk->sol.ratio;
        return nb;
    }

    prev_ok=rtk->par_sd_n>0&&fabs(timediff(obs[0].time,rtk->par_sd_time))<=PAR_TDCP_MAX_AGE;
    if (!prev_ok||sd_n<=0||full_nb<=PAR_MIN_TOTAL_DD) {
        par_store_sd_cache(rtk,obs[0].time,sd,sd_valid);
        par_clear_prev_excl(rtk);
        rtk->sol.prev_ratio1=(float)ratio0;
        rtk->sol.prev_ratio2=rtk->sol.ratio;
        trace(3,"manage_amb_PAR: skip PAR prev_ok=%d sd_n=%d full_nb=%d\n",
              prev_ok,sd_n,full_nb);
        return 0;
    }

    ncand=par_collect_candidates(rtk,sat,ns,nf,refsat,sd,sd_valid,cand,sys_count);
    if (ncand<=0) {
        par_store_sd_cache(rtk,obs[0].time,sd,sd_valid);
        par_clear_prev_excl(rtk);
        rtk->sol.prev_ratio1=(float)ratio0;
        rtk->sol.prev_ratio2=rtk->sol.ratio;
        return 0;
    }
    par_sort_candidates(cand,ncand);

    for (i=0;i<6;i++) sys_left[i]=sys_count[i];
    max_drop=(int)(full_nb*PAR_MAX_DROP_FRAC);
    total_left=full_nb;

    for (i=0;i<ncand;i++) {
        par_snapshot_t snap;
        double norm1;

        if (par_excl[cand[i].sat-1][cand[i].freq]) continue;
        if (drops+1>max_drop) break;
        if (total_left-1<=PAR_MIN_TOTAL_DD) break;
        if (sys_left[cand[i].m]-1<=PAR_MIN_SYS_DD) continue;

        par_save_snapshot(rtk,&snap);
        par_excl[cand[i].sat-1][cand[i].freq]=1;

        nb=resamb_PAR(rtk,bias,xa,gps1,glo1,sbas1,par_excl,0);
        cand[i].ratio=rtk->sol.ratio;
        cand[i].thres=rtk->sol.thres;
        cand[i].norm=par_ratio_norm(cand[i].ratio,cand[i].thres);
        cand[i].improve=cand[i].norm-norm0;
        norm1=cand[i].norm;

        trace(3,"PAR try sat=%d f=%d tdcp=%s%.4f ratio=%.2f thres=%.2f improve=%.3f\n",
              cand[i].sat,cand[i].freq+1,cand[i].has_tdcp?"":"none ",
              cand[i].tdcp,cand[i].ratio,cand[i].thres,cand[i].improve);

        if (nb>1&&norm1>=1.0) {
            nb=resamb_PAR(rtk,bias,xa,gps1,glo1,sbas1,par_excl,1);
            if (nb>1) {
                par_store_prev_excl(rtk,par_excl);
                par_store_sd_cache(rtk,obs[0].time,sd,sd_valid);
                rtk->sol.prev_ratio1=(float)ratio0;
                rtk->sol.prev_ratio2=rtk->sol.ratio;
                return nb;
            }
        }
        if (par_ratio_improved(norm0,norm1)) {
            drops++;
            total_left--;
            sys_left[cand[i].m]--;
            continue;
        }
        par_excl[cand[i].sat-1][cand[i].freq]=0;
        par_restore_snapshot(rtk,&snap);
    }

    par_select_final_excl(cand,ncand,final_excl);
    par_store_prev_excl(rtk,final_excl);
    par_store_sd_cache(rtk,obs[0].time,sd,sd_valid);
    rtk->sol.prev_ratio1=(float)ratio0;
    rtk->sol.prev_ratio2=rtk->sol.ratio;
    return 0;
}

#endif /* RTKPOS_INCLUDE_RTKALG */
