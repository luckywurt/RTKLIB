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
#define PAR_RATIO_FACTOR_DEFAULT 1.5
#define PAR_MIN_TOTAL_DD_DEFAULT 12
#define PAR_MIN_SYS_DD_DEFAULT 3
#define PAR_MAX_DROP_FRAC_DEFAULT 0.50
#define PAR_LOCK_FACTOR_DEFAULT 0.50
#define PAR_EXCLUDE_WHOLE_SAT 1  /* 0:sat/freq PAR, 1:whole-satellite PAR */
#define PAR_DIAG_MAX_CALLS 64
#define PAR_DIAG_TOP_DD 6

typedef struct {        /* PAR double-difference metadata */
    int refsat;
    int sat;
    int freq;
    int sys;
    int m;
    int ib_ref;
    int ib_sat;
} par_dd_t;

typedef struct {        /* PAR sat/frequency exclusion candidate */
    int sat;
    int freq;
    int sys;
    int m;
    int refsat;
    int refsat_by_freq[NFREQ];
    unsigned int freq_mask;
    unsigned int tdcp_mask;
    int dd_count;
    int lock0;
    int has_tdcp;
    double tdcp;
    double tdcp_by_freq[NFREQ];
} par_cand_t;

typedef struct {        /* PAR candidate group sharing DD reference */
    int m;
    int freq;
    int refsat;
    double tdcp;
} par_pair_group_t;

typedef struct {        /* scheduled PAR two-satellite trial */
    int a;
    int b;
    int channel;
    int m;
    int freq;
    int refsat;
} par_pair_trial_t;

typedef struct {        /* PAR single-frequency DD exclusion candidate */
    int dd_index;
    int sat;
    int freq;
    int m;
    int refsat;
    int lock0;
    int has_tdcp;
    int group_size;
    double qdiag;
    double qnorm;
    double tdcp;
    double tnorm;
    double rough;
    double ratio;
    double thres;
    double score;
    int score_valid;
} par_freq_cand_t;

typedef struct {        /* PAR trial state snapshot */
    uint8_t fix[MAXSAT][NFREQ];
    float ratio;
    float thres;
    int nb_ar;
} par_snapshot_t;

typedef struct {        /* diagnostic score for removing one DD */
    int dd_index;
    double score;
} par_diag_score_t;

static int par_ratio_valid(double ratio)
{
    return ratio==ratio&&ratio>0.0&&fabs(ratio)<1E99;
}

static int par_diag_trace_stage(const char *stage)
{
    return !strcmp(stage,"full")||!strncmp(stage,"diag-",5);
}

static int par_count_flags(const uint8_t flags[MAXSAT][NFREQ])
{
    int i,f,n=0;

    for (i=0;i<MAXSAT;i++) for (f=0;f<NFREQ;f++) {
        if (flags[i][f]) n++;
    }
    return n;
}

static unsigned int par_flags_hash(const uint8_t flags[MAXSAT][NFREQ])
{
    unsigned int hash=2166136261u;
    int i,f;

    for (i=0;i<MAXSAT;i++) for (f=0;f<NFREQ;f++) {
        if (!flags[i][f]) continue;
        hash^=(unsigned int)(i*NFREQ+f+1);
        hash*=16777619u;
    }
    return hash;
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

static int par_ar_active(const rtk_t *rtk, int sat, int f, int k, int m,
                         int sbs)
{
    if (sat<1||sat>MAXSAT) return 0;
    if (sbs==0&&satsys(sat,NULL)==SYS_SBS) return 0;
    if (rtk->x[k+sat-1]==0.0||!test_sys(rtk->ssat[sat-1].sys,m)||
        !rtk->ssat[sat-1].vsat[f]) {
        return 0;
    }
    return 1;
}

static int par_ar_fixable(const rtk_t *rtk, int sat, int f, int k, int m,
                          int sbs, int nofix,
                          const uint8_t par_excl[MAXSAT][NFREQ])
{
    if (!par_ar_active(rtk,sat,f,k,m,sbs)||par_excl[sat-1][f]) return 0;
    return rtk->ssat[sat-1].lock[f]>=0&&
           !(rtk->ssat[sat-1].slip[f]&(LLI_SLIP|LLI_HALFC))&&
           rtk->ssat[sat-1].azel[1]>=rtk->opt.elmaskar&&!nofix;
}

/* index for PAR single to double-difference transformation matrix (D') ------*/
static int ddidx_PAR(rtk_t *rtk, int *ix, int gps, int glo, int sbs,
                     const int refsat[6][NFREQ*2],
                     const uint8_t par_excl[MAXSAT][NFREQ],
                     par_dd_t *dd, int maxdd)
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
            int ref0=refsat?refsat[m][f]:0;

            i=k+MAXSAT;
            if (par_ar_fixable(rtk,ref0,f,k,m,sbs,nofix,par_excl)) {
                i=k+ref0-1;
                rtk->ssat[ref0-1].fix[f]=2;
            }
            else if (ref0>=1&&ref0<=MAXSAT&&
                     (par_excl[ref0-1][f]||par_ar_active(rtk,ref0,f,k,m,sbs))) {
                rtk->ssat[ref0-1].fix[f]=1;
            }
            if (i>=k+MAXSAT) {
                for (i=k;i<k+MAXSAT;i++) {
                    int sat_i=i-k+1;

                    if (par_excl[sat_i-1][f]) {
                        rtk->ssat[sat_i-1].fix[f]=1;
                        continue;
                    }
                    if (!par_ar_active(rtk,sat_i,f,k,m,sbs)) continue;
                    if (par_ar_fixable(rtk,sat_i,f,k,m,sbs,nofix,par_excl)) {
                        rtk->ssat[sat_i-1].fix[f]=2;
                        break;
                    }
                    else rtk->ssat[sat_i-1].fix[f]=1;
                }
            }
            if (i>=k+MAXSAT||rtk->ssat[i-k].fix[f]!=2) continue;

            for (n=0,j=k;j<k+MAXSAT;j++) {
                int sat_j=j-k+1;

                if (par_excl[sat_j-1][f]) {
                    rtk->ssat[sat_j-1].fix[f]=1;
                    continue;
                }
                if (i==j||!par_ar_active(rtk,sat_j,f,k,m,sbs)) continue;
                if (par_ar_fixable(rtk,sat_j,f,k,m,sbs,nofix,par_excl)) {
                    ix[nb*2  ]=i;
                    ix[nb*2+1]=j;
                    if (dd&&nb<maxdd) {
                        dd[nb].refsat=i-k+1;
                        dd[nb].sat=sat_j;
                        dd[nb].freq=f;
                        dd[nb].sys=rtk->ssat[sat_j-1].sys;
                        dd[nb].m=m;
                        dd[nb].ib_ref=i;
                        dd[nb].ib_sat=j;
                    }
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

/* translate PAR fixed DD phase-bias values to single-difference states ------*/
static void restamb_PAR(rtk_t *rtk, const double *bias, const int *ix, int nb,
                        double *xa)
{
    int i;

    trace(3,"restamb_PAR : nb=%d\n",nb);

    for (i=0;i<rtk->nx;i++) xa[i]=rtk->x [i];
    for (i=0;i<rtk->na;i++) xa[i]=rtk->xa[i];

    for (i=0;i<nb;i++) {
        xa[ix[i*2  ]]=rtk->x[ix[i*2]];
        xa[ix[i*2+1]]=xa[ix[i*2]]-bias[i];
    }
}

/* resolve integer ambiguity for PAR ----------------------------------------*/
static int resamb_PAR(rtk_t *rtk, double *bias, double *xa, int gps, int glo,
                      int sbs, const int refsat[6][NFREQ*2],
                      const uint8_t par_excl[MAXSAT][NFREQ], int commit,
                      par_dd_t *dd, int maxdd, gtime_t time, int call_id,
                      const char *stage)
{
    prcopt_t *opt=&rtk->opt;
    int i,j,nb,nb1,info,nx=rtk->nx,na=rtk->na;
    double *DP,*y,*b,*db,*Qb,*Qab,*QQ,s[2],coeff[3];
    int *ix;
    char tstr[64];

    trace(3,"resamb_PAR : nx=%d commit=%d\n",nx,commit);

    rtk->sol.ratio=0.0;
    rtk->sol.thres=(float)opt->thresar[0];
    rtk->nb_ar=0;
    ix=imat(nx,2);
    nb=ddidx_PAR(rtk,ix,gps,glo,sbs,refsat,par_excl,dd,maxdd);
    trace(3,"AR call: time=%s solver=par id=%d stage=%s commit=%d nb=%d ddhash=%08X excl=%d exclhash=%08X\n",
          time2str(time,tstr,3),call_id,stage,commit,nb,ar_dd_hash(ix,nb),
          par_count_flags(par_excl),par_flags_hash(par_excl));
    if (nb<(rtk->opt.minfixsats-1)) {
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
    if (rtk->opt.ardiag&&!strcmp(stage,"full")) {
        ar_diag_matrix(rtk,ix,nb,Qb,time,call_id,"par",stage);
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
        if (rtk->opt.ardiag&&par_diag_trace_stage(stage)) {
            trace(3,"AR diag lambda: time=%s solver=par id=%d stage=%s info=0 nb=%d ratio=%.9g thres=%.9g fixed=%d s0=%.9g s1=%.9g\n",
                  time2str(time,tstr,3),call_id,stage,nb,rtk->sol.ratio,
                  rtk->sol.thres,s[0]<=0.0||s[1]/s[0]>=rtk->sol.thres,
                  s[0],s[1]);
        }

        if (s[0]<=0.0||s[1]/s[0]>=rtk->sol.thres) {
            if (!commit) {
                trace(3,"resamb_PAR : trial validation ok (nb=%d ratio=%.2f thresh=%.2f)\n",
                      nb,rtk->sol.ratio,rtk->sol.thres);
            }
            else {
                for (i=0;i<nb;i++) {
                    bias[i]=b[i];
                    y[i]-=b[i];
                }
                if (!matinv(Qb,nb)) {
                    for (i=0;i<na;i++) {
                        rtk->xa[i]=rtk->x[i];
                        for (j=0;j<na;j++) rtk->Pa[i+j*na]=rtk->P[i+j*nx];
                    }
                    matmul("NN",nb,1,nb,Qb ,y,db);
                    matmulm("NN",na,1,nb,Qab,db,rtk->xa);
                    matmul("NN",na,nb,nb,Qab,Qb ,QQ);
                    matmulm("NT",na,na,nb,QQ ,Qab,rtk->Pa);
                    trace(3,"resamb_PAR : validation ok (nb=%d ratio=%.2f thresh=%.2f s=%.2f/%.2f)\n",
                          nb,s[0]==0.0?0.0:s[1]/s[0],rtk->sol.thres,s[0],s[1]);
                    restamb_PAR(rtk,bias,ix,nb,xa);
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
        if (rtk->opt.ardiag&&par_diag_trace_stage(stage)) {
            trace(3,"AR diag lambda: time=%s solver=par id=%d stage=%s info=%d nb=%d ratio=0 thres=%.9g fixed=0 s0=0 s1=0\n",
                  time2str(time,tstr,3),call_id,stage,info,nb,
                  opt->thresar[0]);
        }
        if (commit) errmsg(rtk,"PAR lambda error (info=%d)\n",info);
        nb=0;
    }
    free(ix);
    free(y); free(DP); free(b); free(db); free(Qb); free(Qab); free(QQ);

    return nb;
}

static int par_diag_probe(rtk_t *rtk, double *bias, double *xa, int gps,
                          int glo, int sbs,
                          const int refsat[6][NFREQ*2],
                          const uint8_t excl[MAXSAT][NFREQ], gtime_t time,
                          int *call_id, int *used, const char *mode, int m,
                          int f, const par_dd_t *full_dd, int full_nb,
                          int dd1, int dd2, double *score)
{
    par_snapshot_t snap;
    double ratio,thres;
    int nb,trial_nb,probe=*used+1,id1=dd1>=0&&dd1<full_nb?dd1:-1;
    int id2=dd2>=0&&dd2<full_nb?dd2:-1;
    const par_dd_t *a=id1>=0?full_dd+id1:NULL;
    const par_dd_t *b=id2>=0?full_dd+id2:NULL;
    char tstr[64];

    if (*used>=PAR_DIAG_MAX_CALLS) return 0;
    par_save_snapshot(rtk,&snap);
    nb=resamb_PAR(rtk,bias,xa,gps,glo,sbs,refsat,excl,0,NULL,0,time,
                  ++(*call_id),"diag-probe");
    trial_nb=rtk->nb_ar;
    ratio=rtk->sol.ratio;
    thres=rtk->sol.thres;
    *score=par_ratio_valid(ratio)&&thres>0.0?ratio/thres:-1.0;
    (*used)++;
    trace(3,"AR diag probe: time=%s probe=%d call_id=%d mode=%s m=%d f=%d dd1=%d ref1=%d sat1=%d f1=%d dd2=%d ref2=%d sat2=%d f2=%d flags=%d flaghash=%08X nb=%d ratio=%.9g thres=%.9g score=%.9g fixed=%d\n",
          time2str(time,tstr,3),probe,*call_id,mode,m,f>=0?f+1:0,id1,
          a?a->refsat:0,a?a->sat:0,a?a->freq+1:0,id2,b?b->refsat:0,
          b?b->sat:0,b?b->freq+1:0,par_count_flags(excl),
          par_flags_hash(excl),trial_nb,ratio,thres,*score,nb>1);
    par_restore_snapshot(rtk,&snap);
    return 1;
}

static void par_run_full_diagnostics(rtk_t *rtk, double *bias, double *xa,
                                     int gps, int glo, int sbs,
                                     const int refsat[6][NFREQ*2],
                                     const par_dd_t *full_dd, int full_nb,
                                     gtime_t time, int *call_id)
{
    uint8_t excl[MAXSAT][NFREQ]={{0}};
    par_diag_score_t scores[MAXSAT*NFREQ];
    par_snapshot_t full_snap;
    int i,j,m,f,nf=NF(&rtk->opt),used=0,ngroup=0,nsingle=0,npair=0;
    int nscore=0,topn,pair_reserve=PAR_DIAG_TOP_DD*(PAR_DIAG_TOP_DD-1)/2;
    double score;
    char tstr[64];

    if (!rtk->opt.ardiag||nf<2||full_nb<=0) return;
    par_save_snapshot(rtk,&full_snap);

    for (i=0;i<full_nb;i++) {
        if (full_dd[i].freq!=0) excl[full_dd[i].sat-1][full_dd[i].freq]=1;
    }
    if (par_count_flags(excl)>0&&used<PAR_DIAG_MAX_CALLS) {
        par_diag_probe(rtk,bias,xa,gps,glo,sbs,refsat,excl,time,call_id,
                       &used,"l1-only",-1,0,full_dd,full_nb,-1,-1,&score);
    }
    memset(excl,0,sizeof(excl));
    for (i=0;i<full_nb;i++) {
        if (full_dd[i].freq!=1) excl[full_dd[i].sat-1][full_dd[i].freq]=1;
    }
    if (par_count_flags(excl)>0&&used<PAR_DIAG_MAX_CALLS) {
        par_diag_probe(rtk,bias,xa,gps,glo,sbs,refsat,excl,time,call_id,
                       &used,"l2-only",-1,1,full_dd,full_nb,-1,-1,&score);
    }

    for (m=0;m<6&&used<PAR_DIAG_MAX_CALLS;m++) for (f=0;f<nf&&
        used<PAR_DIAG_MAX_CALLS;f++) {
        int n=0;

        memset(excl,0,sizeof(excl));
        for (i=0;i<full_nb;i++) {
            if (full_dd[i].m!=m||full_dd[i].freq!=f) continue;
            excl[full_dd[i].sat-1][f]=1;
            n++;
        }
        if (!n) continue;
        if (par_diag_probe(rtk,bias,xa,gps,glo,sbs,refsat,excl,time,
                           call_id,&used,"drop-group",m,f,full_dd,full_nb,
                           -1,-1,&score)) {
            ngroup++;
        }
    }

    for (i=0;i<full_nb&&used<PAR_DIAG_MAX_CALLS-pair_reserve;i++) {
        memset(excl,0,sizeof(excl));
        excl[full_dd[i].sat-1][full_dd[i].freq]=1;
        if (!par_diag_probe(rtk,bias,xa,gps,glo,sbs,refsat,excl,time,
                            call_id,&used,"drop-one-dd",full_dd[i].m,
                            full_dd[i].freq,full_dd,full_nb,i,-1,&score)) {
            break;
        }
        scores[nscore].dd_index=i;
        scores[nscore++].score=score;
        nsingle++;
    }
    for (i=1;i<nscore;i++) {
        par_diag_score_t v=scores[i];

        for (j=i;j>0&&scores[j-1].score<v.score;j--) scores[j]=scores[j-1];
        scores[j]=v;
    }
    topn=MIN(nscore,PAR_DIAG_TOP_DD);
    for (i=1;i<topn&&used<PAR_DIAG_MAX_CALLS;i++) for (j=0;j<i&&
        used<PAR_DIAG_MAX_CALLS;j++) {
        int d1=scores[j].dd_index,d2=scores[i].dd_index;

        if (scores[j].score<0.0||scores[i].score<0.0) continue;
        memset(excl,0,sizeof(excl));
        excl[full_dd[d1].sat-1][full_dd[d1].freq]=1;
        excl[full_dd[d2].sat-1][full_dd[d2].freq]=1;
        if (par_count_flags(excl)<2) continue;
        if (par_diag_probe(rtk,bias,xa,gps,glo,sbs,refsat,excl,time,
                           call_id,&used,"drop-pair-dd",-1,-1,full_dd,
                           full_nb,d1,d2,&score)) {
            npair++;
        }
    }
    par_restore_snapshot(rtk,&full_snap);
    trace(3,"AR diag summary: time=%s calls=%d max=%d groups=%d singles=%d pairs=%d full_nb=%d restored_ratio=%.9g restored_thres=%.9g restored_nb=%d\n",
          time2str(time,tstr,3),used,PAR_DIAG_MAX_CALLS,ngroup,nsingle,npair,
          full_nb,rtk->sol.ratio,rtk->sol.thres,rtk->nb_ar);
}

static int par_build_sd(rtk_t *rtk, const obsd_t *obs, const int *sat,
                        const int *iu, const int *ir, int ns, int nf,
                        const double *y, double sd[MAXSAT][NFREQ],
                        uint8_t valid[MAXSAT][NFREQ])
{
    int i,f,n=0;

    for (i=0;i<MAXSAT;i++) for (f=0;f<NFREQ;f++) {
        sd[i][f]=0.0;
        valid[i][f]=0;
    }
    for (i=0;i<ns;i++) for (f=0;f<nf&&f<NFREQ;f++) {
        int s=sat[i];

        if (rtk->ssat[s-1].fix[f]!=2||!rtk->ssat[s-1].vsat[f]) continue;
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

static int par_candidate_flagged(const uint8_t flags[MAXSAT][NFREQ],
                                 const par_cand_t *cand)
{
    int f;

    for (f=0;f<NFREQ;f++) {
        if ((cand->freq_mask&(1u<<f))&&flags[cand->sat-1][f]) return 1;
    }
    return 0;
}

static void par_set_candidate_flag(uint8_t flags[MAXSAT][NFREQ],
                                   const par_cand_t *cand, uint8_t value)
{
    int f;

    for (f=0;f<NFREQ;f++) {
        if (cand->freq_mask&(1u<<f)) flags[cand->sat-1][f]=value;
    }
}

static int par_collect_candidates(rtk_t *rtk, const par_dd_t *dd, int ndd,
                                  const double sd[MAXSAT][NFREQ],
                                  const uint8_t valid[MAXSAT][NFREQ],
                                  int cache_ok, par_cand_t *cand,
                                  int *sys_count)
{
    int i,n=0,m,out=0;

    for (m=0;m<6;m++) sys_count[m]=0;
    for (i=0;i<ndd;i++) {
        int s=dd[i].sat,f=dd[i].freq,sys=dd[i].sys,ref=dd[i].refsat;
        par_cand_t *c;
        double tdcp=0.0;
        int prev_ok,p;

        if (sys==SYS_SBS) continue; /* TODO: SBAS PAR support is intentionally omitted. */
        m=dd[i].m;
        sys_count[m]++;
        if (ref<=0||ref==s) continue;

        p=n;
        if (PAR_EXCLUDE_WHOLE_SAT) {
            for (p=0;p<n;p++) {
                if (cand[p].sat==s&&cand[p].m==m) break;
            }
        }
        if (p>=n) {
            int ff;

            c=cand+n++;
            c->sat=s;
            c->freq=f;
            c->sys=sys;
            c->m=m;
            c->refsat=ref;
            c->freq_mask=0u;
            c->tdcp_mask=0u;
            c->dd_count=0;
            c->lock0=0;
            c->has_tdcp=0;
            c->tdcp=0.0;
            for (ff=0;ff<NFREQ;ff++) {
                c->refsat_by_freq[ff]=0;
                c->tdcp_by_freq[ff]=0.0;
            }
        }
        else c=cand+p;

        c->freq_mask|=1u<<f;
        c->refsat_by_freq[f]=ref;
        c->dd_count++;
        if (rtk->ssat[s-1].lock[f]==0) c->lock0=1;

        prev_ok=valid[s-1][f]&&valid[ref-1][f]&&cache_ok&&
                rtk->par_sd_valid[s-1][f]&&
                rtk->par_sd_valid[ref-1][f];
        if (!prev_ok) continue;
        c->has_tdcp=1;
        c->tdcp_mask|=1u<<f;
        tdcp=(sd[s-1][f]-sd[ref-1][f])-
             (rtk->par_sd[s-1][f]-rtk->par_sd[ref-1][f]);
        c->tdcp_by_freq[f]=tdcp;
        if (fabs(tdcp)>=fabs(c->tdcp)) {
            c->tdcp=tdcp;
            c->freq=f;
            c->refsat=ref;
        }
    }
    for (i=0;i<n;i++) {
        if (!cand[i].lock0&&!cand[i].has_tdcp) continue;
        if (out!=i) cand[out]=cand[i];
        out++;
    }
    return out;
}

static void par_sort_candidates(par_cand_t *cand, int n)
{
    int i,j;

    for (i=0;i<n-1;i++) for (j=0;j<n-i-1;j++) {
        int swap=0;

        if (cand[j+1].lock0!=cand[j].lock0) {
            swap=cand[j+1].lock0>cand[j].lock0;
        }
        else if (cand[j+1].has_tdcp!=cand[j].has_tdcp) {
            swap=cand[j+1].has_tdcp>cand[j].has_tdcp;
        }
        else swap=fabs(cand[j+1].tdcp)>fabs(cand[j].tdcp);

        if (swap) {
            par_cand_t tmp=cand[j];
            cand[j]=cand[j+1];
            cand[j+1]=tmp;
        }
    }
}

static int par_freq_rough_better(const par_freq_cand_t *cand, int a, int b)
{
    if (cand[a].rough!=cand[b].rough) return cand[a].rough>cand[b].rough;
    if (cand[a].qdiag!=cand[b].qdiag) return cand[a].qdiag>cand[b].qdiag;
    if (cand[a].lock0!=cand[b].lock0) return cand[a].lock0>cand[b].lock0;
    return fabs(cand[a].tdcp)>fabs(cand[b].tdcp);
}

static void par_sort_freq_rough(const par_freq_cand_t *cand, int *order, int n)
{
    int i,j,key;

    for (i=1;i<n;i++) {
        key=order[i];
        for (j=i;j>0&&par_freq_rough_better(cand,key,order[j-1]);j--) {
            order[j]=order[j-1];
        }
        order[j]=key;
    }
}

static void par_sort_freq_score(const par_freq_cand_t *cand, int *order, int n)
{
    int i,j,key;

    for (i=1;i<n;i++) {
        key=order[i];
        for (j=i;j>0;j--) {
            int prev=order[j-1],better;

            if (cand[key].score_valid!=cand[prev].score_valid) {
                better=cand[key].score_valid>cand[prev].score_valid;
            }
            else if (cand[key].score_valid&&cand[key].score!=cand[prev].score) {
                better=cand[key].score>cand[prev].score;
            }
            else better=par_freq_rough_better(cand,key,prev);
            if (!better) break;
            order[j]=prev;
        }
        order[j]=key;
    }
}

static int par_collect_freq_candidates(
    const rtk_t *rtk, const par_dd_t *dd, int ndd,
    const uint8_t par_excl[MAXSAT][NFREQ],
    const double sd[MAXSAT][NFREQ],
    const uint8_t valid[MAXSAT][NFREQ], int cache_ok,
    par_freq_cand_t *cand)
{
    double qmin[6][NFREQ],qmax[6][NFREQ],tmax[6][NFREQ];
    int group_size[6][NFREQ]={{0}};
    int i,m,f,n=0;

    for (m=0;m<6;m++) for (f=0;f<NFREQ;f++) {
        qmin[m][f]=1E99;
        qmax[m][f]=0.0;
        tmax[m][f]=0.0;
    }
    for (i=0;i<ndd;i++) {
        const par_dd_t *d=dd+i;
        par_freq_cand_t *c;
        double qdiag;

        if (d->sat<1||d->sat>MAXSAT||d->freq<0||d->freq>=NFREQ||
            d->m<0||d->m>=6||par_excl[d->sat-1][d->freq]) {
            continue;
        }
        c=cand+n++;
        qdiag=rtk->P[d->ib_ref+d->ib_ref*rtk->nx]+
              rtk->P[d->ib_sat+d->ib_sat*rtk->nx]-
              rtk->P[d->ib_ref+d->ib_sat*rtk->nx]-
              rtk->P[d->ib_sat+d->ib_ref*rtk->nx];
        if (!(qdiag==qdiag)||fabs(qdiag)>=1E99) qdiag=0.0;
        if (qdiag<0.0) qdiag=0.0;
        c->dd_index=i;
        c->sat=d->sat;
        c->freq=d->freq;
        c->m=d->m;
        c->refsat=d->refsat;
        c->lock0=rtk->ssat[d->sat-1].lock[d->freq]==0;
        c->has_tdcp=valid[d->sat-1][d->freq]&&
                    valid[d->refsat-1][d->freq]&&cache_ok&&
                    rtk->par_sd_valid[d->sat-1][d->freq]&&
                    rtk->par_sd_valid[d->refsat-1][d->freq];
        c->tdcp=c->has_tdcp?
            (sd[d->sat-1][d->freq]-sd[d->refsat-1][d->freq])-
            (rtk->par_sd[d->sat-1][d->freq]-
             rtk->par_sd[d->refsat-1][d->freq]):0.0;
        c->qdiag=qdiag;
        c->qnorm=c->tnorm=c->rough=0.0;
        c->ratio=c->thres=0.0;
        c->score=-1.0;
        c->score_valid=0;
        group_size[d->m][d->freq]++;
        qmin[d->m][d->freq]=MIN(qmin[d->m][d->freq],qdiag);
        qmax[d->m][d->freq]=MAX(qmax[d->m][d->freq],qdiag);
        tmax[d->m][d->freq]=MAX(tmax[d->m][d->freq],fabs(c->tdcp));
    }
    for (i=0;i<n;i++) {
        par_freq_cand_t *c=cand+i;
        double range=qmax[c->m][c->freq]-qmin[c->m][c->freq];

        c->group_size=group_size[c->m][c->freq];
        c->qnorm=range>0.0?(c->qdiag-qmin[c->m][c->freq])/range:0.0;
        c->tnorm=tmax[c->m][c->freq]>0.0?
                 fabs(c->tdcp)/tmax[c->m][c->freq]:0.0;
        c->rough=c->qnorm+0.5*(c->lock0?1.0:0.0)+0.5*c->tnorm;
    }
    return n;
}

static int par_pair_rank_better(const par_cand_t *cand, int a, int b,
                                const double *score, const int *score_valid,
                                const int *score_gen, int generation,
                                int group_freq)
{
    int class_a=score_valid[a]?(score_gen[a]==generation?2:1):0;
    int class_b=score_valid[b]?(score_gen[b]==generation?2:1):0;
    double tdcp_a=group_freq>=0?cand[a].tdcp_by_freq[group_freq]:cand[a].tdcp;
    double tdcp_b=group_freq>=0?cand[b].tdcp_by_freq[group_freq]:cand[b].tdcp;

    if (class_a!=class_b) return class_a>class_b;
    if (class_a>0&&score_gen[a]!=score_gen[b]) return score_gen[a]>score_gen[b];
    if (class_a>0&&score[a]!=score[b]) return score[a]>score[b];
    if (cand[a].lock0!=cand[b].lock0) return cand[a].lock0>cand[b].lock0;
    return fabs(tdcp_a)>fabs(tdcp_b);
}

static void par_sort_pair_order(const par_cand_t *cand, int *order, int n,
                                const double *score, const int *score_valid,
                                const int *score_gen, int generation,
                                int group_freq)
{
    int i,j,key;

    for (i=1;i<n;i++) {
        key=order[i];
        for (j=i;j>0&&par_pair_rank_better(cand,key,order[j-1],score,
                                           score_valid,score_gen,generation,
                                           group_freq);j--) {
            order[j]=order[j-1];
        }
        order[j]=key;
    }
}

static int par_pair_allowed(const par_cand_t *cand, int a, int b, int drops,
                            int max_drop, int total_left, int min_total_dd,
                            const int *sys_left, int min_sys_dd)
{
    int m,pair_dd=cand[a].dd_count+cand[b].dd_count;

    if (a==b||cand[a].sat==cand[b].sat||drops+pair_dd>max_drop||
        total_left-pair_dd<=min_total_dd) {
        return 0;
    }
    for (m=0;m<6;m++) {
        int sys_drop=(cand[a].m==m?cand[a].dd_count:0)+
                     (cand[b].m==m?cand[b].dd_count:0);
        if (sys_drop>0&&sys_left[m]-sys_drop<=min_sys_dd) return 0;
    }
    return 1;
}

static int par_append_pair(par_pair_trial_t *queue, int n, int capacity,
                           int a, int b, int channel,
                           const par_pair_group_t *group,
                           const par_cand_t *cand, int drops, int max_drop,
                           int total_left, int min_total_dd,
                           const int *sys_left, int min_sys_dd)
{
    int i;

    if (n>=capacity||!par_pair_allowed(cand,a,b,drops,max_drop,total_left,
                                       min_total_dd,sys_left,min_sys_dd)) {
        return n;
    }
    for (i=0;i<n;i++) {
        if ((queue[i].a==a&&queue[i].b==b)||
            (queue[i].a==b&&queue[i].b==a)) {
            return n;
        }
    }
    queue[n].a=a;
    queue[n].b=b;
    queue[n].channel=channel;
    queue[n].m=group?group->m:-1;
    queue[n].freq=group?group->freq:-1;
    queue[n].refsat=group?group->refsat:0;
    return n+1;
}

static int par_collect_new_excl(const rtk_t *rtk, const par_dd_t *dd, int ndd,
                                uint8_t excl[MAXSAT][NFREQ], int *sys_drop)
{
    uint8_t new_sat[MAXSAT]={0};
    int i,m,n=0;

    for (m=0;m<6;m++) sys_drop[m]=0;
    for (i=0;i<ndd;i++) {
        if (rtk->ssat[dd[i].sat-1].lock[dd[i].freq]==0) {
            new_sat[dd[i].sat-1]=1;
        }
    }
    for (i=0;i<ndd;i++) {
        int s=dd[i].sat,f=dd[i].freq;

        if (!new_sat[s-1]) continue;
        if (excl[s-1][f]) continue;
        excl[s-1][f]=1;
        sys_drop[dd[i].m]++;
        n++;
    }
    return n;
}

static void par_merge_flags(uint8_t dest[MAXSAT][NFREQ],
                            const uint8_t src[MAXSAT][NFREQ])
{
    int i,f;

    for (i=0;i<MAXSAT;i++) for (f=0;f<NFREQ;f++) {
        if (src[i][f]) dest[i][f]=1;
    }
}

static void par_record_lock_reset(int reset_nb[MAXSAT][NFREQ],
                                  const uint8_t reset[MAXSAT][NFREQ], int nb)
{
    int i,f;

    for (i=0;i<MAXSAT;i++) for (f=0;f<NFREQ;f++) {
        if (reset[i][f]) reset_nb[i][f]=nb;
    }
}

static void par_record_candidate_lock_reset(int reset_nb[MAXSAT][NFREQ],
                                            const par_cand_t *cand, int nb)
{
    int f;

    for (f=0;f<NFREQ;f++) {
        if (cand->freq_mask&(1u<<f)) reset_nb[cand->sat-1][f]=nb;
    }
}

static int par_apply_lock_reset(rtk_t *rtk,
                                const int reset_nb[MAXSAT][NFREQ],
                                double lock_factor)
{
    int i,f,n=0,lock_delay;

    for (i=0;i<MAXSAT;i++) for (f=0;f<NFREQ;f++) {
        if (reset_nb[i][f]<=0) continue;
        lock_delay=-(int)ceil(reset_nb[i][f]*lock_factor);
        if (lock_delay>=0) lock_delay=-1;
        rtk->ssat[i].lock[f]=MIN(rtk->ssat[i].lock[f],lock_delay);
        n++;
        trace(3,"PAR lock reset sat=%d f=%d nb=%d factor=%.3f lock=%d\n",
              i+1,f+1,reset_nb[i][f],lock_factor,rtk->ssat[i].lock[f]);
    }
    return n;
}

/* resolve integer ambiguity by PAR -----------------------------------------*/
static int manage_amb_PAR(rtk_t *rtk, const obsd_t *obs, const int *sat,
                          const int *iu, const int *ir, int ns, int nf,
                          const int refsat[6][NFREQ*2], const double *y,
                          double *bias, double *xa,
                          int previous_solution_fixed)
{
    uint8_t par_excl[MAXSAT][NFREQ]={{0}},batch_excl[MAXSAT][NFREQ]={{0}};
    uint8_t sd_valid[MAXSAT][NFREQ];
    int lock_reset_nb[MAXSAT][NFREQ]={{0}};
    double sd[MAXSAT][NFREQ],full_ratio,full_thres,current_ratio,current_thres;
    double ratio_factor,max_drop_frac,lock_factor;
    double single_score[MAXSAT*NFREQ],single_ratio[MAXSAT*NFREQ];
    double single_thres[MAXSAT*NFREQ];
    par_cand_t cand[MAXSAT*NFREQ];
    par_pair_group_t pair_group[MAXSAT*NFREQ];
    par_dd_t full_dd[MAXSAT*NFREQ];
    int pair_order[MAXSAT*NFREQ],group_order[MAXSAT*NFREQ];
    int single_score_valid[MAXSAT*NFREQ],single_score_gen[MAXSAT*NFREQ];
    int sys_count[6],sys_left[6],batch_sys[6];
    int i,m,nb,full_nb,ncand,sd_n,cache_ok,current_ratio_ok;
    int gps1=1,glo1,sbas1=0,drops=0,max_drop,total_left;
    int min_total_dd,min_sys_dd,batch_min_sys_dd,lock_count=0;
    int max_pair_trials,max_freq_trials;
    int ar_call_id=0;
    int exclusion_generation=0;
    float posvar=0.0f;

    for (i=0;i<3;i++) posvar+=(float)rtk->P[i+i*rtk->nx];
    posvar/=3.0f;

    ratio_factor=rtk->opt.par_ratio_factor;
    if (!(ratio_factor==ratio_factor)||ratio_factor<1.0||
        fabs(ratio_factor)>=1E99) ratio_factor=PAR_RATIO_FACTOR_DEFAULT;
    max_drop_frac=rtk->opt.par_max_drop_frac;
    if (!(max_drop_frac==max_drop_frac)||max_drop_frac<=0.0||
        max_drop_frac>1.0) max_drop_frac=PAR_MAX_DROP_FRAC_DEFAULT;
    lock_factor=rtk->opt.par_lock_factor;
    if (!(lock_factor==lock_factor)||lock_factor<=0.0||
        fabs(lock_factor)>=1E99) lock_factor=PAR_LOCK_FACTOR_DEFAULT;
    min_total_dd=rtk->opt.par_min_total_dd>=0?
                 rtk->opt.par_min_total_dd:PAR_MIN_TOTAL_DD_DEFAULT;
    min_sys_dd=rtk->opt.par_min_sys_dd>=0?
               rtk->opt.par_min_sys_dd:PAR_MIN_SYS_DD_DEFAULT;
    batch_min_sys_dd=rtk->opt.par_batch_min_sys_dd>0?
                     rtk->opt.par_batch_min_sys_dd:min_sys_dd;
    max_pair_trials=rtk->opt.par_max_pair_trials>=0?
                    rtk->opt.par_max_pair_trials:0;
    max_freq_trials=rtk->opt.par_max_freq_trials>=0?
                    rtk->opt.par_max_freq_trials:0;

    trace(3,"manage_amb_PAR: posvar=%.6f prev_fix=%d refsel=%d ratio_factor=%.3f min_dd=%d min_sys_dd=%d batch_min_sys_dd=%d max_drop=%.3f lock_factor=%.3f max_pair=%d max_freq=%d\n",
          posvar,previous_solution_fixed,rtk->opt.par_refsel==0?0:1,
          ratio_factor,min_total_dd,min_sys_dd,batch_min_sys_dd,max_drop_frac,
          lock_factor,max_pair_trials,max_freq_trials);
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
    glo1=(rtk->opt.navsys&SYS_GLO)&&
         rtk->opt.glomodear!=GLO_ARMODE_OFF&&
         (rtk->opt.glomodear!=GLO_ARMODE_FIXHOLD||rtk->holdamb);

    nb=resamb_PAR(rtk,bias,xa,gps1,glo1,sbas1,refsat,par_excl,1,
                  full_dd,MAXSAT*NFREQ,obs[0].time,++ar_call_id,"full");
    full_nb=rtk->nb_ar;
    full_ratio=rtk->sol.ratio;
    full_thres=rtk->sol.thres;
    current_ratio=full_ratio;
    current_thres=full_thres;
    current_ratio_ok=par_ratio_valid(full_ratio);
    trace(3,"PAR full: nb=%d full_nb=%d ratio=%.3f thres=%.3f valid=%d\n",
          nb,full_nb,full_ratio,full_thres,current_ratio_ok);

    sd_n=par_build_sd(rtk,obs,sat,iu,ir,ns,nf,y,sd,sd_valid);
    if (nb>1) {
        par_store_sd_cache(rtk,obs[0].time,sd,sd_valid);
        rtk->sol.prev_ratio1=(float)full_ratio;
        rtk->sol.prev_ratio2=(float)full_ratio;
        trace(3,"PAR result: full-fixed ratio=%.3f thres=%.3f\n",
              full_ratio,full_thres);
        return nb;
    }
    par_run_full_diagnostics(rtk,bias,xa,gps1,glo1,sbas1,refsat,full_dd,
                             full_nb,obs[0].time,&ar_call_id);

    cache_ok=rtk->par_sd_n>0&&
             fabs(timediff(obs[0].time,rtk->par_sd_time))<=PAR_TDCP_MAX_AGE;
    ncand=par_collect_candidates(rtk,full_dd,full_nb,sd,sd_valid,cache_ok,
                                 cand,sys_count);
    par_sort_candidates(cand,ncand);
    for (i=0;i<ncand;i++) {
        int f;

        single_score[i]=-1.0;
        single_ratio[i]=0.0;
        single_thres[i]=0.0;
        single_score_valid[i]=0;
        single_score_gen[i]=-1;
        trace(3,"PAR order %d: sat=%d f=%d mask=0x%X ref=%d dd=%d no_tdcp=%d lock0=%d tdcp=%.4f\n",
              i+1,cand[i].sat,cand[i].freq+1,cand[i].freq_mask,
              cand[i].refsat,cand[i].dd_count,!cand[i].has_tdcp,
              cand[i].lock0,cand[i].tdcp);
        for (f=0;f<NFREQ;f++) {
            if (!(cand[i].freq_mask&(1u<<f))) continue;
            trace(3,"PAR candidate ref: order=%d sat=%d m=%d f=%d ref=%d tdcp_valid=%d tdcp=%.4f\n",
                  i+1,cand[i].sat,cand[i].m,f+1,
                  cand[i].refsat_by_freq[f],
                  (cand[i].tdcp_mask&(1u<<f))!=0,
                  cand[i].tdcp_by_freq[f]);
        }
    }

    for (i=0;i<6;i++) sys_left[i]=sys_count[i];
    max_drop=(int)(full_nb*max_drop_frac);
    total_left=full_nb;

    if (previous_solution_fixed&&full_nb>min_total_dd) {
        int batch_dd=par_collect_new_excl(rtk,full_dd,full_nb,batch_excl,
                                          batch_sys);
        int batch_ok=batch_dd>0&&batch_dd<=max_drop&&
                     total_left-batch_dd>min_total_dd;

        for (m=0;m<6&&batch_ok;m++) {
            if (batch_sys[m]>0&&
                sys_left[m]-batch_sys[m]<=batch_min_sys_dd) {
                batch_ok=0;
            }
        }
        if (batch_ok) {
            par_snapshot_t snap;
            double trial_ratio,trial_thres;
            int trial_nb;

            par_save_snapshot(rtk,&snap);
            nb=resamb_PAR(rtk,bias,xa,gps1,glo1,sbas1,refsat,batch_excl,0,
                          NULL,0,obs[0].time,++ar_call_id,"batch-trial");
            trial_nb=rtk->nb_ar;
            trial_ratio=rtk->sol.ratio;
            trial_thres=rtk->sol.thres;
            trace(3,"PAR batch: flags=%d dd=%d nb=%d ratio=%.3f thres=%.3f current=%.3f\n",
                  par_count_flags(batch_excl),batch_dd,trial_nb,trial_ratio,
                  trial_thres,current_ratio);

            if (nb>1) {
                nb=resamb_PAR(rtk,bias,xa,gps1,glo1,sbas1,refsat,
                              batch_excl,1,NULL,0,obs[0].time,
                              ++ar_call_id,"batch-commit");
                if (nb>1) {
                    par_merge_flags(par_excl,batch_excl);
                    par_record_lock_reset(lock_reset_nb,batch_excl,trial_nb);
                    par_store_sd_cache(rtk,obs[0].time,sd,sd_valid);
                    lock_count=par_apply_lock_reset(rtk,lock_reset_nb,
                                                    lock_factor);
                    rtk->sol.prev_ratio1=(float)full_ratio;
                    rtk->sol.prev_ratio2=rtk->sol.ratio;
                    trace(3,"PAR result: batch-fixed ratio=%.3f thres=%.3f excl=%d lock_reset=%d\n",
                          rtk->sol.ratio,rtk->sol.thres,
                          par_count_flags(par_excl),lock_count);
                    return nb;
                }
                par_restore_snapshot(rtk,&snap);
            }
            else if (current_ratio_ok&&par_ratio_valid(trial_ratio)&&
                     trial_ratio>=current_ratio*ratio_factor) {
                par_merge_flags(par_excl,batch_excl);
                par_record_lock_reset(lock_reset_nb,batch_excl,trial_nb);
                drops+=batch_dd;
                total_left-=batch_dd;
                for (m=0;m<6;m++) sys_left[m]-=batch_sys[m];
                current_ratio=trial_ratio;
                current_thres=trial_thres;
                exclusion_generation++;
                trace(3,"PAR batch: accepted ratio=%.3f new_current=%.3f drops=%d\n",
                      trial_ratio,current_ratio,drops);
            }
            else {
                par_restore_snapshot(rtk,&snap);
                trace(3,"PAR batch: restored ratio=%.3f current=%.3f valid=%d\n",
                      trial_ratio,current_ratio,current_ratio_ok);
            }
        }
        else if (batch_dd>0) {
            trace(3,"PAR batch: skipped dd=%d total_left=%d max_drop=%d\n",
                  batch_dd,total_left,max_drop);
        }
    }

    for (i=0;i<ncand;i++) {
        par_snapshot_t snap;
        double trial_ratio,trial_thres;
        int trial_nb;

        if (par_candidate_flagged(par_excl,cand+i)) continue;
        if (drops+cand[i].dd_count>max_drop) continue;
        if (total_left-cand[i].dd_count<=min_total_dd) continue;
        if (sys_left[cand[i].m]-cand[i].dd_count<=min_sys_dd) continue;

        par_save_snapshot(rtk,&snap);
        par_set_candidate_flag(par_excl,cand+i,1);

        nb=resamb_PAR(rtk,bias,xa,gps1,glo1,sbas1,refsat,par_excl,0,
                      NULL,0,obs[0].time,++ar_call_id,"single-trial");
        trial_nb=rtk->nb_ar;
        trial_ratio=rtk->sol.ratio;
        trial_thres=rtk->sol.thres;
        single_score_gen[i]=exclusion_generation;
        single_ratio[i]=trial_ratio;
        single_thres[i]=trial_thres;
        single_score_valid[i]=par_ratio_valid(trial_ratio)&&trial_thres>0.0;
        single_score[i]=single_score_valid[i]?trial_ratio/trial_thres:-1.0;

        trace(3,"PAR try sat=%d f=%d mask=0x%X ref=%d dd=%d nb=%d no_tdcp=%d lock0=%d tdcp=%.4f ratio=%.3f thres=%.3f current=%.3f score=%.6f score_gen=%d\n",
              cand[i].sat,cand[i].freq+1,cand[i].freq_mask,
              cand[i].refsat,cand[i].dd_count,trial_nb,!cand[i].has_tdcp,
              cand[i].lock0,cand[i].tdcp,trial_ratio,trial_thres,
              current_ratio,single_score[i],single_score_gen[i]);

        if (nb>1) {
            nb=resamb_PAR(rtk,bias,xa,gps1,glo1,sbas1,refsat,par_excl,1,
                          NULL,0,obs[0].time,++ar_call_id,"single-commit");
            if (nb>1) {
                par_record_candidate_lock_reset(lock_reset_nb,cand+i,
                                                trial_nb);
                drops+=cand[i].dd_count;
                par_store_sd_cache(rtk,obs[0].time,sd,sd_valid);
                lock_count=par_apply_lock_reset(rtk,lock_reset_nb,
                                                lock_factor);
                rtk->sol.prev_ratio1=(float)full_ratio;
                rtk->sol.prev_ratio2=rtk->sol.ratio;
                trace(3,"PAR result: fixed ratio=%.3f thres=%.3f drops=%d final_excl=%d lock_reset=%d\n",
                      rtk->sol.ratio,rtk->sol.thres,drops,
                      par_count_flags(par_excl),lock_count);
                return nb;
            }
            par_set_candidate_flag(par_excl,cand+i,0);
            par_restore_snapshot(rtk,&snap);
            continue;
        }
        if (current_ratio_ok&&par_ratio_valid(trial_ratio)&&
            trial_ratio>=current_ratio*ratio_factor) {
            par_record_candidate_lock_reset(lock_reset_nb,cand+i,trial_nb);
            drops+=cand[i].dd_count;
            total_left-=cand[i].dd_count;
            sys_left[cand[i].m]-=cand[i].dd_count;
            current_ratio=trial_ratio;
            current_thres=trial_thres;
            exclusion_generation++;
            trace(3,"PAR try accepted sat=%d ratio=%.3f drops=%d\n",
                  cand[i].sat,current_ratio,drops);
            continue;
        }
        par_set_candidate_flag(par_excl,cand+i,0);
        par_restore_snapshot(rtk,&snap);
        trace(3,"PAR try restored sat=%d ratio=%.3f current=%.3f valid=%d\n",
              cand[i].sat,trial_ratio,current_ratio,current_ratio_ok);
    }

    if (max_freq_trials>=4&&nf>=2) {
        par_freq_cand_t freq_cand[MAXSAT*NFREQ];
        par_snapshot_t freq_base;
        int rough_order[MAXSAT*NFREQ],score_order[MAXSAT*NFREQ];
        int group_best[6*NFREQ],group_order_freq[6*NFREQ];
        uint8_t chosen[MAXSAT*NFREQ]={0};
        int freq_left[NFREQ]={0},prefix_sys[6]={0},prefix_freq[NFREQ]={0};
        int nfreqcand,ngroup=0,nrough=0,nscore=0,freq_calls=0;
        int prefix_target,prefix_limit,added=0,best_prefix=0,best_nb=0;
        double best_margin=-1.0,best_ratio=0.0,best_thres=0.0;
        int k;

        nfreqcand=par_collect_freq_candidates(rtk,full_dd,full_nb,par_excl,
                                               sd,sd_valid,cache_ok,
                                               freq_cand);
        for (i=0;i<6*NFREQ;i++) group_best[i]=-1;
        for (i=0;i<nfreqcand;i++) {
            int group=freq_cand[i].m*NFREQ+freq_cand[i].freq;

            if (drops+1>max_drop||total_left-1<=min_total_dd||
                sys_left[freq_cand[i].m]-1<=min_sys_dd) {
                continue;
            }
            rough_order[nrough++]=i;
            if (group_best[group]<0||
                par_freq_rough_better(freq_cand,i,group_best[group])) {
                group_best[group]=i;
            }
        }
        par_sort_freq_rough(freq_cand,rough_order,nrough);
        for (i=0;i<6*NFREQ;i++) {
            if (group_best[i]>=0) group_order_freq[ngroup++]=group_best[i];
        }
        par_sort_freq_rough(freq_cand,group_order_freq,ngroup);

        prefix_target=MIN(8,(max_freq_trials-1)/3);
        for (i=0;i<ngroup&&nscore<max_freq_trials-1-prefix_target;i++) {
            int index=group_order_freq[i];

            score_order[nscore++]=index;
            chosen[index]=1;
        }
        for (i=0;i<nrough&&nscore<max_freq_trials-1-prefix_target;i++) {
            int index=rough_order[i];

            if (chosen[index]) continue;
            score_order[nscore++]=index;
            chosen[index]=1;
        }
        for (i=0;i<full_nb;i++) {
            if (!par_excl[full_dd[i].sat-1][full_dd[i].freq]) {
                freq_left[full_dd[i].freq]++;
            }
        }
        trace(3,"PAR freq queue: candidates=%d eligible=%d groups=%d score_budget=%d prefix_target=%d budget=%d reserve_commit=1\n",
              nfreqcand,nrough,ngroup,nscore,prefix_target,max_freq_trials);

        rtk->sol.ratio=(float)current_ratio;
        rtk->sol.thres=(float)current_thres;
        par_save_snapshot(rtk,&freq_base);
        for (i=0;i<nscore;i++) {
            par_freq_cand_t *c=freq_cand+score_order[i];
            int trial_nb;

            par_excl[c->sat-1][c->freq]=1;
            nb=resamb_PAR(rtk,bias,xa,gps1,glo1,sbas1,refsat,par_excl,0,
                          NULL,0,obs[0].time,++ar_call_id,"freq-score");
            freq_calls++;
            trial_nb=rtk->nb_ar;
            c->ratio=rtk->sol.ratio;
            c->thres=rtk->sol.thres;
            c->score_valid=par_ratio_valid(c->ratio)&&c->thres>0.0;
            c->score=c->score_valid?c->ratio/c->thres:-1.0;
            trace(3,"PAR freq score: rank=%d dd=%d sat=%d m=%d f=%d ref=%d group=%d lock0=%d tdcp_valid=%d tdcp=%.4f qdiag=%.9g qnorm=%.6f tnorm=%.6f rough=%.6f nb=%d ratio=%.3f thres=%.3f score=%.6f budget_used=%d budget_left=%d fixed=%d\n",
                  i+1,c->dd_index,c->sat,c->m,c->freq+1,c->refsat,c->group_size,
                  c->lock0,c->has_tdcp,c->tdcp,c->qdiag,c->qnorm,c->tnorm,
                  c->rough,trial_nb,c->ratio,c->thres,c->score,freq_calls,
                  max_freq_trials-freq_calls,nb>1);
            par_excl[c->sat-1][c->freq]=0;
            par_restore_snapshot(rtk,&freq_base);
        }
        par_sort_freq_score(freq_cand,score_order,nscore);
        prefix_limit=MIN(MIN(8,nscore),max_freq_trials-1-freq_calls);
        for (k=0;k<prefix_limit;k++) {
            par_freq_cand_t *c=freq_cand+score_order[k];
            double trial_ratio,trial_thres,margin=-1.0;
            int trial_nb,allowed=1;

            par_excl[c->sat-1][c->freq]=1;
            added++;
            prefix_sys[c->m]++;
            prefix_freq[c->freq]++;
            if (drops+added>max_drop||total_left-added<=min_total_dd) {
                allowed=0;
            }
            for (m=0;m<6&&allowed;m++) {
                if (prefix_sys[m]>0&&
                    sys_left[m]-prefix_sys[m]<=min_sys_dd) allowed=0;
            }
            if (freq_left[0]-prefix_freq[0]<=0||
                freq_left[1]-prefix_freq[1]<=0) allowed=0;
            if (!allowed) {
                trace(3,"PAR freq prefix: prefix=%d action=constraint-stop drops=%d total_left=%d f1_left=%d f2_left=%d\n",
                      added,drops,total_left,freq_left[0]-prefix_freq[0],
                      freq_left[1]-prefix_freq[1]);
                break;
            }
            nb=resamb_PAR(rtk,bias,xa,gps1,glo1,sbas1,refsat,par_excl,0,
                          NULL,0,obs[0].time,++ar_call_id,"freq-prefix");
            freq_calls++;
            trial_nb=rtk->nb_ar;
            trial_ratio=rtk->sol.ratio;
            trial_thres=rtk->sol.thres;
            if (par_ratio_valid(trial_ratio)&&trial_thres>0.0) {
                margin=trial_ratio/trial_thres;
            }
            else if (nb>1) margin=0.0;
            trace(3,"PAR freq prefix: prefix=%d sat=%d f=%d nb=%d ratio=%.3f thres=%.3f margin=%.6f budget_used=%d budget_left=%d action=%s\n",
                  added,c->sat,c->freq+1,trial_nb,trial_ratio,trial_thres,
                  margin,freq_calls,max_freq_trials-freq_calls,
                  nb>1?"fixed":"restored");
            if (nb>1&&margin>best_margin) {
                best_margin=margin;
                best_ratio=trial_ratio;
                best_thres=trial_thres;
                best_prefix=added;
                best_nb=trial_nb;
            }
            par_restore_snapshot(rtk,&freq_base);
        }
        for (i=0;i<added;i++) {
            par_freq_cand_t *c=freq_cand+score_order[i];

            par_excl[c->sat-1][c->freq]=0;
        }
        par_restore_snapshot(rtk,&freq_base);
        if (best_prefix>0&&freq_calls<max_freq_trials) {
            for (i=0;i<best_prefix;i++) {
                par_freq_cand_t *c=freq_cand+score_order[i];

                par_excl[c->sat-1][c->freq]=1;
            }
            nb=resamb_PAR(rtk,bias,xa,gps1,glo1,sbas1,refsat,par_excl,1,
                          NULL,0,obs[0].time,++ar_call_id,"freq-commit");
            freq_calls++;
            if (nb>1) {
                for (i=0;i<best_prefix;i++) {
                    par_freq_cand_t *c=freq_cand+score_order[i];

                    lock_reset_nb[c->sat-1][c->freq]=best_nb;
                }
                drops+=best_prefix;
                par_store_sd_cache(rtk,obs[0].time,sd,sd_valid);
                lock_count=par_apply_lock_reset(rtk,lock_reset_nb,
                                                lock_factor);
                rtk->sol.prev_ratio1=(float)full_ratio;
                rtk->sol.prev_ratio2=rtk->sol.ratio;
                trace(3,"PAR freq result: fixed calls=%d budget=%d prefix=%d trial_ratio=%.3f trial_thres=%.3f commit_ratio=%.3f excl=%d lock_reset=%d\n",
                      freq_calls,max_freq_trials,best_prefix,best_ratio,
                      best_thres,rtk->sol.ratio,par_count_flags(par_excl),
                      lock_count);
                return nb;
            }
            for (i=0;i<best_prefix;i++) {
                par_freq_cand_t *c=freq_cand+score_order[i];

                par_excl[c->sat-1][c->freq]=0;
            }
            par_restore_snapshot(rtk,&freq_base);
            trace(3,"PAR freq result: commit-failed calls=%d budget=%d prefix=%d\n",
                  freq_calls,max_freq_trials,best_prefix);
        }
        else {
            trace(3,"PAR freq result: no-fix calls=%d budget=%d scored=%d prefixes=%d\n",
                  freq_calls,max_freq_trials,nscore,added);
        }
    }

    if (max_pair_trials>0) {
        par_snapshot_t pair_base;
        par_pair_trial_t *queue=NULL;
        double best_margin=-1.0,best_ratio=0.0,best_thres=0.0;
        int max_pairs=ncand>1?ncand*(ncand-1)/2:0;
        int trial_capacity=max_pair_trials>1?max_pair_trials-1:0;
        int queue_n=0,global_n=0,group_n=0,pair_calls=0,best_q=-1,best_nb=0;
        int f,g,j,k,q;

        if (trial_capacity>max_pairs) trial_capacity=max_pairs;
        if (trial_capacity>0) {
            queue=(par_pair_trial_t *)malloc(sizeof(*queue)*trial_capacity);
        }
        if (!queue) {
            trace(3,"PAR pair result: no-search budget=%d reserve_commit=1\n",
                  max_pair_trials);
        }
        else {
            rtk->sol.ratio=(float)current_ratio;
            rtk->sol.thres=(float)current_thres;
            par_save_snapshot(rtk,&pair_base);

            for (i=0;i<ncand;i++) {
                if (par_candidate_flagged(par_excl,cand+i)||
                    !single_score_valid[i]) {
                    continue;
                }
                pair_order[global_n++]=i;
            }
            par_sort_pair_order(cand,pair_order,global_n,single_score,
                                single_score_valid,single_score_gen,
                                exclusion_generation,-1);
            if (global_n>=2) {
                queue_n=par_append_pair(queue,queue_n,trial_capacity,
                                        pair_order[0],pair_order[1],0,NULL,
                                        cand,drops,max_drop,total_left,
                                        min_total_dd,sys_left,min_sys_dd);
            }

            for (i=0;i<ncand;i++) {
                if (par_candidate_flagged(par_excl,cand+i)) continue;
                for (f=0;f<NFREQ;f++) {
                    int ref=cand[i].refsat_by_freq[f];

                    if (!(cand[i].freq_mask&(1u<<f))||ref<=0) continue;
                    for (g=0;g<group_n;g++) {
                        if (pair_group[g].m==cand[i].m&&
                            pair_group[g].freq==f&&
                            pair_group[g].refsat==ref) {
                            break;
                        }
                    }
                    if (g>=group_n) {
                        pair_group[g].m=cand[i].m;
                        pair_group[g].freq=f;
                        pair_group[g].refsat=ref;
                        pair_group[g].tdcp=0.0;
                        group_n++;
                    }
                    if ((cand[i].tdcp_mask&(1u<<f))&&
                        fabs(cand[i].tdcp_by_freq[f])>pair_group[g].tdcp) {
                        pair_group[g].tdcp=fabs(cand[i].tdcp_by_freq[f]);
                    }
                }
            }
            for (i=0;i<group_n-1;i++) for (j=0;j<group_n-i-1;j++) {
                if (pair_group[j+1].tdcp>pair_group[j].tdcp) {
                    par_pair_group_t tmp=pair_group[j];
                    pair_group[j]=pair_group[j+1];
                    pair_group[j+1]=tmp;
                }
            }
            for (g=0;g<group_n&&queue_n<trial_capacity;g++) {
                int member_n=0;

                for (i=0;i<ncand;i++) {
                    if (par_candidate_flagged(par_excl,cand+i)||
                        cand[i].m!=pair_group[g].m||
                        !(cand[i].freq_mask&(1u<<pair_group[g].freq))||
                        cand[i].refsat_by_freq[pair_group[g].freq]!=
                        pair_group[g].refsat) {
                        continue;
                    }
                    group_order[member_n++]=i;
                }
                par_sort_pair_order(cand,group_order,member_n,single_score,
                                    single_score_valid,single_score_gen,
                                    exclusion_generation,pair_group[g].freq);
                trace(3,"PAR pair group: rank=%d m=%d f=%d ref=%d members=%d tdcp=%.4f queued=%d budget=%d\n",
                      g+1,pair_group[g].m,pair_group[g].freq+1,
                      pair_group[g].refsat,member_n,pair_group[g].tdcp,
                      queue_n,max_pair_trials);
                for (k=1;k<member_n&&queue_n<trial_capacity;k++) {
                    for (j=0;j<k&&queue_n<trial_capacity;j++) {
                        queue_n=par_append_pair(queue,queue_n,trial_capacity,
                                                group_order[j],group_order[k],
                                                1,pair_group+g,cand,drops,
                                                max_drop,total_left,
                                                min_total_dd,sys_left,
                                                min_sys_dd);
                    }
                }
            }
            for (k=1;k<global_n&&queue_n<trial_capacity;k++) {
                for (j=0;j<k&&queue_n<trial_capacity;j++) {
                    queue_n=par_append_pair(queue,queue_n,trial_capacity,
                                            pair_order[j],pair_order[k],2,
                                            NULL,cand,drops,max_drop,
                                            total_left,min_total_dd,sys_left,
                                            min_sys_dd);
                }
            }
            trace(3,"PAR pair queue: candidates=%d groups=%d queued=%d budget=%d reserve_commit=1 score_gen=%d\n",
                  global_n,group_n,queue_n,max_pair_trials,
                  exclusion_generation);

            for (q=0;q<queue_n;q++) {
                const par_cand_t *a=cand+queue[q].a;
                const par_cand_t *b=cand+queue[q].b;
                const char *channel=queue[q].channel==0?"probe":
                                    (queue[q].channel==1?"group":"global");
                double trial_ratio,trial_thres,margin=-1.0;
                int trial_nb;

                par_set_candidate_flag(par_excl,a,1);
                par_set_candidate_flag(par_excl,b,1);
                nb=resamb_PAR(rtk,bias,xa,gps1,glo1,sbas1,refsat,
                              par_excl,0,NULL,0,obs[0].time,++ar_call_id,
                              "pair-trial");
                pair_calls++;
                trial_nb=rtk->nb_ar;
                trial_ratio=rtk->sol.ratio;
                trial_thres=rtk->sol.thres;
                if (par_ratio_valid(trial_ratio)&&trial_thres>0.0) {
                    margin=trial_ratio/trial_thres;
                }
                else if (nb>1) margin=0.0;
                trace(3,"PAR pair try: trial=%d channel=%s queue_rank=%d m=%d f=%d ref=%d sat1=%d mask1=0x%X ratio1=%.3f thres1=%.3f score1=%.6f gen1=%d sat2=%d mask2=0x%X ratio2=%.3f thres2=%.3f score2=%.6f gen2=%d nb=%d ratio=%.3f thres=%.3f margin=%.6f budget_used=%d budget_left=%d action=%s\n",
                      pair_calls,channel,q+1,queue[q].m,queue[q].freq+1,
                      queue[q].refsat,a->sat,a->freq_mask,
                      single_ratio[queue[q].a],single_thres[queue[q].a],
                      single_score[queue[q].a],single_score_gen[queue[q].a],
                      b->sat,b->freq_mask,single_ratio[queue[q].b],
                      single_thres[queue[q].b],single_score[queue[q].b],
                      single_score_gen[queue[q].b],trial_nb,trial_ratio,
                      trial_thres,margin,pair_calls,
                      max_pair_trials-pair_calls,nb>1?"fixed":"restored");
                if (nb>1&&margin>best_margin) {
                    best_margin=margin;
                    best_ratio=trial_ratio;
                    best_thres=trial_thres;
                    best_q=q;
                    best_nb=trial_nb;
                }
                par_set_candidate_flag(par_excl,a,0);
                par_set_candidate_flag(par_excl,b,0);
                par_restore_snapshot(rtk,&pair_base);
            }
            if (best_q>=0&&pair_calls<max_pair_trials) {
                const par_cand_t *a=cand+queue[best_q].a;
                const par_cand_t *b=cand+queue[best_q].b;
                const char *channel=queue[best_q].channel==0?"probe":
                                    (queue[best_q].channel==1?"group":"global");

                par_set_candidate_flag(par_excl,a,1);
                par_set_candidate_flag(par_excl,b,1);
                nb=resamb_PAR(rtk,bias,xa,gps1,glo1,sbas1,refsat,par_excl,1,
                              NULL,0,obs[0].time,++ar_call_id,"pair-commit");
                pair_calls++;
                if (nb>1) {
                    par_record_candidate_lock_reset(lock_reset_nb,a,best_nb);
                    par_record_candidate_lock_reset(lock_reset_nb,b,best_nb);
                    drops+=a->dd_count+b->dd_count;
                    par_store_sd_cache(rtk,obs[0].time,sd,sd_valid);
                    lock_count=par_apply_lock_reset(rtk,lock_reset_nb,
                                                    lock_factor);
                    rtk->sol.prev_ratio1=(float)full_ratio;
                    rtk->sol.prev_ratio2=rtk->sol.ratio;
                    trace(3,"PAR pair result: fixed calls=%d budget=%d channel=%s sat1=%d mask1=0x%X sat2=%d mask2=0x%X trial_ratio=%.3f trial_thres=%.3f commit_ratio=%.3f excl=%d lock_reset=%d\n",
                          pair_calls,max_pair_trials,channel,a->sat,
                          a->freq_mask,b->sat,b->freq_mask,best_ratio,
                          best_thres,rtk->sol.ratio,
                          par_count_flags(par_excl),lock_count);
                    free(queue);
                    return nb;
                }
                par_set_candidate_flag(par_excl,a,0);
                par_set_candidate_flag(par_excl,b,0);
                par_restore_snapshot(rtk,&pair_base);
                trace(3,"PAR pair result: commit-failed calls=%d budget=%d sat1=%d sat2=%d\n",
                      pair_calls,max_pair_trials,a->sat,b->sat);
            }
            else {
                trace(3,"PAR pair result: no-fix calls=%d queued=%d budget=%d candidates=%d groups=%d\n",
                      pair_calls,queue_n,max_pair_trials,global_n,group_n);
            }
            free(queue);
        }
    }

    par_store_sd_cache(rtk,obs[0].time,sd,sd_valid);
    lock_count=par_apply_lock_reset(rtk,lock_reset_nb,lock_factor);
    rtk->sol.ratio=(float)current_ratio;
    rtk->sol.thres=(float)current_thres;
    rtk->sol.prev_ratio1=(float)full_ratio;
    rtk->sol.prev_ratio2=rtk->sol.ratio;
    trace(3,"PAR result: float ratio=%.3f thres=%.3f cache_ok=%d sd_n=%d drops=%d final_excl=%d lock_reset=%d\n",
          current_ratio,current_thres,cache_ok,sd_n,drops,
          par_count_flags(par_excl),lock_count);
    return 0;
}

#endif /* RTKPOS_INCLUDE_RTKALG */
