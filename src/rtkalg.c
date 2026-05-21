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
        trace(3,"refsat geom: sys=%d frq=%d sat=%d ndd=%d kappa=%.3e score=%.3f%s%s\n",
              m,frq+1,sat[best],best_ndd,best_kappa,best_score,
              kept_prev?" kept":"",best_degraded?" degraded":"");
    }
}
