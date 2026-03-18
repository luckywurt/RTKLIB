/*------------------------------------------------------------------------------
* rnx2rtcm.c : RINEX observation to RTCM3 (GPS 1077 + BDS 1127)
*
* This tool is intentionally independent from util/rnx2rtcm and only keeps the
* required RTKLIB APIs.
*-----------------------------------------------------------------------------*/
#include "rtklib.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEFAULT_STAID      1
#define DEFAULT_LOCK_MIN   5.0     /* minutes */
#define MIN_LOCK_FALLBACK  60.0    /* seconds */
#define EPOCH_GROUP_TOL    1E-6    /* seconds */

typedef struct {
    FILE *fp_rtcm;
    FILE *fp_tag;
    gtime_t t0;
    char rtcm_path[MAXSTRPATH];
    char tag_path[MAXSTRPATH];
} outctx_t;

static const char *help[] = {
    "",
    "usage: rnx2rtcm [options] rinex_obs_file",
    "",
    "options:",
    "  -sta id         station id (default: 1)",
    "  -lock min       lock-time fallback in minutes (default: 5)",
    "  -locksec sec    lock-time fallback in seconds (overrides -lock)",
    "  -opt opt        rtcm options (passed to rtcm.opt)",
    "  -x level        trace level",
    "  -h              show this help",
    "",
    "outputs:",
    "  <input_basename>.rtcm3",
    "  <input_basename>.rtcm3.tag",
    "  (same directory as input file)",
    ""
};

static void print_help(void)
{
    int i;
    for (i=0;i<(int)(sizeof(help)/sizeof(help[0]));i++) {
        fprintf(stderr,"%s\n",help[i]);
    }
}

static void build_output_paths(const char *infile, char *rtcm, size_t nrtcm,
                               char *tag, size_t ntag)
{
    const char *p_sep1=strrchr(infile,'/');
    const char *p_sep2=strrchr(infile,'\\');
    const char *p_sep=p_sep1>p_sep2?p_sep1:p_sep2;
    const char *p_dot=strrchr(infile,'.');
    size_t len;

    if (p_dot&&(!p_sep||p_dot>p_sep)) len=(size_t)(p_dot-infile);
    else len=strlen(infile);

    if (len>=nrtcm) len=nrtcm-1;
    memcpy(rtcm,infile,len);
    rtcm[len]='\0';
    strncat(rtcm,".rtcm3",nrtcm-strlen(rtcm)-1);

    strncpy(tag,rtcm,ntag-1);
    tag[ntag-1]='\0';
    strncat(tag,".tag",ntag-strlen(tag)-1);
}

static uint32_t to_tag_tick(gtime_t t, gtime_t t0)
{
    double dt=timediff(t,t0);
    if (dt<0.0) dt=0.0;
    if (dt>4294967.295) dt=4294967.295;
    return (uint32_t)(dt*1000.0+0.5);
}

static int open_outputs(outctx_t *out, const char *rtcm_path, const char *tag_path,
                        gtime_t t0)
{
    char tagh[64]={0};
    uint32_t tick0=0;
    uint32_t time_time;
    double time_sec;

    memset(out,0,sizeof(*out));
    strncpy(out->rtcm_path,rtcm_path,sizeof(out->rtcm_path)-1);
    strncpy(out->tag_path ,tag_path ,sizeof(out->tag_path )-1);
    out->t0=t0;

    if (!(out->fp_rtcm=fopen(out->rtcm_path,"wb"))) {
        fprintf(stderr,"rtcm file open error: %s (%s)\n",out->rtcm_path,strerror(errno));
        return 0;
    }
    if (!(out->fp_tag=fopen(out->tag_path,"wb"))) {
        fprintf(stderr,"tag file open error: %s (%s)\n",out->tag_path,strerror(errno));
        fclose(out->fp_rtcm);
        out->fp_rtcm=NULL;
        return 0;
    }

    snprintf(tagh,sizeof(tagh),"TIMETAG RTKLIB %s",VER_RTKLIB);
    memcpy(tagh+60,&tick0,sizeof(tick0));
    time_time=(uint32_t)t0.time;
    time_sec=t0.sec;

    if (fwrite(tagh,1,sizeof(tagh),out->fp_tag)!=sizeof(tagh)||
        fwrite(&time_time,1,sizeof(time_time),out->fp_tag)!=sizeof(time_time)||
        fwrite(&time_sec,1,sizeof(time_sec),out->fp_tag)!=sizeof(time_sec)) {
        fprintf(stderr,"tag file write error: %s\n",out->tag_path);
        fclose(out->fp_tag );
        fclose(out->fp_rtcm);
        out->fp_tag =NULL;
        out->fp_rtcm=NULL;
        return 0;
    }
    return 1;
}

static void close_outputs(outctx_t *out)
{
    if (out->fp_tag ) fclose(out->fp_tag );
    if (out->fp_rtcm) fclose(out->fp_rtcm);
    out->fp_tag =NULL;
    out->fp_rtcm=NULL;
}

static int write_frame(outctx_t *out, const uint8_t *buff, int nbyte, gtime_t t)
{
    uint32_t tick=to_tag_tick(t,out->t0);
    uint32_t fpos=0;
    long fpos_l;

    if (fwrite(buff,1,nbyte,out->fp_rtcm)!=(size_t)nbyte) {
        fprintf(stderr,"rtcm write error: %s\n",out->rtcm_path);
        return 0;
    }
    fpos_l=ftell(out->fp_rtcm);
    if (fpos_l<0) fpos=0;
    else if ((unsigned long)fpos_l>0xFFFFFFFFUL) fpos=0xFFFFFFFFU;
    else fpos=(uint32_t)fpos_l;

    if (fwrite(&tick,1,sizeof(tick),out->fp_tag)!=sizeof(tick)||
        fwrite(&fpos,1,sizeof(fpos),out->fp_tag)!=sizeof(fpos)) {
        fprintf(stderr,"tag write error: %s\n",out->tag_path);
        return 0;
    }
    return 1;
}

static int has_sys_obs(const obsd_t *data, int n, int sys)
{
    int i;
    for (i=0;i<n;i++) {
        if (satsys(data[i].sat,NULL)==sys) return 1;
    }
    return 0;
}

static void prepare_lock_fallback(rtcm_t *rtcm, obsd_t *obs, double lock_fallback)
{
    int i,sat=obs->sat-1;

    if (sat<0||sat>=MAXSAT) return;

    for (i=0;i<NFREQ+NEXOBS;i++) {
        if (!obs->code[i]) continue;
        if (obs->P[i]==0.0&&obs->L[i]==0.0) continue;

        /* Initialize lock-time origin for first appearance. */
        if (!rtcm->lltime[sat][i].time) {
            rtcm->lltime[sat][i]=timeadd(obs->time,-lock_fallback);
        }
        /* Some RINEX sets LLI slip continuously; keep half-cycle bit only. */
        obs->LLI[i]&=(uint8_t)~1u;
    }
}

static int emit_msm(rtcm_t *rtcm, outctx_t *out, const obsd_t *epoch, int nobs,
                    int msg, int sys, int sync_last, double lock_fallback)
{
    int i,j,nobs0,nsat=0,nsig=0,nmsg=0,ns=0,n=0,generated=0;
    int mask[MAXCODE]={0};
    int code;
    obsd_t *obs0;
    obsd_t buff[MAXOBS];

    for (i=0;i<nobs;i++) {
        if (satsys(epoch[i].sat,NULL)!=sys) continue;
        nsat++;
        for (j=0;j<NFREQ+NEXOBS;j++) {
            code=epoch[i].code[j];
            if (code<=0||code>MAXCODE||mask[code-1]) continue;
            mask[code-1]=1;
            nsig++;
        }
    }
    if (nsat<=0||nsig<=0||nsig>64) return 0;

    ns=64/nsig;
    if (ns<=0) return 0;
    nmsg=(nsat-1)/ns+1;

    obs0=rtcm->obs.data;
    nobs0=rtcm->obs.n;
    rtcm->obs.data=buff;

    for (i=0,j=0;i<nmsg;i++) {
        for (n=0;n<ns&&j<nobs;j++) {
            if (satsys(epoch[j].sat,NULL)!=sys) continue;
            buff[n]=epoch[j];
            prepare_lock_fallback(rtcm,buff+n,lock_fallback);
            n++;
        }
        rtcm->obs.n=n;
        if (n<=0) continue;

        if (!gen_rtcm3(rtcm,msg,0,(i<nmsg-1)?1:sync_last)) continue;
        if (!write_frame(out,rtcm->buff,rtcm->nbyte,rtcm->time)) {
            rtcm->obs.data=obs0;
            rtcm->obs.n=nobs0;
            return -1;
        }
        generated++;
    }

    rtcm->obs.data=obs0;
    rtcm->obs.n=nobs0;
    return generated;
}

static int convert_rinex_to_rtcm(const char *infile, int staid, double lock_fallback,
                                 const char *rtcmopt)
{
    gtime_t ts={0},te={0};
    obs_t obs={0};
    nav_t nav={0};
    sta_t sta={{0}};
    rtcm_t rtcm={0};
    outctx_t out={0};
    gtime_t t_first={0},t_last={0};
    int i,j,ret=0;
    int epochs=0;
    int n1077=0,n1127=0;
    char rtcm_path[MAXSTRPATH];
    char tag_path [MAXSTRPATH];
    char tstr0[40]={0};
    char tstr1[40]={0};

    if (readrnxt(infile,0,ts,te,0.0,"",&obs,&nav,&sta)<=0||obs.n<=0) {
        fprintf(stderr,"failed to read RINEX obs: %s\n",infile);
        goto done;
    }
    sortobs(&obs);
    t_first=obs.data[0].time;
    t_last =obs.data[obs.n-1].time;

    build_output_paths(infile,rtcm_path,sizeof(rtcm_path),tag_path,sizeof(tag_path));
    if (!open_outputs(&out,rtcm_path,tag_path,t_first)) goto done;

    if (!init_rtcm(&rtcm)) {
        fprintf(stderr,"init_rtcm failed\n");
        goto done;
    }
    rtcm.staid=staid;
    rtcm.sta=sta;
    if (rtcmopt&&*rtcmopt) {
        strncpy(rtcm.opt,rtcmopt,sizeof(rtcm.opt)-1);
        rtcm.opt[sizeof(rtcm.opt)-1]='\0';
    }

    for (i=0;i<MAXPRNGLO&&i<rtcm.nav.ng;i++) {
        rtcm.nav.glo_fcn[i]=nav.glo_fcn[i];
    }

    for (i=0;i<obs.n;i=j) {
        int has_gps,has_bds;
        int nsys=0;
        int gps_sync=0;
        int bds_sync=0;
        int n;
        char tstr[40];

        for (j=i+1;j<obs.n;j++) {
            if (fabs(timediff(obs.data[j].time,obs.data[i].time))>EPOCH_GROUP_TOL) break;
        }
        rtcm.time=obs.data[i].time;
        rtcm.seqno=(rtcm.seqno+1)&7;

        has_gps=has_sys_obs(obs.data+i,j-i,SYS_GPS);
        has_bds=has_sys_obs(obs.data+i,j-i,SYS_CMP);
        if (!has_gps&&!has_bds) continue;

        if (has_gps) nsys++;
        if (has_bds) nsys++;
        if (has_gps) gps_sync=(nsys>1)?1:0;
        if (has_bds) bds_sync=0;

        if (has_gps) {
            n=emit_msm(&rtcm,&out,obs.data+i,j-i,1077,SYS_GPS,gps_sync,lock_fallback);
            if (n<0) goto done;
            n1077+=n;
        }
        if (has_bds) {
            n=emit_msm(&rtcm,&out,obs.data+i,j-i,1127,SYS_CMP,bds_sync,lock_fallback);
            if (n<0) goto done;
            n1127+=n;
        }
        epochs++;
        fprintf(stderr,"%s: NOBS=%2d\r",time2str(rtcm.time,tstr,3),j-i);
    }
    fprintf(stderr,"\n");

    fprintf(stderr,"input : %s\n",infile);
    fprintf(stderr,"output: %s\n",rtcm_path);
    fprintf(stderr,"tag   : %s\n",tag_path);
    fprintf(stderr,"start : %s\n",time2str(t_first,tstr0,3));
    fprintf(stderr,"end   : %s\n",time2str(t_last ,tstr1,3));
    fprintf(stderr,"epochs: %d\n",epochs);
    fprintf(stderr,"msg1077: %d\n",n1077);
    fprintf(stderr,"msg1127: %d\n",n1127);

    ret=1;

done:
    close_outputs(&out);
    free_rtcm(&rtcm);
    freeobs(&obs);
    freenav(&nav,0xFF);
    return ret;
}

int main(int argc, char **argv)
{
    int i;
    int staid=DEFAULT_STAID;
    int trlevel=0;
    double lock_min=DEFAULT_LOCK_MIN;
    double lock_sec=-1.0;
    double lock_fallback;
    const char *infile=NULL;
    const char *rtcmopt="";

    for (i=1;i<argc;i++) {
        if (!strcmp(argv[i],"-sta")&&i+1<argc) {
            staid=atoi(argv[++i]);
        }
        else if (!strcmp(argv[i],"-lock")&&i+1<argc) {
            lock_min=atof(argv[++i]);
        }
        else if (!strcmp(argv[i],"-locksec")&&i+1<argc) {
            lock_sec=atof(argv[++i]);
        }
        else if (!strcmp(argv[i],"-opt")&&i+1<argc) {
            rtcmopt=argv[++i];
        }
        else if (!strcmp(argv[i],"-x")&&i+1<argc) {
            trlevel=atoi(argv[++i]);
        }
        else if (!strcmp(argv[i],"-h")||!strcmp(argv[i],"--help")) {
            print_help();
            return 0;
        }
        else if (argv[i][0]=='-') {
            print_help();
            return -1;
        }
        else if (!infile) {
            infile=argv[i];
        }
        else {
            print_help();
            return -1;
        }
    }
    if (!infile) {
        print_help();
        return -1;
    }
    if (staid<=0) staid=DEFAULT_STAID;

    if (lock_sec>=0.0) lock_fallback=lock_sec;
    else lock_fallback=lock_min*60.0;
    if (lock_fallback<=0.0) lock_fallback=DEFAULT_LOCK_MIN*60.0;
    if (lock_fallback<MIN_LOCK_FALLBACK) lock_fallback=MIN_LOCK_FALLBACK;

    if (trlevel>0) {
        traceopen("rnx2rtcm.trace");
        tracelevel(trlevel);
    }

    if (!convert_rinex_to_rtcm(infile,staid,lock_fallback,rtcmopt)) {
        if (trlevel>0) traceclose();
        return -1;
    }
    if (trlevel>0) traceclose();
    return 0;
}
