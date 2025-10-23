/*-----------------------------------------------------------------------------
* rnx2str.c : RINEX(OBS) -> RTCM3 实时输出（文件::T + TCP服务端，均可开关）
*----------------------------------------------------------------------------*/
#include "rtklib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ======================== 可按需修改的参数（集中处） ======================== */
static const char *CFG_INFILE   = "../test/data/rinex/XZDJ.25o";   /* 输入 RINEX 观测文件路径（必填） */
static const char *CFG_STA_NAME = "test";               /* 站名（RTCM中可见），<=4或<=16均可 */
static const int   CFG_STA_ID   = 1;                    /* RTCM station ID（整数，>0）      */

/* RTCM 消息集合（仅观测 + 1006站心），四大系统：GPS/GLO/GAL/BDS
   - 1006: 参考站ARP坐标/天线高信息（本实现仅填入站名；如需坐标，可扩展 rtcm.sta）
   - 1077/1087/1097/1127: 四系统 MSM7 观测
   如需精简，可将不需的消息设为 0 即可。*/
static int CFG_RTCM_TYPES[] = {1006, 1077, 1087, 1097, 1127};
static const int CFG_RTCM_TYPES_N =
    (int)(sizeof(CFG_RTCM_TYPES) / sizeof(CFG_RTCM_TYPES[0]));

/* 时间裁剪与采样间隔（均可留默认）
   - 若都为 0 表示全读；tint=0 表示不重采样 */
static const double CFG_TS[6] = {0};    /* 起始时间 [y,m,d,H,M,S] */
static const double CFG_TE[6] = {0};    /* 结束时间 [y,m,d,H,M,S] */
static const double CFG_TINT  = 0.0;    /* 目标采样间隔（秒），0=按文件原始 */

static const char *CFG_RNXOPT  = "";    /* RINEX 读取选项（一般留空） */
static const char *CFG_RTCMOPT = "";    /* RTCM 生成选项（一般留空） */

/* 路1：文件输出（实时 + 自动 .tag）
   - 置为 "" 或 NULL 可关闭文件输出
   - 注意必须带 ::T 才会生成 .tag 文件（RTKLIB 机制） */
static const char *CFG_OUT_FILE_TAGGED = "./output/XZDJ.rtcm3::T";

/* 路2：TCP 服务端输出（解算端连上即可收流）
   - 置为 0 可关闭 TCP 输出；示例端口 2101 */
static const int   CFG_TCP_SVR_PORT = 2101;

/* 日志/调试输出（RTKLIB trace）
   - 0=关闭；>0 可输出到 rnx2str.trace */
static const int   CFG_TRACE_LEVEL  = 2;
/* ========================================================================== */

/* ---- 多路输出封装 --------------------------------------------------------- */
typedef struct {
    int use_file;           /* 是否启用文件输出 (::T 自动产 .tag) */
    int use_tcp;            /* 是否启用 TCP 服务端输出            */
    stream_t file_str;      /* 文件流（STR_FILE）                  */
    stream_t tcp_str;       /* TCP 服务器（STR_TCPSVR）            */
} out_multi_t;

/* 打开双路输出：文件::T / tcpsvr://:port */
static int out_open(out_multi_t *out, const char *file_tagged, int tcp_port) {
    memset(out, 0, sizeof(*out));
    strinit(&out->file_str);
    strinit(&out->tcp_str);

    /* 文件输出 */
    if (file_tagged && *file_tagged) {
        if (!stropen(&out->file_str, STR_FILE, STR_MODE_W, file_tagged)) {
            fprintf(stderr, "[ERR] open file stream failed: %s\n", file_tagged);
            return 0;
        }
        out->use_file = 1;
    }

    /* TCP 输出：tcpsvr://:port */
    if (tcp_port > 0) {
        char path[32];
        snprintf(path, sizeof(path), ":%d", tcp_port);
        if (!stropen(&out->tcp_str, STR_TCPSVR, STR_MODE_W, path)) {
            fprintf(stderr, "[ERR] open tcp server failed: %s\n", path);
            /* 如果你希望“文件成功、TCP失败”时仍可继续，可不返回；此处保持严格 */
            if (out->use_file) strclose(&out->file_str);
            return 0;
        }
        out->use_tcp = 1;
    }

    if (!out->use_file && !out->use_tcp) {
        fprintf(stderr, "[ERR] both outputs are disabled.\n");
        return 0;
    }
    return 1;
}

/* 写二进制数据到已启用的目标（文件/TCP） */
static void out_write(out_multi_t *out, const uint8_t *buff, int n) {
    if (n <= 0 || (!out->use_file && !out->use_tcp)) return;
    if (out->use_file) (void)strwrite(&out->file_str, (uint8_t *)buff, n);
    if (out->use_tcp ) (void)strwrite(&out->tcp_str , (uint8_t *)buff, n);
}

/* 历元结束后：按 dt_ms 睡眠（实时节拍）；file::T 的 tag 由 RTKLIB 内部自动写 */
static void out_after_epoch(out_multi_t *out, int dt_ms) {
    (void)out;
    if (dt_ms > 0) sleepms(dt_ms);
}

/* 关闭输出 */
static void out_close(out_multi_t *out) {
    if (out->use_file) strclose(&out->file_str);
    if (out->use_tcp ) strclose(&out->tcp_str );
    memset(out, 0, sizeof(*out));
}

/* ---- RTCM 生成（观测 + MSM 分包），改造自 rnx2rtcm 思路 ------------------ */
/* 将单条 MSM7 消息按卫星/信号数分段（防止过长），并写入 out */
static void write_rtcm3_msm(out_multi_t *out, rtcm_t *rtcm, int msg, int sync) {
    int sys;
    if      (1071 <= msg && msg <= 1077) sys = SYS_GPS;
    else if (1081 <= msg && msg <= 1087) sys = SYS_GLO;
    else if (1091 <= msg && msg <= 1097) sys = SYS_GAL;
    else if (1111 <= msg && msg <= 1117) sys = SYS_QZS;
    else if (1121 <= msg && msg <= 1127) sys = SYS_CMP;
    else if (1131 <= msg && msg <= 1137) sys = SYS_IRN;
    else return;

    obsd_t *data = rtcm->obs.data;
    int nobs = rtcm->obs.n, nsat = 0, nsig = 0, mask[MAXCODE] = {0};
    for (int i = 0; i < nobs && i < MAXOBS; i++) {
        if (satsys(data[i].sat, NULL) != sys) continue;
        nsat++;
        for (int j = 0; j < NFREQ + NEXOBS; j++) {
            int code = data[i].code[j];
            if (!code || mask[code - 1]) continue;
            mask[code - 1] = 1;
            nsig++;
        }
    }
    if (nsig > 64) return;

    /* 最多 64 bit 信号掩码：按 ns=64/nsig 个卫星一包 */
    int ns = 0, nmsg = 1;
    if (nsig > 0) {
        ns   = 64 / nsig;
        nmsg = (nsat - 1) / ns + 1;
    }

    obsd_t buff[MAXOBS];
    obsd_t *data_save = rtcm->obs.data;
    int     nobs_save = rtcm->obs.n;

    rtcm->obs.data = buff;

    for (int i = 0, j = 0; i < nmsg; i++) {
        int n = 0;
        for (; n < ns && j < nobs && j < MAXOBS; j++) {
            if (satsys(data_save[j].sat, NULL) != sys) continue;
            rtcm->obs.data[n++] = data_save[j];
        }
        rtcm->obs.n = n;

        if (gen_rtcm3(rtcm, msg, 0, (i < nmsg - 1) || sync)) {
            out_write(out, rtcm->buff, rtcm->nbyte);
        }
    }
    rtcm->obs.data = data_save;
    rtcm->obs.n    = nobs_save;
}

/* 一组（同历元）观测 -> 多条 RTCM 消息（含 1006 + 各系统 MSM7） */
static void gen_rtcm_obs(out_multi_t *out, rtcm_t *rtcm, const int *type, int n) {
    int last = -1;
    for (int i = 0; i < n; i++) {            /* 找到“同步位”最后一条 */
        int msg = type[i];
        if (msg <= 0) continue;
        if (msg <= 1012 || (1071 <= msg && msg <= 1137)) last = i;
    }
    if (last < 0) return;

    for (int i = 0; i < n; i++) {
        int msg = type[i];
        int sync = i != last;                /* 最后一条 sync=0，其余 sync=1 */
        if (msg <= 0) continue;

        if (msg <= 1012) {                   /* 1006 等旧体制 */
            if (!gen_rtcm3(rtcm, msg, 0, sync)) continue;
            out_write(out, rtcm->buff, rtcm->nbyte);
        } else {                             /* MSM */
            write_rtcm3_msm(out, rtcm, msg, sync);
        }
    }
}

/* 整体：观测 -> RTCM，逐历元实时写出（按真实间隔 sleep） */
static int conv_rtcm_obs_realtime(const int *type, int n, const char *rtcmopt,
                                  const obs_t *obs, const nav_t *nav,
                                  const sta_t *sta, int staid,
                                  out_multi_t *out) {
    rtcm_t rtcm;
    memset(&rtcm, 0, sizeof(rtcm));
    strcpy(rtcm.opt, rtcmopt ? rtcmopt : "");

    /* 必需的导航缓冲（MSM 需要 GLO FCN 等） */
    rtcm.nav.eph  = (eph_t  *)calloc(MAXSAT * 2, sizeof(eph_t));
    rtcm.nav.geph = (geph_t *)calloc(MAXPRNGLO > 0 ? MAXPRNGLO : 1, sizeof(geph_t));
    rtcm.nav.n = rtcm.nav.nmax = 0;
    rtcm.nav.ng = rtcm.nav.ngmax = (MAXPRNGLO > 0 ? MAXPRNGLO : 1);
    if (rtcm.nav.geph && nav) {
        for (int i = 0; i < MAXPRNGLO; i++) rtcm.nav.glo_fcn[i] = nav->glo_fcn[i];
    }

    /* 站信息（若需填站心坐标/天线偏心，可补充 rtcm.sta 成员） */
    rtcm.staid = staid;
    rtcm.sta   = *sta;

    /* 逐历元写出 */
    int ret = 1;
    int j = -1;
    for (int i = 0; i < obs->n; i = j) {
        for (j = i + 1; j < obs->n; j++) {
            if (timediff(obs->data[j].time, obs->data[i].time) > DTTOL) break;
        }
        rtcm.time     = obs->data[i].time;
        rtcm.seqno++;                 /* 递增序号 */
        rtcm.obs.data = obs->data + i;
        rtcm.obs.n    = j - i;

        /* 本历元：按配置消息集合写多条 RTCM */
        gen_rtcm_obs(out, &rtcm, type, n);

        /* 打印一点进度 */
        char tstr[40];
        fprintf(stderr, "%s  NOBS=%2d\r", time2str(rtcm.time, tstr, 0), rtcm.obs.n);

        /* 计算下一历元时间间隔（毫秒），实时 sleep，同步 file::T 的 tag */
        int dt_ms = 0;
        if (j < obs->n) {
            double dt = timediff(obs->data[j].time, obs->data[i].time); /* seconds */
            if (dt <= 0.0) dt = 1.0;
            dt_ms = (int)(dt * 1000.0 + 0.5);
        }
        out_after_epoch(out, dt_ms);
    }
    fprintf(stderr, "\n");

    /* 输出消息统计（可选） */
    fprintf(stderr, "  MT  # OF MSGS\n");
    for (int i = 1; i < 299; i++) {
        if (!rtcm.nmsg3[i]) continue;
        fprintf(stderr, "%04d %10u\n", 1000 + i, rtcm.nmsg3[i]);
    }

    free(rtcm.nav.eph);
    free(rtcm.nav.geph);
    return ret;
}

/* ---- 主流程 --------------------------------------------------------------- */
int main(void) {
    /* 1) 可选：打开 RTKLIB trace（便于调试问题） */
    if (CFG_TRACE_LEVEL > 0) {
        traceopen("rnx2str.trace");
        tracelevel(CFG_TRACE_LEVEL);
    }

    /* 2) 读取 RINEX（按需裁剪） */
    if (!CFG_INFILE || !*CFG_INFILE) {
        fprintf(stderr, "[ERR] input RINEX is empty.\n");
        if (CFG_TRACE_LEVEL > 0) traceclose();
        return -1;
    }
    gtime_t ts = {0}, te = {0};
    if (CFG_TS[0] > 0.0) ts = epoch2time(CFG_TS);
    if (CFG_TE[0] > 0.0) te = epoch2time(CFG_TE);

    obs_t obs = {0};
    nav_t nav = {0};
    sta_t sta = {{0}};
    strncpy(sta.name, CFG_STA_NAME, sizeof(sta.name) - 1);

    if (readrnxt(CFG_INFILE, 0, ts, te, CFG_TINT, (char *)CFG_RNXOPT, &obs, &nav, &sta) <= 0) {
        fprintf(stderr, "[ERR] failed to read RINEX obs: %s\n", CFG_INFILE);
        if (CFG_TRACE_LEVEL > 0) traceclose();
        return -1;
    }
    sortobs(&obs);
    uniqnav(&nav);

    /* 3) 打开双路实时输出（文件::T / TCP） */
    out_multi_t out = {0};
    const char *file_tagged = CFG_OUT_FILE_TAGGED && *CFG_OUT_FILE_TAGGED ? CFG_OUT_FILE_TAGGED : NULL;
    int tcp_port = CFG_TCP_SVR_PORT > 0 ? CFG_TCP_SVR_PORT : 0;

    if (!out_open(&out, file_tagged, tcp_port)) {
        fprintf(stderr, "[ERR] open outputs failed.\n");
        free(obs.data);
        freenav(&nav, 0xFF);
        if (CFG_TRACE_LEVEL > 0) traceclose();
        return -1;
    }

    /* 4) 逐历元生成并实时输出 RTCM3 */
    (void)conv_rtcm_obs_realtime(CFG_RTCM_TYPES, CFG_RTCM_TYPES_N, CFG_RTCMOPT,
                                 &obs, &nav, &sta, CFG_STA_ID, &out);

    /* 5) 清理与关闭 */
    out_close(&out);
    free(obs.data);
    freenav(&nav, 0xFF);

    if (CFG_TRACE_LEVEL > 0) traceclose();
    return 0;
}
