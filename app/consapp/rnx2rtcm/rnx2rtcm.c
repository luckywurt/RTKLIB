//-----------------------------------------------------------------------------
// rnx3_to_rtcm3.c : RINEX 3 (OBS) -> RTCM 3 (OBS) converter with time-tag output
//-----------------------------------------------------------------------------
#include "rtklib.h"
#define TRACEFILE "rnx2rtcm.trace"

/* 本文件支持两种输出模式：
 * 1) 实时模式（realtime=1）：通过 stream 文件输出 outfile::T，并在历元间 sleep，生成真实时间间隔的 tag。
 * 2) 快速模式（realtime=0）：直接写 outfile 和 outfile.tag，使用“虚拟 tick”按 1 Hz 递增，快速完成转换。
 */

#define TAG_HEADER_LEN 64 /* 与 stream.c 中 TIMETAGH_LEN 保持一致 */

/* 输出抽象 */
typedef struct {
  int use_stream;        /* 1: 使用 stream 写 outfile::T  0: 直接写文件与 .tag */
  stream_t str;          /* 实时模式的 stream */
  FILE *fp;              /* 快速模式的数据文件 */
  FILE *fp_tag;          /* 快速模式的 tag 文件 */
  uint32_t tick_f;       /* 快速模式：起始 tick（仅写入 header 使用） */
  uint32_t vtick;        /* 快速模式：虚拟 tick，毫秒 */
  gtime_t start_time;    /* 快速模式：起始时间（GPST），写 tag 头用 */
} out_t;

/* 写 tag 文件头（快速模式） */
static int write_tag_header(FILE *fp_tag, uint32_t tick_f, gtime_t gpst_time) {
  if (!fp_tag) return 0;
  char tagh[TAG_HEADER_LEN];
  memset(tagh, 0, sizeof(tagh));
  /* "TIMETAG RTKLIB %s" 并把 tick_f 写入 header 的最后 4 字节 */
  int n = snprintf(tagh, sizeof(tagh), "TIMETAG RTKLIB %s", VER_RTKLIB);
  if (n < 0) return 0;
  memcpy(tagh + TAG_HEADER_LEN - 4, &tick_f, sizeof(tick_f));

  /* header 后紧跟 time.time(uint32) 与 time.sec(double) */
  uint32_t time_time = (uint32_t)gpst_time.time;
  double   time_sec  = gpst_time.sec;

  if (fwrite(tagh, 1, TAG_HEADER_LEN, fp_tag) != TAG_HEADER_LEN) return 0;
  if (fwrite(&time_time, 1, sizeof(time_time), fp_tag) != sizeof(time_time)) return 0;
  if (fwrite(&time_sec , 1, sizeof(time_sec ), fp_tag) != sizeof(time_sec )) return 0;
  /* 之后每条记录为 TICK(4) + FPOS(4)，我们采用 4 字节文件位置，避免播放端必须加 ::P=8 */
  return 1;
}

/* 打开输出（两种模式） */
static int out_open(out_t *out, const char *outfile, int realtime) {
  memset(out, 0, sizeof(*out));
  if (!outfile || !*outfile) return 0;

  if (realtime) {
    /* 实时模式：使用 stream 打开 outfile::T（默认 4B 文件位置） */
    char path_with_tag[MAXSTRPATH];
    snprintf(path_with_tag, sizeof(path_with_tag), "%s::T", outfile);
    strinit(&out->str);
    if (!stropen(&out->str, STR_FILE, STR_MODE_W, path_with_tag)) {
      fprintf(stderr, "file open error (stream): %s\n", outfile);
      return 0;
    }
    out->use_stream = 1;
    return 1;
  } else {
    /* 快速模式：直接打开 outfile 和 outfile.tag，并写 tag 头 */
    out->fp = fopen(outfile, "wb");
    if (!out->fp) {
      fprintf(stderr, "file open error: %s\n", outfile);
      return 0;
    }
    char tagpath[MAXSTRPATH];
    snprintf(tagpath, sizeof(tagpath), "%s.tag", outfile);
    out->fp_tag = fopen(tagpath, "wb");
    if (!out->fp_tag) {
      fprintf(stderr, "tag open error: %s\n", tagpath);
      fclose(out->fp);
      out->fp = NULL;
      return 0;
    }
    out->tick_f = tickget();
    out->start_time = utc2gpst(timeget());
    out->vtick = 0; /* 从 0 ms 开始 */

    if (!write_tag_header(out->fp_tag, out->tick_f, out->start_time)) {
      fprintf(stderr, "write tag header failed\n");
      fclose(out->fp_tag);
      fclose(out->fp);
      out->fp = out->fp_tag = NULL;
      return 0;
    }
    out->use_stream = 0;
    return 1;
  }
}

/* 写消息数据 */
static void out_write(out_t *out, const uint8_t *buff, int n) {
  if (n <= 0) return;
  if (out->use_stream) {
    (void)strwrite(&out->str, (uint8_t *)buff, n);
  } else {
    (void)fwrite(buff, 1, n, out->fp);
  }
}

/* 一个历元写完后的处理：
 * - 实时模式：按 dt_ms sleep，tag 由 stream 自动写。
 * - 快速模式：写一条 tag 记录：TICK(4) + FPOS(4)，然后把 vtick 加上 dt_ms。
 *   注意：本实现每个历元只写一条 tag（聚合历元内所有消息），以历元开始时刻为 tag 时间。
 */
static void out_after_epoch(out_t *out, int dt_ms) {
  if (out->use_stream) {
    if (dt_ms > 0) sleepms(dt_ms);
  } else {
    /* 记录本历元的 tag：tick=当前 vtick，fpos=当前文件位置 */
    uint32_t tick = out->vtick;
    long fpos = ftell(out->fp);
    if (fpos < 0) fpos = 0;
    uint32_t fpos_4B = (uint32_t)fpos;

    (void)fwrite(&tick, 1, sizeof(tick), out->fp_tag);
    (void)fwrite(&fpos_4B, 1, sizeof(fpos_4B), out->fp_tag);
    fflush(out->fp_tag);
    /* 为下一个历元推进虚拟时间 */
    if (dt_ms > 0) out->vtick += (uint32_t)dt_ms;
  }
}

/* 关闭输出 */
static void out_close(out_t *out) {
  if (out->use_stream) {
    strclose(&out->str);
  } else {
    if (out->fp_tag) fclose(out->fp_tag);
    if (out->fp) fclose(out->fp);
    out->fp = out->fp_tag = NULL;
  }
}

/* -------- 下面保持原有的 RTCM 生成逻辑，仅把写出改为 out_write ---------- */

static void write_rtcm3_msm(out_t *out, rtcm_t *rtcm, int msg, int sync) {
  int sys;
  if (1071 <= msg && msg <= 1077) sys = SYS_GPS;
  else if (1081 <= msg && msg <= 1087) sys = SYS_GLO;
  else if (1091 <= msg && msg <= 1097) sys = SYS_GAL;
  else if (1101 <= msg && msg <= 1107) sys = SYS_SBS;
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

  int ns = 0, nmsg = 1;
  if (nsig > 0) {
    ns = 64 / nsig;
    nmsg = (nsat - 1) / ns + 1;
  }

  obsd_t buff[MAXOBS];
  obsd_t *data_save = rtcm->obs.data;
  int nobs_save = rtcm->obs.n;

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
  rtcm->obs.n = nobs_save;
}

static void gen_rtcm_obs(rtcm_t *rtcm, const int *type, int n, out_t *out) {
  int last = -1;
  for (int i = 0; i < n; i++) {
    int msg = type[i];
    if (msg <= 0) continue;
    if (msg <= 1012 || (1071 <= msg && msg <= 1137)) last = i;
  }
  if (last < 0) return;

  for (int i = 0; i < n; i++) {
    int msg = type[i];
    int sync = i != last;
    if (msg <= 0) continue;

    if (msg <= 1012) {
      if (!gen_rtcm3(rtcm, msg, 0, sync)) continue;
      out_write(out, rtcm->buff, rtcm->nbyte);
    } else {
      write_rtcm3_msm(out, rtcm, msg, sync);
    }
  }
}

/* 观测到 RTCM：输出到 out（实时或快速），快速模式下使用虚拟 tick 写 tag */
static int conv_rtcm_obs_only(const int *type, int n, const char *rtcmopt, const char *outfile,
                              const obs_t *obs, const nav_t *nav, const sta_t *sta, int staid,
                              int realtime) {
  if (!outfile || !*outfile) {
    fprintf(stderr, "[ERR] outfile must be set.\n");
    return 0;
  }

  out_t out;
  if (!out_open(&out, outfile, realtime)) return 0;

  rtcm_t rtcm;
  memset(&rtcm, 0, sizeof(rtcm));
  strcpy(rtcm.opt, rtcmopt ? rtcmopt : "");

  rtcm.nav.eph  = (eph_t  *)calloc(MAXSAT * 2, sizeof(eph_t));
  rtcm.nav.geph = (geph_t *)calloc(MAXPRNGLO > 0 ? MAXPRNGLO : 1, sizeof(geph_t));
  rtcm.nav.n = rtcm.nav.nmax = 0;
  rtcm.nav.ng = rtcm.nav.ngmax = (MAXPRNGLO > 0 ? MAXPRNGLO : 1);

  if (rtcm.nav.geph && nav) {
    for (int i = 0; i < MAXPRNGLO; i++) rtcm.nav.glo_fcn[i] = nav->glo_fcn[i];
  }

  rtcm.staid = staid;
  rtcm.sta = *sta;

  int ret = 1;
  int j = -1;
  for (int i = 0; i < obs->n; i = j) {
    for (j = i + 1; j < obs->n; j++) {
      if (timediff(obs->data[j].time, obs->data[i].time) > DTTOL) break;
    }
    rtcm.time = obs->data[i].time;
    rtcm.seqno++;
    rtcm.obs.data = obs->data + i;
    rtcm.obs.n = j - i;

    gen_rtcm_obs(&rtcm, type, n, &out);

    char tstr[40];
    fprintf(stderr, "%s: NOBS=%2d\r", time2str(rtcm.time, tstr, 0), rtcm.obs.n);

    /* 计算到下一历元的间隔（ms）：
     * - 实时模式：sleep 该时长，stream 自动写 tag
     * - 快速模式：写一条 tag 并推进虚拟 tick，无需 sleep
     */
    int dt_ms = 0;
    if (j < obs->n) {
      double dt = timediff(obs->data[j].time, obs->data[i].time); /* seconds */
      if (dt <= 0.0) dt = 1.0;
      dt_ms = (int)(dt * 1000.0 + 0.5);
    }
    out_after_epoch(&out, dt_ms);
  }
  fprintf(stderr, "\n");

  fprintf(stderr, "  MT  # OF MSGS\n");
  for (int i = 1; i < 299; i++) {
    if (!rtcm.nmsg3[i]) continue;
    fprintf(stderr, "%04d %10u\n", 1000 + i, rtcm.nmsg3[i]);
  }
  fprintf(stderr, "\n");

  out_close(&out);
  free(rtcm.nav.eph);
  free(rtcm.nav.geph);
  return ret;
}

int main(void) {
  /* 参数 */
  const char *infile = "../test/data/rinex/XZDJ.25o";
  const char *outfile = "./output/XZDJ.rtcm3";
  const int staid = 1;
  const char *sta_name = "test";
  double es[6] = {0}, ee[6] = {0};
  double tint = 0.0;
  const char *rnxopt = "";
  const char *rtcmopt = "";
  /* 常用：1006, GPS 1077, GLO 1087, GAL 1097, QZSS 1117, BDS 1127 */
  int type[] = {1006, 1077, 1087, 1097, 1117, 1127};
  int trlevel = 3;
  int m = (int)(sizeof(type) / sizeof(type[0]));

  /* 选择模式：0=快速生成（推荐），1=实时按历元 sleep */
  const int realtime = 1;

  if (!infile || !*infile) {
    fprintf(stderr, "[ERR] input file is empty. Exit.\n");
    return -1;
  }

  if (trlevel > 0) { traceopen(TRACEFILE); tracelevel(trlevel); }

  gtime_t ts = {0}, te = {0};
  if (es[0] > 0.0) ts = epoch2time(es);
  if (ee[0] > 0.0) te = epoch2time(ee);

  obs_t obs = {0};
  nav_t nav = {0};
  sta_t sta = {{0}};
  strncpy(sta.name, sta_name, sizeof(sta.name) - 1);

  if (readrnxt(infile, 0, ts, te, tint, (char *)rnxopt, &obs, &nav, &sta) <= 0) {
    fprintf(stderr, "[ERR] failed to read RINEX obs: %s\n", infile);
    if (trlevel > 0) traceclose();
    return -1;
  }

  sortobs(&obs);
  uniqnav(&nav);

  int ret = conv_rtcm_obs_only(type, m, rtcmopt, outfile, &obs, &nav, &sta, staid, realtime) ? 0 : -1;

  free(obs.data);
  freenav(&nav, 0xFF);

  if (trlevel > 0) traceclose();
  return ret;
}