//-----------------------------------------------------------------------------
// rnx3_to_rtcm3.c : RINEX 3 (OBS) -> RTCM 3 (OBS) converter
//-----------------------------------------------------------------------------
#include "rtklib.h"
#define TRACEFILE "rnx2rtcm.trace"

// --------- 工具说明 ---------
// 本工具读取单个 RINEX 3 观测文件（./input.rnx），按历元生成 RTCM 3 MSM7 观测消息：
//   GPS(1077), GLONASS(1087), Galileo(1097), QZSS(1117), BDS(1127)
// 若输出文件名为空字符串，则把 RTCM3 数据写到 stdout；否则写到 ./output.RTCM3
// 起始时间/结束时间/间隔默认不限制（可在 main 里按需设置）。
// 站点名称设置为 "test"；RTCM 参考站 ID 采用整数 1（RTCM 字段要求整数）。

static void write_rtcm3_msm(FILE *fp, rtcm_t *rtcm, int msg, int sync) {
  int sys;
  if (1071 <= msg && msg <= 1077)
    sys = SYS_GPS;
  else if (1081 <= msg && msg <= 1087)
    sys = SYS_GLO;
  else if (1091 <= msg && msg <= 1097)
    sys = SYS_GAL;
  else if (1101 <= msg && msg <= 1107)
    sys = SYS_SBS;  // 非必须，保留占位
  else if (1111 <= msg && msg <= 1117)
    sys = SYS_QZS;
  else if (1121 <= msg && msg <= 1127)
    sys = SYS_CMP;  // BDS
  else if (1131 <= msg && msg <= 1137)
    sys = SYS_IRN;  // 可忽略
  else
    return;

  // 统计该系统下的卫星与信号集合
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
  if (nsig > 64) return;  // 信号类型数量过多，放弃该历元该系统

  // 当 nsat*nsig > 64 时分页发送
  int ns = 0, nmsg = 1;
  if (nsig > 0) {
    ns = 64 / nsig;              // 单页最大卫星数
    nmsg = (nsat - 1) / ns + 1;  // 需要的页数
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
      if (fwrite(rtcm->buff, rtcm->nbyte, 1, fp) < 1) break;
    }
  }
  // 还原
  rtcm->obs.data = data_save;
  rtcm->obs.n = nobs_save;
}

// 仅生成观测类（MSM 或传统 10xx≤msg≤1012）RTCM3 数据
static void gen_rtcm_obs(rtcm_t *rtcm, const int *type, int n, FILE *fp) {
  // 找到最后一个观测消息的索引，用于设置 sync 位
  int last = -1;
  for (int i = 0; i < n; i++) {
    int msg = type[i];
    if (msg <= 0) continue;
    if (msg <= 1012 || (1071 <= msg && msg <= 1137)) last = i;
  }
  if (last < 0) return;

  // 顺序写出观测消息（仅观测，去除 nav/ant 等）
  for (int i = 0; i < n; i++) {
    int msg = type[i];
    int sync = i != last;
    if (msg <= 0) continue;

    if (msg <= 1012) {
      // 早期非 MSM 观测消息：这里一般不使用，保留兼容
      if (!gen_rtcm3(rtcm, msg, 0, sync)) continue;
      if (fwrite(rtcm->buff, rtcm->nbyte, 1, fp) < 1) break;
    } else {
      // MSM 类消息
      write_rtcm3_msm(fp, rtcm, msg, sync);
    }
  }
}

// 观测到 RTCM
static int conv_rtcm_obs_only(const int *type, int n, const char *rtcmopt, const char *outfile,
                              const obs_t *obs, const nav_t *nav, const sta_t *sta, int staid) {
  rtcm_t rtcm;
  memset(&rtcm, 0, sizeof(rtcm));

  // 可选：设置 rtcm 生成选项（此处留空或按需填写 "staid=..." 等）
  strcpy(rtcm.opt, rtcmopt ? rtcmopt : "");

  // ---- GLONASS 频点信息（MSM 可能用到），从输入 nav 拷贝 glo_fcn ----
  // 这里不输出导航消息，但可把 GLO FCN 复制到 rtcm.nav 以便 gen_rtcm3 使用。
  rtcm.nav.eph = (eph_t *)calloc(MAXSAT * 2, sizeof(eph_t));
  rtcm.nav.geph = (geph_t *)calloc(MAXPRNGLO > 0 ? MAXPRNGLO : 1, sizeof(geph_t));
  rtcm.nav.n = rtcm.nav.nmax = 0;
  rtcm.nav.ng = rtcm.nav.ngmax = (MAXPRNGLO > 0 ? MAXPRNGLO : 1);

  if (rtcm.nav.geph && nav) {
    for (int i = 0; i < MAXPRNGLO; i++) {
      rtcm.nav.glo_fcn[i] = nav->glo_fcn[i];
    }
  }

  // ---- 参考站信息 ----
  rtcm.staid = staid;  // RTCM 需要整数参考站 ID
  rtcm.sta = *sta;     // 名称等信息保留在结构体中

  // ---- 输出介质 ----
  FILE *fp = stdout;
  if (outfile && *outfile) {
#if defined(_WIN32)
    fp = fopen(outfile, "wb");  // Windows 需二进制
#else
    fp = fopen(outfile, "w");
#endif
    if (!fp) {
      fprintf(stderr, "file open error: %s\n", outfile);
      free(rtcm.nav.eph);
      free(rtcm.nav.geph);
      return 0;
    }
  }

  // ---- 历元循环：只写观测 ----
  int ret = 1;
  int j = -1;
  for (int i = 0; i < obs->n; i = j) {
    // 聚合同一历元的观测
    for (j = i + 1; j < obs->n; j++) {
      if (timediff(obs->data[j].time, obs->data[i].time) > DTTOL) break;
    }
    rtcm.time = obs->data[i].time;
    rtcm.seqno++;
    rtcm.obs.data = obs->data + i;
    rtcm.obs.n = j - i;

    // 仅生成观测类消息
    gen_rtcm_obs(&rtcm, type, n, fp);

    // 进度提示
    char tstr[40];
    fprintf(stderr, "%s: NOBS=%2d\r", time2str(rtcm.time, tstr, 0), rtcm.obs.n);
  }
  fprintf(stderr, "\n");

  if (fp && fp != stdout) fclose(fp);

  // 打印生成的各消息计数
  fprintf(stderr, "  MT  # OF MSGS\n");
  for (int i = 1; i < 299; i++) {
    if (!rtcm.nmsg3[i]) continue;
    fprintf(stderr, "%04d %10u\n", 1000 + i, rtcm.nmsg3[i]);
  }
  fprintf(stderr, "\n");

  free(rtcm.nav.eph);
  free(rtcm.nav.geph);
  return ret;
}

int main(void) {
  // =====================参数=====================
  // 若设为空字符串，则直接退出
  const char *infile = "../test/data/rinex/WUHN.25o";
  // 以下参数可选
  const char *outfile = "./output/WUHN.rtcm3";// 若设为空字符串，则输出到 stdout
  const int staid = 1;                        // RTCM 参考站 ID（整数）
  const char *sta_name = "test";              // 站点名称（写入 sta_t 结构体）
  double es[6] = {0};                         // 起始时间（年-月-日-时-分-秒，留 0 则不限制）
  double ee[6] = {0};                         // 结束时间（年-月-日-时-分-秒，留 0 则不限制）
  double tint = 0.0;                          // 采样间隔（秒，留 0 则不限制）
  const char *rnxopt = "";                    // RINEX 读取选项
  const char *rtcmopt = "";                   // RTCM 生成选项
  // 测站坐标 1006, 接收机/天线信息 1033, GPS 1077, GLO 1087, GAL 1097, QZSS 1117, BDS 1127
  int type[] = {1006, 1033,1077, 1087, 1097, 1117,1127};
  int trlevel = 3;                            // 0=关闭；1/2/3=递增
  int m = sizeof(type) / sizeof(type[0]);

  // —— 必要性检查：输入留空则退出 ——
  if (!infile || !*infile) {
    fprintf(stderr, "[ERR] input file is empty. Exit.\n");
    return -1;
  }

  if (trlevel > 0) {
    traceopen(TRACEFILE);
    tracelevel(trlevel);
  }

  // 起止时间
  gtime_t ts = {0}, te = {0};
  if (es[0] > 0.0) ts = epoch2time(es);
  if (ee[0] > 0.0) te = epoch2time(ee);

  // —— 读取 RINEX（仅 v3 观测），并排序 ——
  obs_t obs = {0};
  nav_t nav = {0};  // 读取后仅用于获取 GLO FCN，不输出导航
  sta_t sta = {{0}};

  // 写入站点名称（供需要时使用；RTCM 参考站 ID 使用上面的整数 staid）
  strncpy(sta.name, sta_name, sizeof(sta.name) - 1);

  // readrnxt: 读取 RINEX（支持按时间窗口/采样间隔；此处默认不限制）
  if (readrnxt(infile, 0, ts, te, tint, (char *)rnxopt, &obs, &nav, &sta) <= 0) {
    fprintf(stderr, "[ERR] failed to read RINEX obs: %s\n", infile);
    if (trlevel > 0) traceclose();
    return -1;
  }

  sortobs(&obs);  // 按时间排序观测
  uniqnav(&nav);  // 导航去重/排序（后续仅需其 glo_fcn，保持一致调用）

  // 转换为 RTCM3（仅观测）
  int ret = conv_rtcm_obs_only(type, m, rtcmopt, outfile, &obs, &nav, &sta, staid) ? 0 : -1;

  // 释放
  free(obs.data);
  freenav(&nav, 0xFF);

  if (trlevel > 0) traceclose();
  return ret;
}
