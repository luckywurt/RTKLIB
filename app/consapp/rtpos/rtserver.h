#pragma once

#include <vector>
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include <string>

#include "rtklib.h"

#define PHONE_OBS_BUFF 16   // 内部观测队列最大长度

/**
 * RtServer: 基于 RTKLIB 的实时解算服务（C++封装）
 */
class RtServer {
public:
    RtServer();
    ~RtServer();

    // 初始化 / 释放
    bool init();
    void cleanup();

    // 启动（等价于 rtksvrstart）
    bool start(int cycle, int buffsize, int *strs,
               const char **paths, int *formats, int navsel,
               const char **cmds, const char **cmds_periodic,
               const char **rcvopts, int nmeacycle, int nmeareq,
               const double *nmeapos, prcopt_t *prcopt, solopt_t *solopt,
               stream_t *moni, bool roverInternal, char *errmsg);

    // 停止（等价于 rtksvrstop）
    void stop(const char **cmds);

    // 运行状态
    bool isRunning() const { return m_svr.state == 1; }

    // 向内部队列推送一历元观测（obsd_t 直接喂给解算，避免 RTCM 编码/解码）
    bool pushInternalObs(gtime_t time, const obsd_t *obs, int n);

    // 返回当前解算结果的文本（单行），true 表示有有效结果
    bool getSolutionText(std::string &out);
    // 返回当前服务状态的文本（多行），true 表示有有效状态
    bool getStatusText(std::string &out);
    // 返回当前卫星状态的文本（多行），true 表示有有效状态
    bool getSatelliteStatusText(std::string &out, int nf = 0);
    // 返回当前观测数据状态的文本（多行），true 表示有有效状态
    bool getObservationStatusText(std::string &out, int nf = 0);
    // 返回当前数据流状态的文本（多行），true 表示有有效状态
    bool getStreamStatusText(std::string &out);
    // 返回当前错误消息的文本（多行），true 表示有新错误消息
    bool getErrorStatusText(std::string &out);

    // 与rtksvr.c一致的方法：
    int  rtksvropenstr(int index, int str, const char *path,
                       const solopt_t *solopt, const prcopt_t *prcopt);
    void rtksvrclosestr(int index);
    int  rtksvrostat(int rcv, gtime_t *time, int *sat,
                     double *az, double *el, int **snr, int *vsat);
    void rtksvrsstat(int *sstat, char *msg);
    int  rtksvrmark(const char *name, const char *comment);

private:
    // 线程与内部数据结构
    struct EpochObs {
        gtime_t time{};
        int     n{0};
        obsd_t  data[MAXOBS]{};
    };

    // 解算线程：复刻 rtksvrthread 的主体逻辑
    void solverThread();

    // 从内部队列取一帧观测注入解算线程
    int decodeInternal(rtksvr_t *svr, int index);

    // 复刻自 rtksvr.c 的辅助函数（按需摘取/微调）
    static void writesolhead(stream_t *stream, const solopt_t *solopt, const prcopt_t *prcopt);
    static void saveoutbuf(rtksvr_t *svr, uint8_t *buff, int n, int index);
    static void writesol(rtksvr_t *svr, int index);
    static void update_glofcn(rtksvr_t *svr);
    static void update_obs(rtksvr_t *svr, obs_t *obs, int index, int iobs);
    static void update_eph(rtksvr_t *svr, nav_t *nav, int ephsat, int ephset, int index);
    static void update_sbs(rtksvr_t *svr, sbsmsg_t *sbsmsg, int index);
    static void update_ionutc(rtksvr_t *svr, nav_t *nav, int index);
    static void update_antpos(rtksvr_t *svr, int index);
    static void update_ssr(rtksvr_t *svr, int index);
    static void update_svr(rtksvr_t *svr, int ret, obs_t *obs, nav_t *nav,
                           int ephsat, int ephset, sbsmsg_t *sbsmsg, int index, int iobs);
    static int  decoderaw(rtksvr_t *svr, int index);
    static void decodefile(rtksvr_t *svr, int index);
    static void corr_phase_bias(obsd_t *obs, int n, const nav_t *nav);
    static void periodic_cmd(int cycle, const char *cmd, stream_t *stream);
    static double baseline_len(const rtk_t *rtk);
    static void send_nmea(rtksvr_t *svr, uint32_t *tickreset);

    // RTKLIB server 对象与监控流
    rtksvr_t m_svr{};
    stream_t m_moni{};

    // 内部观测队列
    static bool m_roverInternal;
    std::deque<EpochObs> m_q;
    std::mutex m_qmtx;
};

