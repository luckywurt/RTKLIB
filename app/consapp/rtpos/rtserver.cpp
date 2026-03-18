#include <algorithm>
#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <sstream>
#include <iomanip>

#include "rtserver.h"
#include "rtklib.h"

#define SQRT(x)     ((x)<=0.0||(x)!=(x)?0.0:sqrt(x))

// 辅助函数：格式化并追加字符串
static void appendFmt(std::string& out, const char* format, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    out += buffer;
}

// 构造/析构/初始化/释放
RtServer::RtServer() { }
RtServer::~RtServer() { cleanup(); }

bool RtServer::m_roverInternal = false;
bool RtServer::init() {
    // 等价 rtksvrinit()，并初始化监控流
    if (!rtksvrinit(&m_svr)) return false;
    strinit(&m_moni);
    return true;
}

void RtServer::cleanup() {
    if (isRunning()) return; // 运行中不释放
    rtksvrfree(&m_svr);
    strclose(&m_moni);
    m_q.clear();
}

// 启动/停止
bool RtServer::start(int cycle, int buffsize, int *strs,
                     const char **paths, int *formats, int navsel,
                     const char **cmds, const char **cmds_periodic,
                     const char **rcvopts, int nmeacycle, int nmeareq,
                     const double *nmeapos, prcopt_t *prcopt, solopt_t *solopt,
                     stream_t *moni, bool roverInternal, char *errmsg) {
    if (isRunning()) { if (errmsg) sprintf(errmsg,"server already started"); return false; }
    m_roverInternal = roverInternal;

    // 复刻 rtksvrstart()逻辑
    gtime_t time,time0={0};
    int i,j,rw;

    strinitcom();
    m_svr.cycle     = cycle>1?cycle:1;
    m_svr.nmeacycle = nmeacycle>1000?nmeacycle:1000;
    m_svr.nmeareq   = nmeareq;
    for (i=0;i<3;i++) m_svr.nmeapos[i]=nmeapos[i];
    m_svr.buffsize  = buffsize>4096?buffsize:4096;
    for (i=0;i<3;i++) m_svr.format[i]=formats[i];
    m_svr.navsel    = navsel;
    m_svr.nsbs = 0;
    m_svr.nsol = 0;
    m_svr.prcout = 0;

    rtkfree(&m_svr.rtk);
    rtkinit(&m_svr.rtk, prcopt);

    if (prcopt->initrst) {
        m_svr.nave = 0;
        for (i=0;i<3;i++) m_svr.rb_ave[i]=0.0;
    }

    // 输入/peek 缓冲
    for (i=0;i<3;i++) {
        m_svr.nb[i]=m_svr.npb[i]=0;

        if (!(m_svr.buff[i]=(uint8_t *)malloc(buffsize))||
            !(m_svr.pbuf[i]=(uint8_t *)malloc(buffsize))) {
            tracet(1,"rtksvrstart: malloc error\n");

            return 0;
        }

        for (j=0;j<10;j++) m_svr.nmsg[i][j]=0;
        for (j=0;j<MAXOBSBUF;j++) m_svr.obs[i][j].n=0;

        if (cmds_periodic && cmds_periodic[i]) {
            strncpy(m_svr.cmds_periodic[i], cmds_periodic[i], MAXRCVCMD-1);
            m_svr.cmds_periodic[i][MAXRCVCMD-1]='\0';
        } else {
            *m_svr.cmds_periodic[i]='\0';
        }

        // 三路都 init_raw/init_rtcm 设置 rcvopts/dgps
        init_raw (m_svr.raw +i, m_svr.format[i]);
        init_rtcm(m_svr.rtcm+i);
        if (rcvopts) {
            if (rcvopts[i]) strcpy(m_svr.raw [i].opt, rcvopts[i]);
            if (rcvopts[i]) strcpy(m_svr.rtcm[i].opt, rcvopts[i]);
        }
        m_svr.rtcm[i].dgps = m_svr.nav.dgps;
    }

    // 输出 peek 缓冲
    for (i=0;i<2;i++) {
        m_svr.sbuf[i]=(uint8_t*)malloc(buffsize);
        if (!m_svr.sbuf[i]) { tracet(1,"rtksvrstart: malloc error\n"); return false; }
    }
    // solution 选项
    m_svr.solopt[0]=solopt[0];
    m_svr.solopt[1]=solopt[1];

    // 基站坐标
    if (prcopt->refpos!=POSOPT_SINGLE) {
        for (i=0;i<6;i++) m_svr.rtk.rb[i]= i<3?prcopt->rb[i]:0.0;
    }

    // 导航数据清零时间戳
    for (i=0;i<MAXSAT*4 ;i++) m_svr.nav.eph [i].ttr=time0;
    for (i=0;i<MAXPRNGLO*2;i++) m_svr.nav.geph[i].tof=time0;
    for (i=0;i<NSATSBS*2;i++) m_svr.nav.seph[i].tof=time0;

    // 监控流
    m_svr.moni = moni;

    // 打开输入/输出流：index=0 (rover) 在“内部模式”下不打开
    for (i=0;i<8;i++) {
        if (m_roverInternal && i==0) { m_svr.stream[i].state=0; continue; }
        rw = i<3 ? STR_MODE_R : STR_MODE_W;
        if (strs[i] != STR_FILE) rw |= STR_MODE_W;
        if (!stropen(m_svr.stream+i, strs[i], rw, paths[i])) {
            if (errmsg) { sprintf(errmsg,"str%d open error path=%s", i+1, paths[i]); }
            for (i--; i>=0; i--) if (!(m_roverInternal && i==0)) strclose(m_svr.stream+i);
            return false;
        }
        if (i<3) {
            time = utc2gpst(timeget());
            m_svr.raw [i].time = (strs[i]==STR_FILE) ? strgettime(m_svr.stream+i) : time;
            m_svr.rtcm[i].time = (strs[i]==STR_FILE) ? strgettime(m_svr.stream+i) : time;
        }
    }

    // 同步流（仅外部 rover）
    if (!m_roverInternal) {
        strsync(m_svr.stream,   m_svr.stream+1);
        strsync(m_svr.stream,   m_svr.stream+2);
    }

    // 输入启动命令（仅外部流）
    for (i=0;i<3;i++) {
        if (m_roverInternal && i==0) continue;
        if (!cmds || !cmds[i]) continue;
        strwrite(m_svr.stream+i,(unsigned char *)"",0);
        sleepms(100);
        strsendcmd(m_svr.stream+i,cmds[i]);
    }

    // 输出头
    for (i=3;i<5;i++) writesolhead(m_svr.stream+i, m_svr.solopt+(i-3), prcopt);

    // 启动线程，直接使用 pthread，因为这是 Android 项目
    if (pthread_create(&m_svr.thread,NULL,
                       [](void* arg)->void*{ ((RtServer*)arg)->solverThread(); return nullptr; },
                       this)) {
        for (i=0;i<MAXSTRRTK;i++) if (!(m_roverInternal && i==0)) strclose(m_svr.stream+i);
        if (errmsg) sprintf(errmsg,"thread create error");
        return false;
    }
    return true;
}

void RtServer::stop(const char **cmds) {
    // 发送停止命令
    rtksvrlock(&m_svr);
    for (int i=0;i<3;i++) {
        if (m_roverInternal && i==0) continue;
        if (cmds && cmds[i]) strsendcmd(m_svr.stream+i,cmds[i]);
    }
    rtksvrunlock(&m_svr);

    m_svr.state = 0;
    pthread_join(m_svr.thread,nullptr);
    // 内部队列清空
    std::lock_guard<std::mutex> lk(m_qmtx);
    m_q.clear();
}

// 向观测队列注入观测
bool RtServer::pushInternalObs(gtime_t time, const obsd_t *obs, int n) {
    if (!m_roverInternal || !isRunning() || !obs || n<=0) return false;
    EpochObs epoch{};
    epoch.time = time;
    epoch.n = n > MAXOBS ? MAXOBS : n;
    for (int i=0; i < epoch.n; i++) epoch.data[i] = obs[i];
    // 排序保证 sat 升序
    std::sort(epoch.data, epoch.data + epoch.n, [](const obsd_t&a, const obsd_t&b){return a.sat < b.sat;});

    std::lock_guard<std::mutex> lk(m_qmtx);
    if ((int)m_q.size() >= PHONE_OBS_BUFF) m_q.pop_front(); // 丢最旧
    m_q.push_back(epoch);
    return true;
}

// 获取当前解算结果的文本表示（单行）
bool RtServer::getSolutionText(std::string &out) {
    out.clear();

    // 拷贝当前解算结果和输出选项，避免长时间持锁
    sol_t   sol{};
    solopt_t solopt0{};
    rtksvrlock(&m_svr);
    sol     = m_svr.rtk.sol;
    solopt0 = m_svr.solopt[0];
    rtksvrunlock(&m_svr);

    if (sol.time.time == 0 || sol.stat == SOLQ_NONE) {
        return false;
    }

    char tstr[64];
    time2str(sol.time, tstr, 2);

    static const char* kStatusStr[] = {"------","FIX","FLOAT","SBAS","DGPS","SINGLE","PPP",""};
    const char *qstr = kStatusStr[sol.stat];

    double pos[3] = {0}, Qr[9], Qe[9] = {0};

    Qr[0] = sol.qr[0];
    Qr[4] = sol.qr[1];
    Qr[8] = sol.qr[2];
    Qr[1] = Qr[3] = sol.qr[3];
    Qr[5] = Qr[7] = sol.qr[4];
    Qr[2] = Qr[6] = sol.qr[5];

    // ECEF -> 经纬度 + 协方差转换到 ENU
    if (norm(sol.rr, 3) > 0.0) {
        ecef2pos(sol.rr, pos);
        covenu(pos, Qr, Qe);
        // 高度模式为大地高时，需要减去大地水准面
        if (solopt0.height == 1) {
            pos[2] -= geoidh(pos);
        }
    }

    appendFmt(out, "%s Mode:%s\n", tstr, qstr);
    appendFmt(out, "Lat:%.8f Lon:%.8f H:%.3f\n", pos[0] * R2D, pos[1] * R2D, pos[2]);
    appendFmt(out, "σE:%.3f σN:%.3f σU:%.3f\n", SQRT(Qe[0]), SQRT(Qe[4]), SQRT(Qe[8]));
    appendFmt(out, "A:%.1f R:%.1f Ns:%d", sol.age, sol.ratio,  sol.ns);
    return true;
}

// 获取当前服务状态文本（多行）
bool RtServer::getStatusText(std::string &out) {
    out.clear();
    // 状态定义常量
    const char *svrstate[]={"stop","run"};
    const char *type[]={"rover","base","corr"};
    const char *sol[]={"-","fix","float","SBAS","DGPS","single","PPP",""};
    const char *mode[]={
            "single","DGPS","kinematic","static","static-start","moving-base","fixed",
            "PPP-kinema","PPP-static"
    };
    const char *freq[]={"-","L1","L1+L2","L1+L2+E5b","L1+L2+E5b+L5","5","6","7"};

    rtksvr_t *svr = &m_svr;
    rtk_t *rtk = nullptr;
    rtcm_t *rtcm = nullptr;

    // 临时变量定义
    gtime_t eventime={0};
    int i,j,n,cycle,state,rtkstat,nsat0,nsat1,prcout,rcvcount,tmcount,timevalid,nave;
    int cputime,nb[3]={0};
    uint32_t nmsg[3][10]={{0}};
    char tstr[64],tmstr[64],s[1024],*p;
    double runtime,rt[3]={0},dop[4]={0},rr[3],bl1=0.0,bl2=0.0;
    double azel[MAXSAT*2],pos[3],vel[3],*del;

    // 分配临时 rtk和 rtcm 结构体，防止栈溢出，并检查分配情况
    rtk = (rtk_t *)calloc(1, sizeof(rtk_t));
    rtcm = (rtcm_t *)calloc(3, sizeof(rtcm_t));
    if (!rtk || !rtcm) {
        if (rtk) std::free(rtk);
        if (rtcm) std::free(rtcm);
        return false;
    }

    // 加锁复制数据
    rtksvrlock(svr);

    *rtk = svr->rtk;
    cycle = svr->cycle;
    state = svr->state;
    rtkstat = svr->rtk.sol.stat;
    nsat0 = svr->obs[0][0].n;
    nsat1 = svr->obs[1][0].n;
    rcvcount = svr->raw[0].obs.rcvcount;
    tmcount = svr->raw[0].obs.tmcount;

    cputime = svr->cputime;
    prcout = svr->prcout;
    nave = svr->nave;

    for (i=0;i<3;i++) nb[i] = svr->nb[i];
    for (i=0;i<3;i++) for (j=0;j<10;j++) {
            nmsg[i][j] = svr->nmsg[i][j];
        }

    // 计算运行时间
    if (svr->state) {
        runtime = (double)(tickget() - svr->tick)/1000.0;
        rt[0] = floor(runtime/3600.0); runtime -= rt[0]*3600.0;
        rt[1] = floor(runtime/60.0); rt[2] = runtime - rt[1]*60.0;
    }

    for (i=0;i<3;i++) rtcm[i] = svr->rtcm[i];
    // 获取时间标记
    if (svr->raw[0].obs.data != NULL && svr->raw[0].obs.n > 0) {
        timevalid = svr->raw[0].obs.data[0].timevalid;
        eventime = svr->raw[0].obs.data[0].eventime;
    }
    time2str(eventime, tmstr, 9);

    rtksvrunlock(svr);
    // 解锁完成

    // 2. 数据处理 (计算 DOP 等)
    for (i=n=0;i<MAXSAT;i++) {
        if (rtk->opt.mode == PMODE_SINGLE && !rtk->ssat[i].vs) continue;
        if (rtk->opt.mode != PMODE_SINGLE && !rtk->ssat[i].vsat[0]) continue;
        azel[  n*2] = rtk->ssat[i].azel[0];
        azel[1+n*2] = rtk->ssat[i].azel[1];
        n++;
    }
    dops(n, azel, 0.0, dop);

    // 3. 格式化输出字符串
    appendFmt(out,"%-28s: %s\n", "rtk server state", svrstate[state]);
    appendFmt(out,"%-28s: %d\n", "processing cycle (ms)", cycle);
    appendFmt(out,"%-28s: %s\n", "positioning mode", mode[rtk->opt.mode]);
    appendFmt(out,"%-28s: %s\n", "frequencies", freq[rtk->opt.nf]);
    appendFmt(out,"%-28s: %02.0f:%02.0f:%04.1f\n", "accumulated time to run", rt[0], rt[1], rt[2]);
    appendFmt(out,"%-28s: %d\n", "cpu time for a cycle (ms)", cputime);
    appendFmt(out,"%-28s: %d\n", "missing obs data count", prcout);
    appendFmt(out,"%-28s: %d,%d\n", "bytes in input buffer", nb[0], nb[1]);

    // 输入数据统计
    for (i=0;i<3;i++) {
        appendFmt(out,"# input data %s :\n", type[i]);
        appendFmt(out,"obs(%d),nav(%d),gnav(%d),ion(%d),sbs(%d),pos(%d),dgps(%d),ssr(%d),err(%d)\n",
                  nmsg[i][0], nmsg[i][1], nmsg[i][6], nmsg[i][2], nmsg[i][3],
                  nmsg[i][4], nmsg[i][5], nmsg[i][7], nmsg[i][9]);
    }

    // RTCM 消息详情
    for (i=0;i<3;i++) {
        p=s; *p='\0';
        for (j=1;j<100;j++) {
            if (rtcm[i].nmsg2[j]==0) continue;
            p+=sprintf(p,"%s%d(%d)",p>s?",":"",j,rtcm[i].nmsg2[j]);
        }
        if (rtcm[i].nmsg2[0]>0) {
            sprintf(p,"%sother2(%d)",p>s?",":"",rtcm[i].nmsg2[0]);
        }
        for (j=1;j<300;j++) {
            if (rtcm[i].nmsg3[j]==0) continue;
            p+=sprintf(p,"%s%d(%d)",p>s?",":"",j+1000,rtcm[i].nmsg3[j]);
        }
        if (rtcm[i].nmsg3[0]>0) {
            sprintf(p,"%sother3(%d)",p>s?",":"",rtcm[i].nmsg3[0]);
        }
        appendFmt(out,"# rtcm messages %s: \n%s\n", type[i], s);
    }

    // 解算状态
    appendFmt(out,"%-28s: %s\n", "solution status", sol[rtkstat]);
    time2str(rtk->sol.time, tstr, 9);
    appendFmt(out,"%-28s: %s\n", "time of receiver clock rover", rtk->sol.time.time ? tstr : "-");
    appendFmt(out,"%-28s: %.3f,%.3f,%.3f,%.3f\n", "time sys offset (ns)",
              rtk->sol.dtr[1]*1e9, rtk->sol.dtr[2]*1e9, rtk->sol.dtr[3]*1e9, rtk->sol.dtr[4]*1e9);
    appendFmt(out,"%-28s: %.3f\n", "solution interval (s)", rtk->tt);
    appendFmt(out,"%-28s: %.3f\n", "age of differential (s)", rtk->sol.age);
    appendFmt(out,"%-28s: %.3f\n", "ratio for ar validation", rtk->sol.ratio);
    appendFmt(out,"%-28s: %d\n", "# satellites rover", nsat0);
    appendFmt(out,"%-28s: %d\n", "# satellites base", nsat1);
    appendFmt(out,"%-28s: %d\n", "# valid satellites", rtk->sol.ns);
    appendFmt(out,"%-28s: %.1f,%.1f,%.1f,%.1f\n", "GDOP/PDOP/HDOP/VDOP", dop[0], dop[1], dop[2], dop[3]);
    appendFmt(out,"%-28s: %d\n", "# real estimated states", rtk->na);
    appendFmt(out,"%-28s: %d\n", "# all estimated states", rtk->nx);

    // 坐标结果 - Rover Single
    appendFmt(out,"%-28s: %.3f,%.3f,%.3f\n", "pos xyz single (m) rover",
              rtk->sol.rr[0], rtk->sol.rr[1], rtk->sol.rr[2]);
    if (norm(rtk->sol.rr, 3) > 0.0) ecef2pos(rtk->sol.rr, pos); else pos[0]=pos[1]=pos[2]=0.0;
    appendFmt(out,"%-28s: %.8f,%.8f,%.3f\n", "pos llh single (deg,m) rover",
              pos[0]*R2D, pos[1]*R2D, pos[2]);
    ecef2enu(pos, rtk->sol.rr+3, vel);
    appendFmt(out,"%-28s: %.3f,%.3f,%.3f\n", "vel enu (m/s) rover", vel[0], vel[1], vel[2]);

    // 坐标结果 - Rover Float
    appendFmt(out,"%-28s: %.3f,%.3f,%.3f\n", "pos xyz float (m) rover",
              rtk->x ? rtk->x[0] : 0, rtk->x ? rtk->x[1] : 0, rtk->x ? rtk->x[2] : 0);
    appendFmt(out,"%-28s: %.3f,%.3f,%.3f\n", "pos xyz float std (m) rover",
              rtk->P ? SQRT(rtk->P[0]) : 0,
              rtk->P ? SQRT(rtk->P[1+1*rtk->nx]) : 0,
              rtk->P ? SQRT(rtk->P[2+2*rtk->nx]) : 0);

    // 坐标结果 - Rover Fixed
    appendFmt(out,"%-28s: %.3f,%.3f,%.3f\n", "pos xyz fixed (m) rover",
              rtk->xa ? rtk->xa[0] : 0, rtk->xa ? rtk->xa[1] : 0, rtk->xa ? rtk->xa[2] : 0);
    appendFmt(out,"%-28s: %.3f,%.3f,%.3f\n", "pos xyz fixed std (m) rover",
              rtk->Pa ? SQRT(rtk->Pa[0]) : 0,
              rtk->Pa ? SQRT(rtk->Pa[1+1*rtk->na]) : 0,
              rtk->Pa ? SQRT(rtk->Pa[2+2*rtk->na]) : 0);

    // 坐标结果 - Base
    appendFmt(out,"%-28s: %.3f,%.3f,%.3f\n", "pos xyz (m) base",
              rtk->rb[0], rtk->rb[1], rtk->rb[2]);
    if (norm(rtk->rb, 3) > 0.0) ecef2pos(rtk->rb, pos); else pos[0]=pos[1]=pos[2]=0.0;
    appendFmt(out,"%-28s: %.8f,%.8f,%.3f\n", "pos llh (deg,m) base",
              pos[0]*R2D, pos[1]*R2D, pos[2]);
    appendFmt(out,"%-28s: %d\n", "# average single pos base", nave);

    // 天线信息
    appendFmt(out,"%-28s: %s\n", "ant type rover", rtk->opt.pcvr[0].type);
    del = rtk->opt.antdel[0];
    appendFmt(out,"%-28s: %.4f %.4f %.4f\n", "ant delta rover", del[0], del[1], del[2]);
    appendFmt(out,"%-28s: %s\n", "ant type base", rtk->opt.pcvr[1].type);
    del = rtk->opt.antdel[1];
    appendFmt(out,"%-28s: %.4f %.4f %.4f\n", "ant delta base", del[0], del[1], del[2]);

    // 基线信息
    ecef2enu(pos, rtk->rb+3, vel);
    appendFmt(out,"%-28s: %.3f,%.3f,%.3f\n", "vel enu (m/s) base", vel[0], vel[1], vel[2]);

    if (rtk->opt.mode > 0 && rtk->x && norm(rtk->x, 3) > 0.0) {
        for (i=0;i<3;i++) rr[i] = rtk->x[i] - rtk->rb[i];
        bl1 = norm(rr, 3);
    }
    if (rtk->opt.mode > 0 && rtk->xa && norm(rtk->xa, 3) > 0.0) {
        for (i=0;i<3;i++) rr[i] = rtk->xa[i] - rtk->rb[i];
        bl2 = norm(rr, 3);
    }
    appendFmt(out,"%-28s: %.3f\n", "baseline length float (m)", bl1);
    appendFmt(out,"%-28s: %.3f\n", "baseline length fixed (m)", bl2);

    // 事件标记信息
    appendFmt(out,"%-28s: %s\n", "last time mark", tmcount ? tmstr : "-");
    appendFmt(out,"%-28s: %d\n", "receiver time mark count", rcvcount);
    appendFmt(out,"%-28s: %d\n", "rtklib time mark count", tmcount);

    // 释放内存
    std::free(rtk);
    std::free(rtcm);
    return true;
}

// 获取卫星状态文本，仿照 rtkrcv.c 中 prsatellite() 实现
bool RtServer::getSatelliteStatusText(std::string &out, int nf) {
    out.clear();

    rtksvr_t *svr = &m_svr;

    // 若服务未启动，则不返回任何内容
    if (svr->state == 0) {
        return false;
    }

    // 频点标记，与 rtkrcv 中保持一致
    int frq[] = {1, 2, 5, 7, 8, 6};
    int maxnf = (int)(sizeof(frq) / sizeof(frq[0]));

    if (nf <= 0 || nf > NFREQ) nf = NFREQ;
    if (nf > maxnf) nf = maxnf;

    rtk_t *rtk = (rtk_t *)std::calloc(1, sizeof(rtk_t));
    if (!rtk) return false;

    // 加锁拷贝 rtk 状态
    rtksvrlock(svr);
    *rtk = svr->rtk;
    rtksvrunlock(svr);

    // 表头
    appendFmt(out, "%3s %2s %5s %4s", "SAT", "C1", "Az", "El");
    for (int j = 0; j < nf; j++) appendFmt(out, " L%d", frq[j]);
    for (int j = 0; j < nf; j++) appendFmt(out, "  Fix%d", frq[j]);
    for (int j = 0; j < nf; j++) appendFmt(out, "  P%dRes", frq[j]);
    for (int j = 0; j < nf; j++) appendFmt(out, "   L%dRes", frq[j]);
    for (int j = 0; j < nf; j++) appendFmt(out, "  Sl%d", frq[j]);
    for (int j = 0; j < nf; j++) appendFmt(out, "  Lock%d", frq[j]);
    for (int j = 0; j < nf; j++) appendFmt(out, " Rj%d", frq[j]);
    appendFmt(out, "\n");

    // 每颗卫星逐行输出
    for (int i = 0; i < MAXSAT; i++) {
        if (rtk->ssat[i].azel[1] <= 0.0) continue;

        char id[8];
        satno2id(i + 1, id);

        double az = rtk->ssat[i].azel[0] * R2D;
        if (az < 0.0) az += 360.0;
        double el = rtk->ssat[i].azel[1] * R2D;

        appendFmt(out, "%3s %2s %5.1f %4.1f",
                  id, rtk->ssat[i].vs ? "OK" : "-",
                  az, el);

        // 各频点可用性
        for (int j = 0; j < nf; j++) {
            appendFmt(out, " %2s", rtk->ssat[i].vsat[j] ? "OK" : "-");
        }

        // 固定状态 FLOAT/FIX/HOLD
        for (int j = 0; j < nf; j++) {
            int fix = rtk->ssat[i].fix[j];
            const char *fstr =
                    (fix == 1) ? "FLOAT" :
                    (fix == 2) ? "FIX"   :
                    (fix == 3) ? "HOLD"  : "-";
            appendFmt(out, " %5s", fstr);
        }

        // 伪距/相位残差
        for (int j = 0; j < nf; j++) appendFmt(out, "%7.3f", rtk->ssat[i].resp[j]);
        for (int j = 0; j < nf; j++) appendFmt(out, "%8.4f", rtk->ssat[i].resc[j]);

        // cycle-slip / lock / reject 计数
        for (int j = 0; j < nf; j++) appendFmt(out, " %4d", rtk->ssat[i].slipc[j]);
        for (int j = 0; j < nf; j++) appendFmt(out, " %6d", rtk->ssat[i].lock[j]);
        for (int j = 0; j < nf; j++) appendFmt(out, " %3d", rtk->ssat[i].rejc[j]);

        appendFmt(out, "\n");
    }

    std::free(rtk);
    return !out.empty();
}

// 获取观测数据状态文本，仿照 rtkrcv.c 中 probserv() 实现
bool RtServer::getObservationStatusText(std::string &out, int nf) {
    out.clear();

    rtksvr_t *svr = &m_svr;

    if (svr->state == 0) {
        return false;
    }

    // 频点序号，与 probserv 中保持一致
    int frq[] = {1, 2, 5, 7, 8, 6, 9};
    int maxnf = (int)(sizeof(frq) / sizeof(frq[0]));

    if (nf <= 0 || nf > NFREQ) nf = NFREQ;
    if (nf > maxnf) nf = maxnf;

    // 最多两路接收机观测（rover + base）
    obsd_t *obs = (obsd_t *)std::calloc(MAXOBS * 2, sizeof(obsd_t));
    if (!obs) return false;

    int n = 0;

    // 加锁复制观测数据
    rtksvrlock(svr);
    for (int i = 0; i < svr->obs[0][0].n && n < MAXOBS * 2; i++) {
        obs[n++] = svr->obs[0][0].data[i];
    }
    for (int i = 0; i < svr->obs[1][0].n && n < MAXOBS * 2; i++) {
        obs[n++] = svr->obs[1][0].data[i];
    }
    rtksvrunlock(svr);

    if (n <= 0) {
        std::free(obs);
        return false;
    }

    char tstr[40], id[8];

    // 表头
    appendFmt(out, "%-22s %3s %s", "TIME(GPST)", "SAT", "R");
    for (int i = 0; i < nf; i++) appendFmt(out, "        P%d(m)", frq[i]);
    for (int i = 0; i < nf; i++) appendFmt(out, "       L%d(cyc)", frq[i]);
    for (int i = 0; i < nf; i++) appendFmt(out, "  D%d(Hz)", frq[i]);
    for (int i = 0; i < nf; i++) appendFmt(out, " S%d", frq[i]);
    appendFmt(out, " LLI\n");

    // 每条观测记录一行
    for (int i = 0; i < n; i++) {
        time2str(obs[i].time, tstr, 2);
        satno2id(obs[i].sat, id);

        appendFmt(out, "%s %3s %d", tstr, id, obs[i].rcv);

        for (int j = 0; j < nf; j++) appendFmt(out, "%13.3f", obs[i].P[j]);
        for (int j = 0; j < nf; j++) appendFmt(out, "%14.3f", obs[i].L[j]);
        for (int j = 0; j < nf; j++) appendFmt(out, "%8.1f",  obs[i].D[j]);
        for (int j = 0; j < nf; j++) appendFmt(out, "%3.0f",  obs[i].SNR[j]);
        for (int j = 0; j < nf; j++) appendFmt(out, "%2d",    obs[i].LLI[j]);

        appendFmt(out, "\n");
    }

    std::free(obs);
    return !out.empty();
}

// 获取数据流状态文本，仿照 rtkrcv.c 中 prstream() 实现
bool RtServer::getStreamStatusText(std::string &out) {
    out.clear();

    rtksvr_t *svr = &m_svr;

    if (svr->state == 0) {
        return false;
    }

    const char *ch[] = {
            "input rover","input base","input corr","output sol1","output sol2",
            "log rover","log base","log corr","monitor"
    };
    const char *type[] = {
            "-","serial","file","tcpsvr","tcpcli","ntrips","ntripc","ftp",
            "http","ntripcas","udpsvr","udpcli","membuf"
    };
    const char *fmt[] = {
            "rtcm2","rtcm3","oem4","","ubx","swift","hemis","skytreq",
            "javad","nvs","binex","rt17","sbf","","unicore","sp3",""
    };
    const char *sol[] = {"llh","xyz","enu","nmea","stat","-"};

    stream_t stream[9];
    int format[9] = {0};

    // 加锁拷贝流状态
    rtksvrlock(svr);
    for (int i = 0; i < 8; i++) stream[i] = svr->stream[i];
    for (int i = 0; i < 3; i++) format[i] = svr->format[i];
    for (int i = 3; i < 5; i++) format[i] = svr->solopt[i - 3].posf;
    stream[8] = m_moni;
    format[8] = SOLF_LLH;
    rtksvrunlock(svr);

    // 表头
    appendFmt(out,
              "%-12s %-8s %-5s %s %10s %7s %10s %7s %-24s %s\n",
              "Stream","Type","Fmt","S","In-byte","In-bps",
              "Out-byte","Out-bps","Path","Message");

    // 每个流一行
    for (int i = 0; i < 9; i++) {
        const char *typeStr = type[stream[i].type];
        const char *fmtStr = "-";

        if (i < 3) {
            fmtStr = fmt[format[i]];
        }
        else if (i < 5 || i == 8) {
            fmtStr = sol[format[i]];
        }

        const char *stateStr = stream[i].state < 0 ? "E" :
                               (stream[i].state ? "C" : "-");

        appendFmt(out,
                  "%-12s %-8s %-5s %s %10d %7d %10d %7d %-24.24s %s\n",
                  ch[i],
                  typeStr,
                  fmtStr,
                  stateStr,
                  stream[i].inb,
                  stream[i].inr,
                  stream[i].outb,
                  stream[i].outr,
                  stream[i].path,
                  stream[i].msg);
    }

    return !out.empty();
}

// 获取错误消息文本，仿照 rtkrcv.c 中 prerror() 实现
bool RtServer::getErrorStatusText(std::string &out) {
    out.clear();

    rtksvr_t *svr = &m_svr;

    if (svr->state == 0) {
        return false;
    }

    int n = 0;

    // 加锁读取错误缓冲区
    rtksvrlock(svr);
    n = svr->rtk.neb;
    if (n > 0) {
        // 确保以 0 结尾，防止字符串越界
        svr->rtk.errbuf[n] = '\0';
        out.assign(svr->rtk.errbuf, n);
        svr->rtk.neb = 0;   // 读取后清空缓冲
    }
    rtksvrunlock(svr);

    return !out.empty();
}

// 解算线程
void RtServer::solverThread() {
    rtksvr_t *svr=&m_svr;

    obs_t obs;
    sol_t sol={{0}};
    double tt;
    uint32_t tick,ticknmea,tick1hz,tickreset;
    uint8_t *p,*q;
    char msg[128];
    int i,j,n,cycle,cputime;

    // 分配 obs.data
    obsd_t *data = (obsd_t*)calloc(MAXOBS*2, sizeof(obsd_t));
    if (!data) return;
    obs.data = data; obs.n=0; obs.nmax=MAXOBS*2;

    svr->state=1;
    svr->tick=tickget();
    ticknmea=tick1hz=svr->tick-1000;
    tickreset=svr->tick-30000;

    for (cycle=0; svr->state; cycle++) {
        tick=tickget();

        for (i=0;i<3;i++) {
            if (m_roverInternal && i==0) continue; // rover internal mode uses queue
            p=svr->buff[i]+svr->nb[i]; q=svr->buff[i]+svr->buffsize;

            if ((n=strread(svr->stream+i,p,q-p))<=0) {
                continue;
            }
            strwrite(svr->stream+i+5,p,n);
            svr->nb[i]+=n;

            rtksvrlock(svr);
            n=n<svr->buffsize-svr->npb[i]?n:svr->buffsize-svr->npb[i];
            memcpy(svr->pbuf[i]+svr->npb[i],p,n);
            svr->npb[i]+=n;
            rtksvrunlock(svr);
        }
        int fobs[3]={0};
        for (i=0;i<3;i++) {
            if (m_roverInternal && i==0) {
                fobs[i]=decodeInternal(svr,i);
            } else if (svr->format[i]==STRFMT_SP3||svr->format[i]==STRFMT_RNXCLK) {
                decodefile(svr,i);
            } else {
                fobs[i]=decoderaw(svr,i);
                if (1==i&&svr->rtcm[1].staid>0) sol.refstationid=svr->rtcm[1].staid;
            }
        }
        if (fobs[1]>0&&svr->rtk.opt.refpos==POSOPT_SINGLE) {
            if ((svr->rtk.opt.maxaveep<=0||svr->nave<svr->rtk.opt.maxaveep)&&
                pntpos(svr->obs[1][0].data,svr->obs[1][0].n,&svr->nav, &svr->rtk.opt,&sol,NULL,NULL,msg)) {
                svr->nave++;
                for (i=0;i<3;i++) {
                    svr->rb_ave[i]+=(sol.rr[i]-svr->rb_ave[i])/svr->nave;
                }
            }
            for (i=0;i<3;i++) svr->rtk.opt.rb[i]=svr->rb_ave[i];
        }
        for (i=0;i<fobs[0];i++) {
            obs.n=0;
            for (j=0;j<svr->obs[0][i].n&&obs.n<MAXOBS*2; j++) {
                obs.data[obs.n++]=svr->obs[0][i].data[j];
            }
            for (j=0;j<svr->obs[1][0].n&&obs.n<MAXOBS*2; j++) {
                obs.data[obs.n++]=svr->obs[1][0].data[j];
            }
            if (!strstr(svr->rtk.opt.pppopt,"-DIS_FCB")) {
                corr_phase_bias(obs.data,obs.n,&svr->nav);
            }
            rtksvrlock(svr);
            rtkpos(&svr->rtk,obs.data,obs.n,&svr->nav);
            rtksvrunlock(svr);

            if (svr->rtk.sol.stat!=SOLQ_NONE) {
                tt=(int)(tickget()-tick)/1000.0+DTTOL;
                timeset(gpst2utc(timeadd(svr->rtk.sol.time,tt)));
                writesol(svr,i);
            }
            if ((int)(tickget()-tick)>=svr->cycle) {
                svr->prcout+=fobs[0]-i-1;
            }
        }
        if (svr->rtk.sol.stat==SOLQ_NONE && (int)(tick-tick1hz)>=1000) {
            writesol(svr,0);
            tick1hz=tick;
        }
        for (i=0;i<3;i++) periodic_cmd(cycle*svr->cycle, svr->cmds_periodic[i], svr->stream+i);
        if (svr->nmeacycle>0 && (int)(tick-ticknmea)>=svr->nmeacycle) { send_nmea(svr,&tickreset); ticknmea=tick; }

        if ((cputime=(int)(tickget()-tick))>0) svr->cputime=cputime;
        sleepms(svr->cycle - cputime);
    }

    std::free(data);
    for (int i=0;i<MAXSTRRTK;i++) strclose(svr->stream+i);
    for (int i=0;i<3;i++) {
        svr->nb[i]=svr->npb[i]=0;
        std::free(svr->buff[i]); svr->buff[i]=nullptr;
        std::free(svr->pbuf[i]); svr->pbuf[i]=nullptr;
        free_raw (svr->raw +i);
        free_rtcm(svr->rtcm+i);
    }
    for (int i=0;i<2;i++) {
        svr->nsb[i]=0;
        std::free(svr->sbuf[i]); svr->sbuf[i]=nullptr;
    }
    return;
}

// rtksvr.c 原样封装的辅助函数（实现保持一致）
void RtServer::writesolhead(stream_t *stream, const solopt_t *solopt, const prcopt_t *prcopt) {
    if (solopt->posf == SOLF_NMEA || solopt->posf == SOLF_STAT || solopt->posf==SOLF_GSIF) return;
    uint8_t buff[MAXSOLMSG+1];
    if (solopt->outhead) {
        if (!*solopt->prog) {
            int n=snprintf((char*)buff,sizeof(buff),"%s program   : RTKLIB ver.%s %s\n",COMMENTH,VER_RTKLIB,PATCH_LEVEL);
            if (n<(int)sizeof(buff)) strwrite(stream,buff,n);
        } else {
            int n=snprintf((char*)buff,sizeof(buff),"%s program   : %s\n",COMMENTH,solopt->prog);
            if (n<(int)sizeof(buff)) strwrite(stream,buff,n);
        }
    }
    if (solopt->outopt) {
        int n=outprcopts(buff, prcopt);
        strwrite(stream,buff,n);
    }
    if (solopt->outhead || solopt->outopt) {
        int n=snprintf((char*)buff,sizeof(buff),"%s\n",COMMENTH);
        if (n<(int)sizeof(buff)) strwrite(stream,buff,n);
    }
    int n=outsolheads(buff,solopt);
    strwrite(stream,buff,n);
}
void RtServer::saveoutbuf(rtksvr_t *svr, uint8_t *buff, int n, int index) {
    n=n<svr->buffsize-svr->nsb[index]?n:svr->buffsize-svr->nsb[index];
    memcpy(svr->sbuf[index]+svr->nsb[index],buff,n);
    svr->nsb[index]+=n;
}
void RtServer::writesol(rtksvr_t *svr, int index) {
    solopt_t solopt=solopt_default;
    uint8_t buff[MAXSOLMSG+1];
    int i,n;
    for (i=0;i<2;i++) {
        if (svr->solopt[i].posf==SOLF_STAT) {
            rtksvrlock(svr);
            n=rtkoutstat(&svr->rtk,svr->solopt[i].sstat,(char*)buff);
            rtksvrunlock(svr);
        } else {
            n=outsols(buff,&svr->rtk.sol,svr->rtk.rb,svr->solopt+i);
        }
        strwrite(svr->stream+i+3,buff,n);
        rtksvrlock(svr); saveoutbuf(svr,buff,n,i); rtksvrunlock(svr);
        n=outsolexs(buff,&svr->rtk.sol,svr->rtk.ssat,svr->solopt+i);
        strwrite(svr->stream+i+3,buff,n);
        rtksvrlock(svr); saveoutbuf(svr,buff,n,i); rtksvrunlock(svr);
    }
    if (svr->moni) {
        n=outsols(buff,&svr->rtk.sol,svr->rtk.rb,&solopt);
        strwrite(svr->moni,buff,n);
    }
    if (svr->nsol<MAXSOLBUF) {
        rtksvrlock(svr); svr->solbuf[svr->nsol++]=svr->rtk.sol; rtksvrunlock(svr);
    }
}
void RtServer::update_glofcn(rtksvr_t *svr) {
    for (int i=0;i<MAXPRNGLO;i++) {
        int sat = satno(SYS_GLO,i+1), frq=-999;
        for (int j=0;j<3;j++) {
            if (svr->raw[j].nav.geph[i].sat!=sat) continue;
            frq=svr->raw[j].nav.geph[i].frq;
        }
        if (frq<-7||frq>6) continue;
        for (int j=0;j<3;j++) {
            if (svr->raw[j].nav.geph[i].sat==sat) continue;
            svr->raw[j].nav.geph[i].sat=sat;
            svr->raw[j].nav.geph[i].frq=frq;
        }
    }
}
void RtServer::update_obs(rtksvr_t *svr, obs_t *obs, int index, int iobs) {
    int i,n=0,sat,sys;
    if (iobs<MAXOBSBUF) {
        for (i=0;i<obs->n;i++) {
            sat=obs->data[i].sat;
            sys=satsys(sat,NULL);
            if (svr->rtk.opt.exsats[sat-1]==1||!(sys&svr->rtk.opt.navsys)) continue;
            svr->obs[index][iobs].data[n]=obs->data[i];
            svr->obs[index][iobs].data[n++].rcv=index+1;
        }
        svr->obs[index][iobs].n=n;
        sortobs(&svr->obs[index][iobs]);
    }
    svr->nmsg[index][0]++;
}
void RtServer::update_eph(rtksvr_t *svr, nav_t *nav, int ephsat, int ephset, int index) {
    eph_t *eph1,*eph2,*eph3; int prn;
    if (satsys(ephsat,&prn)!=SYS_GLO) {
        if (!svr->navsel||svr->navsel==index+1) {
            eph1=nav->eph+ephsat-1+MAXSAT*ephset;
            eph2=svr->nav.eph+ephsat-1+MAXSAT*ephset;
            eph3=svr->nav.eph+ephsat-1+MAXSAT*(2+ephset);
            if (eph2->ttr.time==0||
                (eph1->iode!=eph3->iode&&eph1->iode!=eph2->iode)||
                (timediff(eph1->toe,eph3->toe)!=0.0&&timediff(eph1->toe,eph2->toe)!=0.0)||
                (timediff(eph1->toc,eph3->toc)!=0.0&&timediff(eph1->toc,eph2->toc)!=0.0)) {
                *eph3=*eph2; *eph2=*eph1;
            }
        }
        svr->nmsg[index][1]++;
    } else {
        if (!svr->navsel||svr->navsel==index+1) {
            geph_t *geph1,*geph2,*geph3; geph1=nav->geph+prn-1; geph2=svr->nav.geph+prn-1; geph3=svr->nav.geph+prn-1+MAXPRNGLO;
            if (geph2->tof.time==0||(geph1->iode!=geph3->iode&&geph1->iode!=geph2->iode)) { *geph3=*geph2; *geph2=*geph1; update_glofcn(svr); }
        }
        svr->nmsg[index][6]++;
    }
}
void RtServer::update_sbs(rtksvr_t *svr, sbsmsg_t *sbsmsg, int index){
    int i,sbssat=svr->rtk.opt.sbassatsel;
    if (sbsmsg&&(sbssat==sbsmsg->prn||sbssat==0)) {
        sbsmsg->rcv=index+1;
        if (svr->nsbs<MAXSBSMSG) svr->sbsmsg[svr->nsbs++]=*sbsmsg;
        else { for (i=0;i<MAXSBSMSG-1;i++) svr->sbsmsg[i]=svr->sbsmsg[i+1]; svr->sbsmsg[i]=*sbsmsg; }
        sbsupdatecorr(sbsmsg,&svr->nav);
    }
    svr->nmsg[index][3]++;
}
void RtServer::update_ionutc(rtksvr_t *svr, nav_t *nav, int index){
    if (svr->navsel==0||svr->navsel==index+1) {
        matcpy(svr->nav.utc_gps,nav->utc_gps,8,1);
        matcpy(svr->nav.utc_glo,nav->utc_glo,8,1);
        matcpy(svr->nav.utc_gal,nav->utc_gal,8,1);
        matcpy(svr->nav.utc_qzs,nav->utc_qzs,8,1);
        matcpy(svr->nav.utc_cmp,nav->utc_cmp,8,1);
        matcpy(svr->nav.utc_irn,nav->utc_irn,9,1);
        matcpy(svr->nav.utc_sbs,nav->utc_sbs,4,1);
        matcpy(svr->nav.ion_gps,nav->ion_gps,8,1);
        matcpy(svr->nav.ion_gal,nav->ion_gal,4,1);
        matcpy(svr->nav.ion_qzs,nav->ion_qzs,8,1);
        matcpy(svr->nav.ion_cmp,nav->ion_cmp,8,1);
        matcpy(svr->nav.ion_irn,nav->ion_irn,8,1);
    }
    svr->nmsg[index][2]++;
}
void RtServer::update_antpos(rtksvr_t *svr, int index){
    sta_t *sta;
    if (svr->format[index] == STRFMT_RTCM2 || svr->format[index] == STRFMT_RTCM3) sta=&svr->rtcm[index].sta;
    else                                                                          sta=&svr->raw [index].sta;

    if (index==1 && svr->rtk.opt.refpos==POSOPT_RTCM) { for (int i=0;i<3;i++) svr->rtk.rb[i]=sta->pos[i]; }
    if (index==0 && svr->rtk.opt.rovpos==POSOPT_RTCM &&
        (svr->rtk.opt.mode==PMODE_FIXED || svr->rtk.opt.mode==PMODE_PPP_FIXED)) {
        for (int i=0;i<3;i++) svr->rtk.opt.ru[i]=sta->pos[i];
    }
    if (strcmp(svr->rtk.opt.anttype[index], "*")==0) {
        if (sta->antdes[0]!='\0' && strcmp(svr->rtk.opt.pcvr[index].type, sta->antdes)!=0) {
            pcv_t *pcv = searchpcv(0, sta->antdes, utc2gpst(timeget()), &svr->pcvsr);
            if (pcv) svr->rtk.opt.pcvr[index]=*pcv;
        }
        if (sta->deltype==1) {
            if (norm(sta->pos,3)>0.0) {
                double pos[3]; ecef2pos(sta->pos,pos);
                ecef2enu(pos, sta->del, svr->rtk.opt.antdel[index]);
                svr->rtk.opt.antdel[index][2]+=sta->hgt;
            }
        } else {
            for (int i=0;i<3;i++) svr->rtk.opt.antdel[index][i]=sta->del[i];
            svr->rtk.opt.antdel[index][2]+=sta->hgt;
        }
    }
    svr->nmsg[index][4]++;
}
void RtServer::update_ssr(rtksvr_t *svr, int index){
    for (int i=0;i<MAXSAT;i++) {
        if (!svr->rtcm[index].ssr[i].update) continue;
        if (svr->rtcm[index].ssr[i].iod[0]!=svr->rtcm[index].ssr[i].iod[1]) continue;
        svr->rtcm[index].ssr[i].update=0;
        int iode=svr->rtcm[index].ssr[i].iode;
        int sys,prn; sys=satsys(i+1,&prn);
        if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS) {
            if (svr->nav.eph[i].iode!=iode && svr->nav.eph[i+MAXSAT].iode!=iode) continue;
        } else if (sys==SYS_GLO) {
            if (svr->nav.geph[prn-1].iode!=iode && svr->nav.geph[prn-1+MAXPRNGLO].iode!=iode) continue;
        }
        svr->nav.ssr[i]=svr->rtcm[index].ssr[i];
    }
    svr->nmsg[index][7]++;
}
void RtServer::update_svr(rtksvr_t *svr, int ret, obs_t *obs, nav_t *nav, int ephsat, int ephset, sbsmsg_t *sbsmsg, int index, int iobs){
    if (ret==1) update_obs(svr,obs,index,iobs);
    else if (ret==2) update_eph(svr,nav,ephsat,ephset,index);
    else if (ret==3) update_sbs(svr,sbsmsg,index);
    else if (ret==9) update_ionutc(svr,nav,index);
    else if (ret==5) update_antpos(svr,index);
    else if (ret==7) svr->nmsg[index][5]++;
    else if (ret==10) update_ssr(svr,index);
    else if (ret==-1) svr->nmsg[index][9]++;
}
int RtServer::decodeInternal(rtksvr_t *svr, int index){
    EpochObs epoch{};
    bool has_data = false;

    // 队列加锁取最新一帧，清空剩余
    {
        std::lock_guard<std::mutex> lk(m_qmtx);
        if (!m_q.empty()) {
            epoch = m_q.back(); // 取最新数据
            m_q.clear();
            has_data = true;
        }
    }
    if (!has_data) return 0;

    // 构造 obs_t
    obs_t obs{};
    obs.data = epoch.data;
    obs.n = epoch.n;
    obs.nmax = MAXOBS;

    int fobs = 0;
    rtksvrlock(svr);

    // 更新统计计数
    svr->raw[index].obs.rcvcount++;
    // 将观测数据注入 Server 状态
    update_svr(svr, 1, &obs, &svr->nav, 0, 0, nullptr, index, fobs);
    fobs++;

    // 溢出统计
    if (fobs >= MAXOBSBUF) {
        svr->prcout++;
        fobs = MAXOBSBUF;
    }

    svr->nb[index] = 0;
    rtksvrunlock(svr);
    return fobs;
}
int RtServer::decoderaw(rtksvr_t *svr, int index){
    obs_t *obs; nav_t *nav; sbsmsg_t *sbsmsg=NULL;
    int i,ret,ephsat,ephset,fobs=0;
    rtksvrlock(svr);
    for (i=0;i<svr->nb[index];i++) {
        if (svr->format[index]==STRFMT_RTCM2) {
            ret=input_rtcm2(svr->rtcm+index,svr->buff[index][i]);
            obs=&svr->rtcm[index].obs; nav=&svr->rtcm[index].nav;
            ephsat=svr->rtcm[index].ephsat; ephset=svr->rtcm[index].ephset;
        } else if (svr->format[index]==STRFMT_RTCM3) {
            ret=input_rtcm3(svr->rtcm+index,svr->buff[index][i]);
            if (svr->rtcm[index].nbyte_invalid!=0) {
                i-=svr->rtcm[index].nbyte_invalid-1; i=i>=0?i:0;
                svr->rtcm[index].nbyte_invalid=0;
            }
            obs=&svr->rtcm[index].obs; nav=&svr->rtcm[index].nav;
            ephsat=svr->rtcm[index].ephsat; ephset=svr->rtcm[index].ephset;
        } else {
            ret=input_raw(svr->raw+index,svr->format[index],svr->buff[index][i]);
            obs=&svr->raw[index].obs; nav=&svr->raw[index].nav;
            ephsat=svr->raw[index].ephsat; ephset=svr->raw[index].ephset;
            sbsmsg=&svr->raw[index].sbsmsg;
        }
        if (ret>0) update_svr(svr,ret,obs,nav,ephsat,ephset,sbsmsg,index,fobs);
        if (ret==1) { if (fobs<MAXOBSBUF) fobs++; else svr->prcout++; }
    }
    svr->nb[index]=0;
    rtksvrunlock(svr);
    return fobs;
}
void RtServer::decodefile(rtksvr_t *svr, int index){
    nav_t *nav=(nav_t*)calloc(1,sizeof(nav_t)); if(!nav) return;
    rtksvrlock(svr);
    int nb=svr->nb[index];
    if (nb<=2||svr->buff[index][nb-2]!='\r'||svr->buff[index][nb-1]!='\n'){ rtksvrunlock(svr);
        std::free(nav); return; }
    char file[1024]; strncpy(file,(char*)svr->buff[index],nb-2); file[nb-2]='\0'; svr->nb[index]=0;
    rtksvrunlock(svr);
    if (svr->format[index]==STRFMT_SP3) {
        readsp3(file,nav,0);
        if (nav->ne<=0) { std::free(nav); return; }
        rtksvrlock(svr);
        if (svr->nav.peph) std::free(svr->nav.peph);
        svr->nav.ne=nav->ne; svr->nav.nemax=nav->nemax; svr->nav.peph=nav->peph;
        svr->ftime[index]=utc2gpst(timeget()); strcpy(svr->files[index],file);
        rtksvrunlock(svr);
    } else if (svr->format[index]==STRFMT_RNXCLK) {
        if (readrnxc(file,nav)<=0) { std::free(nav); return; }
        rtksvrlock(svr);
        if (svr->nav.pclk) std::free(svr->nav.pclk);
        svr->nav.nc=nav->nc; svr->nav.ncmax=nav->ncmax; svr->nav.pclk=nav->pclk;
        svr->ftime[index]=utc2gpst(timeget()); strcpy(svr->files[index],file);
        rtksvrunlock(svr);
    }
    std::free(nav);
}
void RtServer::corr_phase_bias(obsd_t *obs, int n, const nav_t *nav){
    for (int i=0;i<n;i++) for (int j=0;j<NFREQ;j++) {
            uint8_t code=obs[i].code[j];
            double freq=sat2freq(obs[i].sat,code,nav);
            if (freq==0.0) continue;
            obs[i].L[j]-=nav->ssr[obs[i].sat-1].pbias[code-1]*freq/CLIGHT;
        }
}
void RtServer::periodic_cmd(int cycle, const char *cmd, stream_t *stream){
    const char *p=cmd,*q; char msg[1024],*r; int n,period;
    for (p=cmd;;p=q+1) {
        for (q=p;;q++) if (*q=='\r'||*q=='\n'||*q=='\0') break;
        n=(int)(q-p); strncpy(msg,p,n); msg[n]='\0';
        period=0;
        if ((r=strrchr(msg,'#'))) { sscanf(r,"# %d",&period); *r='\0'; while (*--r==' ') *r='\0'; }
        if (period<=0) period=1000;
        if (*msg && cycle%period==0) { strsendcmd(stream,msg); }
        if (!*q) break;
    }
}
double RtServer::baseline_len(const rtk_t *rtk){
    if (norm(rtk->sol.rr,3)<=0.0||norm(rtk->rb,3)<=0.0) return 0.0;
    double dr[3]; for (int i=0;i<3;i++) dr[i]=rtk->sol.rr[i]-rtk->rb[i];
    return norm(dr,3)*0.001;
}
void RtServer::send_nmea(rtksvr_t *svr, uint32_t *tickreset){
    if (svr->stream[1].state!=1) return;
    sol_t sol_nmea={{0}}; sol_nmea.ns=10;
    if (svr->nmeareq==1) {
        sol_nmea.stat=SOLQ_SINGLE; sol_nmea.time=utc2gpst(timeget());
        matcpy(sol_nmea.rr,svr->nmeapos,3,1); strsendnmea(svr->stream+1,&sol_nmea);
    } else if (svr->nmeareq==2) {
        if (norm(svr->rtk.sol.rr,3)<=0.0) return;
        sol_nmea.stat=SOLQ_SINGLE; sol_nmea.time=utc2gpst(timeget());
        matcpy(sol_nmea.rr,svr->rtk.sol.rr,3,1); strsendnmea(svr->stream+1,&sol_nmea);
    } else if (svr->nmeareq==3) {
        double bl=baseline_len(&svr->rtk);
        uint32_t tick=tickget();
        if (bl>=svr->bl_reset && (int)(tick-*tickreset)>30000) {
            strsendcmd(svr->stream+1,svr->cmd_reset); *tickreset=tick;
        }
        if (norm(svr->rtk.sol.rr,3)<=0.0) return;
        sol_nmea.stat=SOLQ_SINGLE; sol_nmea.time=utc2gpst(timeget());
        matcpy(sol_nmea.rr,svr->rtk.sol.rr,3,1);
        double vel=norm(svr->rtk.sol.rr+3,3);
        if (vel>10.0) for (int i=0;i<3;i++) sol_nmea.rr[i]+=svr->rtk.sol.rr[i+3]/vel*svr->bl_reset*0.8;
        strsendnmea(svr->stream+1,&sol_nmea);
    }
}

// open output/log stream
int RtServer::rtksvropenstr(int index, int str, const char *path,
                            const solopt_t *solopt, const prcopt_t *prcopt)
{
    tracet(3,"rtksvropenstr: index=%d str=%d path=%s\n",index,str,path);

    if (index<3||index>7||!m_svr.state) return 0;

    rtksvrlock(&m_svr);

    if (m_svr.stream[index].state>0) {
        rtksvrunlock(&m_svr);
        return 0;
    }
    if (!stropen(m_svr.stream+index,str,STR_MODE_W,path)) {
        tracet(2,"stream open error: index=%d\n",index);
        rtksvrunlock(&m_svr);
        return 0;
    }
    if (index<=4) {
        m_svr.solopt[index-3]=*solopt;
        // write solution header to solution stream
        writesolhead(m_svr.stream+index,m_svr.solopt+(index-3),prcopt);
    }
    rtksvrunlock(&m_svr);
    return 1;
}

// close output/log stream
void RtServer::rtksvrclosestr(int index)
{
    tracet(3,"rtksvrclosestr: index=%d\n",index);

    if (index<3||index>7||!m_svr.state) return;

    rtksvrlock(&m_svr);
    strclose(m_svr.stream+index);
    rtksvrunlock(&m_svr);
}

// get observation data status
int RtServer::rtksvrostat(int rcv, gtime_t *time, int *sat,
                          double *az, double *el, int **snr, int *vsat)
{
    int i,j,ns;

    tracet(4,"rtksvrostat: rcv=%d\n",rcv);

    if (!m_svr.state) return 0;
    rtksvrlock(&m_svr);
    ns=m_svr.obs[rcv][0].n;
    if (ns>0) {
        *time=m_svr.obs[rcv][0].data[0].time;
    }
    for (i=0;i<ns;i++) {
        sat [i]=m_svr.obs[rcv][0].data[i].sat;
        az  [i]=m_svr.rtk.ssat[sat[i]-1].azel[0];
        el  [i]=m_svr.rtk.ssat[sat[i]-1].azel[1];
        for (j=0;j<NFREQ;j++) {
            snr[i][j]=(int)(m_svr.obs[rcv][0].data[i].SNR[j]);
        }
        if (m_svr.rtk.sol.stat==SOLQ_NONE||m_svr.rtk.sol.stat==SOLQ_SINGLE) {
            vsat[i]=m_svr.rtk.ssat[sat[i]-1].vs;
        }
        else {
            vsat[i]=m_svr.rtk.ssat[sat[i]-1].vsat[0];
        }
    }
    rtksvrunlock(&m_svr);
    return ns;
}

// get stream status
void RtServer::rtksvrsstat(int *sstat, char *msg)
{
    int i;
    char s[MAXSTRMSG],*p=msg;

    tracet(4,"rtksvrsstat:\n");

    rtksvrlock(&m_svr);
    for (i=0;i<MAXSTRRTK;i++) {
        sstat[i]=strstat(m_svr.stream+i,s);
        if (*s) p+=sprintf(p,"(%d) %s ",i+1,s);
    }
    rtksvrunlock(&m_svr);
}

// mark current position
int RtServer::rtksvrmark(const char *name, const char *comment)
{
    char buff[MAXSOLMSG+1],tstr[40],*p,*q;
    double tow,pos[3];
    int i,sum,week;

    tracet(4,"rtksvrmark:name=%s comment=%s\n",name,comment);

    if (!m_svr.state) return 0;

    rtksvrlock(&m_svr);

    time2str(m_svr.rtk.sol.time,tstr,3);
    tow=time2gpst(m_svr.rtk.sol.time,&week);
    ecef2pos(m_svr.rtk.sol.rr,pos);

    for (i=0;i<2;i++) {
        p=buff;
        if (m_svr.solopt[i].posf==SOLF_STAT) {
            p+=sprintf(p,"$MARK,%d,%.3f,%d,%.4f,%.4f,%.4f,%s,%s\r\n",week,tow,
                       m_svr.rtk.sol.stat,m_svr.rtk.sol.rr[0],m_svr.rtk.sol.rr[1],
                       m_svr.rtk.sol.rr[2],name,comment);
        }
        else if (m_svr.solopt[i].posf==SOLF_NMEA) {
            p+=sprintf(p,"$GPTXT,01,01,02,MARK:%s,%s,%.9f,%.9f,%.4f,%d,%s",
                       name,tstr,pos[0]*R2D,pos[1]*R2D,pos[2],m_svr.rtk.sol.stat,
                       comment);
            for (q=(char *)buff+1,sum=0;*q;q++) sum^=*q; /* check-sum */
            p+=sprintf(p,"*%02X\r\n",sum);
        }
        else {
            p+=sprintf(p,"%s MARK: %s,%s,%.9f,%.9f,%.4f,%d,%s\r\n",COMMENTH,
                       name,tstr,pos[0]*R2D,pos[1]*R2D,pos[2],m_svr.rtk.sol.stat,
                       comment);
        }
        strwrite(m_svr.stream+i+3,(uint8_t *)buff,(int)(p-buff));
        saveoutbuf(&m_svr,(uint8_t *)buff,(int)(p-buff),i);
    }
    if (m_svr.moni) {
        p=buff;
        p+=sprintf(p,"%s MARK: %s,%s,%.9f,%.9f,%.4f,%d,%s\r\n",COMMENTH,
                   name,tstr,pos[0]*R2D,pos[1]*R2D,pos[2],m_svr.rtk.sol.stat,
                   comment);
        strwrite(m_svr.moni,(uint8_t *)buff,(int)(p-buff));
    }
    rtksvrunlock(&m_svr);
    return 1;
}
