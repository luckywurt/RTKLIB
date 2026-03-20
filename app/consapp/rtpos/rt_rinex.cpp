#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include "rt_rinex.h"
#include "rtklib.h"
#include "rtserver.h"

namespace {

struct EpochBlock {
  int begin;
  int n;
};

static std::thread g_thread;
static std::atomic<int> g_stop{0};
static std::atomic<int> g_running{0};

static RtServer *g_server = nullptr;
static obs_t g_obs = {0};
static std::vector<EpochBlock> g_epochs;
static int g_interval_ms = 1000;
static int g_time_offset_s = 0;

static void set_error(char *errmsg, std::size_t errmsg_len, const char *format, ...) {
  if (!errmsg || errmsg_len == 0) return;
  va_list ap;
  va_start(ap, format);
  vsnprintf(errmsg, errmsg_len, format, ap);
  va_end(ap);
}

static void clear_loaded_obs() {
  if (g_obs.data) {
    freeobs(&g_obs);
  }
  std::memset(&g_obs, 0, sizeof(g_obs));
  g_epochs.clear();
  g_server = nullptr;
}

// 可中断等待，便于 stop 时快速退出线程。
static bool wait_with_stop(int total_ms) {
  if (total_ms <= 0) return g_stop.load() == 0;

  int left = total_ms;
  while (left > 0) {
    if (g_stop.load()) return false;
    const int step = left > 100 ? 100 : left;
    sleepms(step);
    left -= step;
  }
  return g_stop.load() == 0;
}

static bool wait_server_running(int timeout_ms) {
  int left = timeout_ms;
  while (left > 0) {
    if (g_stop.load()) return false;
    if (g_server && g_server->isRunning()) return true;
    const int step = left > 50 ? 50 : left;
    sleepms(step);
    left -= step;
  }
  return g_server && g_server->isRunning();
}

static void inject_worker() {
  g_running.store(1);
  trace(3, "rt_rinex: inject thread start (epochs=%d, interval=%dms, offset=%ds)\n",
        (int)g_epochs.size(), g_interval_ms, g_time_offset_s);

  if (g_time_offset_s > 0) {
    if (!wait_with_stop(g_time_offset_s * 1000)) {
      trace(3, "rt_rinex: inject thread stopped before offset elapsed\n");
      g_running.store(0);
      return;
    }
  }
  if (!wait_server_running(5000)) {
    trace(2, "rt_rinex: rtserver is not running, skip injection\n");
    g_running.store(0);
    return;
  }

  for (std::size_t i = 0; i < g_epochs.size(); ++i) {
    if (g_stop.load()) break;
    if (!g_server || !g_server->isRunning()) break;

    const EpochBlock &ep = g_epochs[i];
    if (ep.n <= 0) continue;

    const obsd_t *epoch_obs = g_obs.data + ep.begin;
    if (!g_server->pushInternalObs(epoch_obs[0].time, epoch_obs, ep.n)) {
      if (!g_server->isRunning()) break;

      char ts[64] = "";
      time2str(epoch_obs[0].time, ts, 3);
      trace(2, "rt_rinex: pushInternalObs failed (%s, n=%d)\n", ts, ep.n);
    }

    if (i + 1 < g_epochs.size() && !wait_with_stop(g_interval_ms)) {
      break;
    }
  }

  trace(3, "rt_rinex: inject thread stop\n");
  g_running.store(0);
}

}  // namespace

bool rt_rinex_start(RtServer *server, const char *rinex_path, int interval_ms,
                    int time_offset_s, char *errmsg, std::size_t errmsg_len) {
  rt_rinex_stop();

  if (!server) {
    set_error(errmsg, errmsg_len, "invalid rtserver");
    return false;
  }
  if (!rinex_path || !*rinex_path) {
    set_error(errmsg, errmsg_len, "rinex-path is empty");
    return false;
  }

  if (interval_ms < 1) interval_ms = 1;
  if (interval_ms > 100000) interval_ms = 100000;
  if (time_offset_s < 0) time_offset_s = 0;

  g_server = server;
  g_interval_ms = interval_ms;
  g_time_offset_s = time_offset_s;
  g_stop.store(0);

  gtime_t ts = {0}, te = {0};
  const int ret = readrnxt(rinex_path, 1, ts, te, 0.0, "", &g_obs, nullptr, nullptr);
  if (ret <= 0 || g_obs.n <= 0) {
    set_error(errmsg, errmsg_len, "read rinex obs failed: %s", rinex_path);
    clear_loaded_obs();
    return false;
  }

  sortobs(&g_obs);

  for (int i = 0, j = 0; i < g_obs.n; i = j) {
    for (j = i + 1; j < g_obs.n; ++j) {
      if (timediff(g_obs.data[j].time, g_obs.data[i].time) > DTTOL) break;
    }
    EpochBlock ep = {i, j - i};
    g_epochs.push_back(ep);
  }
  if (g_epochs.empty()) {
    set_error(errmsg, errmsg_len, "no valid obs epochs in rinex: %s", rinex_path);
    clear_loaded_obs();
    return false;
  }

  trace(3, "rt_rinex: loaded %s (nobs=%d, nepoch=%d)\n", rinex_path, g_obs.n, (int)g_epochs.size());

  try {
    g_thread = std::thread(inject_worker);
  } catch (...) {
    set_error(errmsg, errmsg_len, "create rinex inject thread failed");
    clear_loaded_obs();
    return false;
  }

  return true;
}

void rt_rinex_stop() {
  g_stop.store(1);

  if (g_thread.joinable()) {
    g_thread.join();
  }

  g_running.store(0);
  clear_loaded_obs();
  g_stop.store(0);
}

bool rt_rinex_is_running() { return g_running.load() != 0; }
