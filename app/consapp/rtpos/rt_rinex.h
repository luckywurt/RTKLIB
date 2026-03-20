#pragma once

#include <cstddef>

class RtServer;

bool rt_rinex_start(RtServer *server, const char *rinex_path, int interval_ms,
                    int time_offset_s, char *errmsg, std::size_t errmsg_len);
void rt_rinex_stop();
bool rt_rinex_is_running();
