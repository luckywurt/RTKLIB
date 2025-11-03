#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RTKLIB telnet 监控页面（Tkinter）
- 8 个命令页：status、solution、satellite、stream、observ、navidata、ssr、error
- 每页独立连接/轮询（开始/停止），支持“开始全部 / 停止全部”
- 覆盖显示：status、satellite、observ、navidata、stream、ssr
- 追加显示：solution、error
- 独立“日志”页记录运行日志
- 连接：172.16.0.2:9000，密码：admin（顶部可改）
"""

import telnetlib
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk, messagebox

# =============== Telnet 会话封装 ===============
class RTKLibTelnetSession:
    def __init__(self, host, port, password, on_data, on_status):
        """
        host/port/password : 连接参数
        on_data(str)       : 收到数据回调（追加/覆盖到UI）
        on_status(str)     : 状态回调（显示连接/错误/发送日志）
        """
        self.host = host
        self.port = port
        self.password = password
        self.on_data = on_data
        self.on_status = on_status

        self._tn = None
        self._reader_thread = None
        self._stop_event = threading.Event()
        self._alive = False

    def _dump_bytes_to_ui(self, b: bytes):
        if not b:
            return
        try:
            self.on_data(b.decode("utf-8", errors="ignore"))
        except Exception:
            pass

    def connect_and_login(self, timeout=12.0):
        """
        建立连接并登录：兼容两种情况
        1) 出现 password 提示 -> 发送密码 -> 等待 rtpos>
        2) 直接出现 rtpos> （无密码）
        """
        try:
            self._tn = telnetlib.Telnet(self.host, self.port, timeout=timeout)
            self.on_status(f"[INFO] 连接 {self.host}:{self.port} 成功，等待 password 或 rtpos> 提示...\n")

            idx, match, text = self._tn.expect([b"password", b"rtpos> "], timeout=timeout)
            self._dump_bytes_to_ui(text or b"")

            if idx == 0:
                # 需要密码
                self.on_status("[INFO] 检测到 password，发送口令...\n")
                self._tn.write((self.password + "\r\n").encode("ascii"))
                idx2, match2, text2 = self._tn.expect([b"rtpos> ", b"invalid password"], timeout=timeout)
                self._dump_bytes_to_ui(text2 or b"")
                if idx2 == 0:
                    self._alive = True
                    self.on_status("[OK] 登录成功（已获得 rtpos> 提示符）。\n")
                    return True
                elif idx2 == 1:
                    self.on_status("[ERR] 登录失败：invalid password。\n")
                    self.close()
                    return False
                else:
                    self.on_status("[ERR] 登录失败：未等到 rtpos> 提示符。\n")
                    self.close()
                    return False

            elif idx == 1:
                # 无密码直接进入
                self._alive = True
                self.on_status("[OK] 无密码登录，已获得 rtpos> 提示符。\n")
                return True
            else:
                self.on_status("[ERR] 登录失败：未见 password / rtpos>。\n")
                self.close()
                return False

        except Exception as e:
            self.on_status(f"[ERR] 连接/登录异常：{e}\n")
            self.close()
            return False

    def start_reader(self):
        """启动后台读线程"""
        if not self._tn:
            return
        self._stop_event.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def send_cycle_cmd(self, cmd, cycle):
        """发送循环命令，如 'status 1' """
        if not self._tn:
            self.on_status("[ERR] 连接未建立，无法发送命令。\n")
            return
        full = f"{cmd} {int(cycle)}\r\n"
        try:
            self._tn.write(full.encode("ascii"))
            self.on_status(f"[SEND] {full}")
        except Exception as e:
            self.on_status(f"[ERR] 发送命令失败：{e}\n")

    def stop_cycle(self):
        """发送 Ctrl-C 停止远端循环"""
        if not self._tn:
            return
        try:
            self._tn.write(b"\x03")  # Ctrl-C
            time.sleep(0.2)
            self._tn.write(b"\r\n")
        except Exception as e:
            self.on_status(f"[WARN] 停止循环时异常：{e}\n")

    def close(self):
        """关闭连接"""
        self._stop_event.set()
        try:
            if self._tn:
                try:
                    self._tn.write(b"exit\r\n")
                    time.sleep(0.1)
                except Exception:
                    pass
                self._tn.close()
        finally:
            self._tn = None
            self._alive = False

    def _reader_loop(self):
        """持续读取远端输出"""
        try:
            while not self._stop_event.is_set():
                try:
                    data = self._tn.read_very_eager() if self._tn else b""
                except EOFError:
                    self.on_status("[INFO] 远端主动关闭连接。\n")
                    break
                except Exception as e:
                    self.on_status(f"[ERR] 读取异常：{e}\n")
                    break

                if data:
                    text = data.decode("utf-8", errors="ignore")
                    if text:
                        self.on_data(text)
                else:
                    time.sleep(0.05)
        finally:
            self._alive = False
            self.on_status("[INFO] 读线程已结束。\n")


# =============== 日志标签页 ===============
class LogTab(ttk.Frame):
    def __init__(self, master):
        super().__init__(master)
        top = ttk.Frame(self)
        top.pack(fill="x", padx=8, pady=6)
        ttk.Button(top, text="清空日志", command=self.clear).pack(side="left")

        mid = ttk.Frame(self)
        mid.pack(fill="both", expand=True, padx=8, pady=6)
        self.text = tk.Text(mid, wrap="none", height=20)
        self.text.configure(font=("Consolas", 10))
        vs = ttk.Scrollbar(mid, orient="vertical", command=self.text.yview)
        self.text.configure(yscrollcommand=vs.set)
        self.text.pack(side="left", fill="both", expand=True)
        vs.pack(side="left", fill="y")

        self.queue = queue.Queue()
        self._running = True
        self._pump()

    def _pump(self):
        try:
            while True:
                s = self.queue.get_nowait()
                self.text.insert("end", s)
                self.text.see("end")
        except queue.Empty:
            pass
        if self._running:
            self.after(80, self._pump)

    def log(self, s: str):
        self.queue.put(s)

    def clear(self):
        self.text.delete("1.0", "end")


# =============== 命令标签页 ===============
class CommandTab(ttk.Frame):
    def __init__(self, master, cmd_name, host_var, port_var, pwd_var, append_mode, global_logger):
        """
        cmd_name      : RTKLIB console 命令名
        append_mode   : True=追加显示；False=覆盖显示
        global_logger : 调用 log_tab.log 的函数，用于写全局日志
        """
        super().__init__(master)
        self.cmd_name = cmd_name
        self.host_var = host_var
        self.port_var = port_var
        self.pwd_var = pwd_var
        self.append_mode = append_mode
        self.global_logger = global_logger

        # 顶栏控件
        top = ttk.Frame(self)
        top.pack(fill="x", padx=8, pady=6)

        ttk.Label(top, text=f"命令：{cmd_name}").pack(side="left")
        ttk.Label(top, text=" 轮询周期(s)：").pack(side="left")
        self.cycle_var = tk.StringVar(value="1")
        ttk.Entry(top, width=6, textvariable=self.cycle_var).pack(side="left", padx=(0, 8))

        self.btn_start = ttk.Button(top, text="开始轮询", command=self.start_poll)
        self.btn_start.pack(side="left", padx=4)
        self.btn_stop = ttk.Button(top, text="停止", command=self.stop_poll, state="disabled")
        self.btn_stop.pack(side="left", padx=4)
        self.btn_clear = ttk.Button(top, text="清空", command=self.clear_output)
        self.btn_clear.pack(side="left", padx=4)

        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(self, textvariable=self.status_var, foreground="#555").pack(anchor="w", padx=8)

        # 文本区
        mid = ttk.Frame(self)
        mid.pack(fill="both", expand=True, padx=8, pady=6)
        self.text = tk.Text(mid, wrap="none", height=20)
        self.text.configure(font=("Consolas", 10))
        vs = ttk.Scrollbar(mid, orient="vertical", command=self.text.yview)
        self.text.configure(yscrollcommand=vs.set)
        self.text.pack(side="left", fill="both", expand=True)
        vs.pack(side="left", fill="y")

        # Telnet 会话 & 队列
        self.session = None
        self.queue = queue.Queue()
        self._queue_pumper_running = False

    # ------ UI 写入 ------
    def _append_text(self, s: str):
        if self.append_mode:
            # 追加模式
            self.text.insert("end", s)
        else:
            # 覆盖模式：每次清空后写入最新块
            self.text.delete("1.0", "end")
            self.text.insert("end", s)
        self.text.see("end")

    def _on_data(self, s: str):
        self.queue.put(("data", s))

    def _on_status(self, s: str):
        # 状态既写入本页，也写入全局日志
        self.queue.put(("status", s))
        try:
            self.status_var.set(s.strip())
        except Exception:
            pass
        if self.global_logger:
            self.global_logger(f"[{self.cmd_name}] {s}")

    def _pump_queue(self):
        try:
            while True:
                kind, payload = self.queue.get_nowait()
                if kind == "data":
                    self._append_text(payload)
                elif kind == "status":
                    # 也在页面中显示状态（不清空）
                    self.text.insert("end", payload)
                    self.text.see("end")
        except queue.Empty:
            pass
        if self._queue_pumper_running:
            self.after(80, self._pump_queue)

    # ------ 控制 ------
    def start_poll(self):
        host = self.host_var.get().strip()
        pwd = self.pwd_var.get().strip()
        try:
            port = int(self.port_var.get().strip())
        except Exception:
            messagebox.showerror("错误", "端口必须是整数")
            return
        try:
            cyc = int(self.cycle_var.get().strip())
            if cyc <= 0:
                raise ValueError
        except Exception:
            messagebox.showerror("错误", "轮询周期必须是正整数秒")
            return

        # 若已有会话，先停
        if self.session:
            self.stop_poll()

        self.btn_start.config(state="disabled")
        self.btn_stop.config(state="normal")

        # 建立新会话
        self.session = RTKLibTelnetSession(
            host, port, pwd, on_data=self._on_data, on_status=self._on_status
        )

        # 启动队列泵
        if not self._queue_pumper_running:
            self._queue_pumper_running = True
            self._pump_queue()

        def worker():
            ok = self.session.connect_and_login()
            if ok:
                self.session.start_reader()
                self.session.send_cycle_cmd(self.cmd_name, cyc)
            else:
                self._on_status("[ERR] 无法开始轮询。\n")
                self.btn_start.config(state="normal")
                self.btn_stop.config(state="disabled")

        threading.Thread(target=worker, daemon=True).start()

    def stop_poll(self):
        self.btn_stop.config(state="disabled")
        try:
            if self.session:
                self.session.stop_cycle()
                time.sleep(0.2)
                self.session.close()
                self._on_status("[INFO] 已停止并断开连接。\n")
        finally:
            self.btn_start.config(state="normal")
            self.session = None

    def clear_output(self):
        self.text.delete("1.0", "end")

    # 便于“开始全部/停止全部”判断
    def is_running(self):
        return self.session is not None


# =============== 主窗体 ===============
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("RTKLIB 实时解算（telnet）监控")
        self.geometry("1100x700")

        # 连接参数栏
        top = ttk.LabelFrame(self, text="连接参数")
        top.pack(fill="x", padx=8, pady=8)

        ttk.Label(top, text="Host:").pack(side="left", padx=(8, 4))
        self.host_var = tk.StringVar(value="172.16.0.2")
        ttk.Entry(top, width=16, textvariable=self.host_var).pack(side="left")

        ttk.Label(top, text="Port:").pack(side="left", padx=(12, 4))
        self.port_var = tk.StringVar(value="9000")
        ttk.Entry(top, width=8, textvariable=self.port_var).pack(side="left")

        ttk.Label(top, text="Password:").pack(side="left", padx=(12, 4))
        self.pwd_var = tk.StringVar(value="admin")
        ttk.Entry(top, width=16, show="*", textvariable=self.pwd_var).pack(side="left")

        ttk.Label(top, text=" ").pack(side="left", padx=4)
        ttk.Button(top, text="开始全部", command=self.start_all).pack(side="left", padx=4)
        ttk.Button(top, text="停止全部", command=self.stop_all).pack(side="left", padx=4)

        ttk.Label(top, text="（说明：每个标签页独立连接",
                  foreground="#666").pack(side="left", padx=12)

        # Notebook
        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        # 全局日志页
        self.log_tab = LogTab(self.nb)
        self.log = self.log_tab.log  # 简化调用

        # 命令标签页顺序（按需求）
        commands = [
            ("status",   False),  # 覆盖
            ("solution", True),   # 追加
            ("satellite",False),  # 覆盖
            ("stream",   False),  # 覆盖
            ("observ",   False),  # 覆盖
            ("navidata", False),  # 覆盖
            ("ssr",      False),  # 覆盖
            ("error",    True),   # 追加
        ]

        self.tabs = []  # 存 CommandTab

        for cmd_name, append_mode in commands:
            tab = CommandTab(self.nb, cmd_name,
                             host_var=self.host_var,
                             port_var=self.port_var,
                             pwd_var=self.pwd_var,
                             append_mode=append_mode,
                             global_logger=self.log)
            self.nb.add(tab, text=cmd_name)
            self.tabs.append(tab)

        # 最后再添加日志标签页
        self.nb.add(self.log_tab, text="日志")

    # ====== 批量控制 ======
    def start_all(self):
        self.log("[SYS] 执行开始全部。\n")
        # 顺序启动即可（各自独立连接）
        for tab in self.tabs:
            try:
                if not tab.is_running():
                    tab.start_poll()
            except Exception as e:
                self.log(f"[SYS] 启动 {tab.cmd_name} 失败：{e}\n")

    def stop_all(self):
        self.log("[SYS] 执行停止全部。\n")
        for tab in self.tabs:
            try:
                if tab.is_running():
                    tab.stop_poll()
            except Exception as e:
                self.log(f"[SYS] 停止 {tab.cmd_name} 失败：{e}\n")


if __name__ == "__main__":
    App().mainloop()
