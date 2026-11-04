#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
viewer.py —— MLX90640 热斑实时监控（电脑端）

界面布局：
    左：32×24 彩色热力图 + 温度色标（白叉=最高温位置，红框=热斑包围盒）
    右上：实时数据面板（最低/最高/平均温度、温差、变化量、热斑像素、报警等级）
    右下：温度趋势曲线（最高温 / 平均温，最近 60 帧）

用法：
    python viewer.py            # 自动找有数据的串口
    python viewer.py COM6       # 也可命令行指定

数据格式（与 main.c 一致，勿改）：
    [FF FF][32][24] + 32×24×2 字节（每像素大端 uint16 = 温度×100）
    + 一行 ASCII： hot=(列,行) max=xx.xC med=xx.xC area=n box=(..) level=n
"""
import sys
import re
import time
from collections import deque

import numpy as np
import serial

# ================= CONFIG =================
PORT = "COM5"          # 首选串口；没有数据会自动换到有数据的那个
BAUD = 115200
W, H = 32, 24
SCALE_MIN = None       # 色标下限，None = 按本帧自动
SCALE_MAX = None       # 色标上限
CMAP = "jet"
HISTORY = 60           # 趋势曲线保留的帧数
# ==========================================

HEADER = bytes([0xFF, 0xFF, W, H])
N = W * H
FRAME_BIN = 4 + N * 2

# 面板配色
C_BG = "#F5F7FA"
C_BORDER = "#D3DAE6"
C_TITLE = "#2B3A55"
C_LABEL = "#5A6B85"
C_VALUE = "#16233A"
C_MIN = "#2E7BD6"
C_MAX = "#D63A34"
C_AVG = "#1E9E5A"
C_UP = "#D63A34"
C_DOWN = "#2E7BD6"
C_FLAT = "#8B98AB"


# ---------------- 协议解析 ----------------
def parse_ascii(line):
    info = {"hot": (None, None), "max": None, "med": None, "area": None,
            "box": None, "level": None}
    m = re.search(r"hot=\((\d+),(\d+)\)", line)
    if m:
        info["hot"] = (int(m.group(1)), int(m.group(2)))
    m = re.search(r"max=([-\d.]+)C", line)
    if m:
        info["max"] = float(m.group(1))
    m = re.search(r"med=([-\d.]+)C", line)
    if m:
        info["med"] = float(m.group(1))
    m = re.search(r"area=(\d+)", line)
    if m:
        info["area"] = int(m.group(1))
    m = re.search(r"box=\((\d+)\.\.(\d+),(\d+)\.\.(\d+)\)", line)
    if m:
        info["box"] = tuple(int(x) for x in m.groups())
    m = re.search(r"level=(\d+)", line)
    if m:
        info["level"] = int(m.group(1))
    return info


def decode_pixels(raw):
    a = np.frombuffer(raw, dtype=">u2").astype(np.float32) / 100.0
    return a.reshape(H, W)


def frames(ser):
    buf = bytearray()
    while True:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)
        while True:
            idx = buf.find(HEADER)
            if idx < 0:
                if len(buf) > 4:
                    del buf[:-4]
                break
            if idx + FRAME_BIN > len(buf):
                del buf[:idx]
                break
            nl = buf.find(b"\n", idx + FRAME_BIN)
            if nl < 0:
                del buf[:idx]
                break
            raw = bytes(buf[idx + 4: idx + FRAME_BIN])
            line = buf[idx + FRAME_BIN: nl].decode("latin-1", "replace")
            del buf[:nl + 1]
            info = parse_ascii(line)
            if info["hot"][0] is None:
                continue
            yield decode_pixels(raw), info


# ---------------- 串口选择 ----------------
def open_serial(preferred):
    """选真正有数据的串口。

    板子上有两个 USB 串口：
      CH340（描述 USB-SERIAL CH340）→ 接 UART0，数据在这里
      FTDI （描述 USB Serial Port）→ JTAG 下载口，没有数据
    CH340 插拔后 COM 号会变，所以直接探测"哪个口能收到数据"。
    """
    def open_and_probe(dev, seconds=1.5):
        try:
            s = serial.Serial(dev, BAUD, timeout=0.2)
        except Exception:
            return None
        t0 = time.time()
        while time.time() - t0 < seconds:
            try:
                if s.in_waiting:
                    return s
            except Exception:
                break
            time.sleep(0.05)
        try:
            s.close()
        except Exception:
            pass
        return None

    ser = open_and_probe(preferred)
    if ser is not None:
        return ser, preferred

    try:
        from serial.tools import list_ports
        ports = [p.device for p in list_ports.comports()]
    except Exception:
        ports = []
    others = [d for d in ports if d != preferred]
    others.sort(key=lambda d: 0 if "CH340" in d.upper() else 1)

    for dev in others:
        ser = open_and_probe(dev)
        if ser is not None:
            print(f"{preferred} 没有数据，改用 {dev}")
            return ser, dev

    for dev in [preferred] + others:
        try:
            ser = serial.Serial(dev, BAUD, timeout=0.2)
            print(f"注意：{dev} 已打开，但暂时没收到数据 —— 板子上的程序可能还没运行")
            return ser, dev
        except Exception:
            continue

    return None, preferred


# ---------------- 界面 ----------------
def build_ui():
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, FancyBboxPatch

    matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False

    fig = plt.figure(figsize=(12.2, 6.4), facecolor="white")
    fig.canvas.manager.set_window_title("MLX90640 热斑监控")

    gs = fig.add_gridspec(1, 2, width_ratios=[3.05, 1.0],
                          left=0.045, right=0.975, top=0.93, bottom=0.055,
                          wspace=0.10)
    ax = fig.add_subplot(gs[0, 0])
    gsr = gs[0, 1].subgridspec(2, 1, height_ratios=[2.55, 1.0], hspace=0.20)
    panel = fig.add_subplot(gsr[0, 0])
    trend = fig.add_subplot(gsr[1, 0])

    # ---- 左：热力图 ----
    im = ax.imshow(np.full((H, W), 20.0, dtype=np.float32), cmap=CMAP,
                   vmin=15, vmax=45, interpolation="nearest", aspect="auto")
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_edgecolor(C_BORDER)
        sp.set_linewidth(1.0)
    cb = fig.colorbar(im, ax=ax, fraction=0.045, pad=0.02)
    cb.set_label("温度 (℃)", color=C_LABEL, fontsize=10)
    cb.ax.tick_params(labelsize=9, colors=C_LABEL)
    cb.outline.set_edgecolor(C_BORDER)

    marker, = ax.plot([], [], marker="x", color="white",
                      markersize=13, markeredgewidth=2.2, zorder=5)
    box = Rectangle((0, 0), 1, 1, fill=False, edgecolor="#FF3B30",
                    linewidth=1.8, zorder=4)
    ax.add_patch(box)
    ax.set_title("热力图（32×24）", fontsize=12, color=C_TITLE,
                 pad=8, fontweight="bold")

    # ---- 右上：数据面板 ----
    panel.set_xlim(0, 1)
    panel.set_ylim(0, 1)
    panel.axis("off")
    panel.add_patch(FancyBboxPatch(
        (0.015, 0.015), 0.97, 0.97,
        boxstyle="round,pad=0.012,rounding_size=0.03",
        linewidth=1.2, edgecolor=C_BORDER, facecolor=C_BG,
        transform=panel.transAxes, zorder=0))

    panel.text(0.5, 0.935, "实时数据", ha="center", va="center",
               fontsize=12.5, color=C_TITLE, fontweight="bold",
               transform=panel.transAxes)
    panel.plot([0.10, 0.90], [0.885, 0.885], color=C_BORDER,
               linewidth=1.0, transform=panel.transAxes)

    rows = ["min", "max", "avg", "diff", "delta", "hot"]
    labels = {"min": "最低温度", "max": "最高温度", "avg": "平均温度",
              "diff": "温差(最高-最低)", "delta": "温度变化",
              "hot": "热斑像素"}
    y0, dy = 0.805, 0.118
    txt_label, txt_value = {}, {}
    for i, k in enumerate(rows):
        y = y0 - i * dy
        txt_label[k] = panel.text(0.075, y, labels[k], ha="left", va="center",
                                  fontsize=10, color=C_LABEL,
                                  transform=panel.transAxes)
        txt_value[k] = panel.text(0.925, y, "--", ha="right", va="center",
                                  fontsize=12.5, color=C_VALUE,
                                  fontweight="bold",
                                  transform=panel.transAxes)

    # 报警徽标（单独一行，比列表里的文字更醒目）
    badge = FancyBboxPatch((0.075, 0.030), 0.85, 0.082,
                           boxstyle="round,pad=0.008,rounding_size=0.02",
                           linewidth=0, facecolor="#E8ECF3",
                           transform=panel.transAxes, zorder=1)
    panel.add_patch(badge)
    txt_badge = panel.text(0.5, 0.0710, "等待数据…", ha="center", va="center",
                           fontsize=11.5, color=C_FLAT, fontweight="bold",
                           transform=panel.transAxes, zorder=2)

    # ---- 右下：趋势曲线 ----
    trend.set_facecolor("#FFFFFF")
    for sp in trend.spines.values():
        sp.set_edgecolor(C_BORDER)
        sp.set_linewidth(0.9)
    trend.tick_params(labelsize=8, colors=C_LABEL, length=2.5)
    trend.grid(True, color="#EDF0F5", linewidth=0.8)
    trend.set_axisbelow(True)
    trend.set_title("温度趋势（℃ / 最近 %d 帧）" % HISTORY, fontsize=10,
                    color=C_TITLE, pad=5)
    line_max, = trend.plot([], [], color=C_MAX, linewidth=1.4, label="最高")
    line_avg, = trend.plot([], [], color=C_AVG, linewidth=1.4, label="平均")
    trend.legend(loc="upper left", fontsize=8, frameon=False, ncol=2,
                 handlelength=1.2, columnspacing=0.9)

    return dict(fig=fig, ax=ax, panel=panel, trend=trend, im=im, cb=cb,
                marker=marker, box=box, lbl=txt_label, val=txt_value,
                badge=badge, badge_txt=txt_badge,
                line_max=line_max, line_avg=line_avg)


def set_value(ui, key, text, color=C_VALUE):
    ui["val"][key].set_text(text)
    ui["val"][key].set_color(color)


def new_state():
    """每帧更新需要的状态。单独抽出来便于测试。"""
    return {"prev_max": None, "prev_med": None,
            "hist_max": deque(maxlen=HISTORY),
            "hist_avg": deque(maxlen=HISTORY)}


def apply_frame(ui, temp, info, state):
    """把一帧数据画到界面上。"""
    t_min = float(temp.min())
    t_max = float(temp.max())
    t_avg = float(temp.mean())
    t_diff = t_max - t_min

    # 色标
    vmin = SCALE_MIN if SCALE_MIN is not None else t_min
    vmax = SCALE_MAX if SCALE_MAX is not None else t_max
    if vmax - vmin < 0.5:
        vmin -= 1.0
        vmax += 1.0
    ui["im"].set_data(temp)
    ui["im"].set_clim(vmin, vmax)

    # 最高温标记 + 热斑框
    c, r = info["hot"]
    ui["marker"].set_data([c], [r])
    if info["box"]:
        c0, c1, r0, r1 = info["box"]
        ui["box"].set_bounds(c0 - 0.5, r0 - 0.5, c1 - c0 + 1, r1 - r0 + 1)
        ui["box"].set_visible(True)
    else:
        ui["box"].set_visible(False)

    # 数值面板
    set_value(ui, "min", f"{t_min:.1f} ℃", C_MIN)
    set_value(ui, "max", f"{t_max:.1f} ℃", C_MAX)
    set_value(ui, "avg", f"{t_avg:.1f} ℃", C_AVG)
    set_value(ui, "diff", f"{t_diff:.1f} ℃", C_VALUE)

    if state["prev_max"] is None:
        set_value(ui, "delta", "--", C_FLAT)
    else:
        d = t_max - state["prev_max"]
        if d > 0.05:
            set_value(ui, "delta", f"{d:+.1f} ℃  ▲", C_UP)
        elif d < -0.05:
            set_value(ui, "delta", f"{d:+.1f} ℃  ▼", C_DOWN)
        else:
            set_value(ui, "delta", "0.0 ℃  —", C_FLAT)
    state["prev_max"] = t_max
    state["prev_med"] = info["med"]

    area = info["area"] if info["area"] is not None else 0
    set_value(ui, "hot", f"{area}", C_VALUE)

    # 报警等级（徽标显示，比列表文字更醒目）
    lv = info["level"] or 0
    badge_text = {0: "状态正常", 1: "一般热斑", 2: "严重热斑"}.get(lv, f"L{lv}")
    badge_bg = {0: "#E6F4EA", 1: "#FFF1DB", 2: "#FDE7E7"}.get(lv, "#E8ECF3")
    badge_fg = {0: "#1E7B3C", 1: "#B86A00", 2: "#C0392B"}.get(lv, C_FLAT)
    ui["badge"].set_facecolor(badge_bg)
    ui["badge"].set_edgecolor(badge_fg)
    ui["badge"].set_linewidth(1.0)
    ui["badge_txt"].set_text(badge_text)
    ui["badge_txt"].set_color(badge_fg)

    # 趋势
    state["hist_max"].append(t_max)
    state["hist_avg"].append(t_avg)
    x = list(range(len(state["hist_max"])))
    ui["line_max"].set_data(x, list(state["hist_max"]))
    ui["line_avg"].set_data(x, list(state["hist_avg"]))
    lo = min(min(state["hist_max"]), min(state["hist_avg"]))
    hi = max(max(state["hist_max"]), max(state["hist_avg"]))
    pad = max(0.5, (hi - lo) * 0.15)
    ui["trend"].set_xlim(0, max(1, len(x) - 1))
    ui["trend"].set_ylim(lo - pad, hi + pad)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("缺少 matplotlib，请先运行： pip install matplotlib numpy pyserial")
        return

    ser, port = open_serial(port)
    if ser is None:
        print("所有串口都打不开。请确认：")
        print("  1) 板子已上电，CH340 驱动正常")
        print("  2) 串口助手（SSCOM 等）已关闭 —— 一个串口同时只能一个程序用")
        return
    print(f"已打开 {port} @ {BAUD}，等待 ZYNQ 数据……（Ctrl+C 退出）")

    ui = build_ui()
    plt.ion()
    plt.show(block=False)

    state = new_state()
    for temp, info in frames(ser):
        apply_frame(ui, temp, info, state)
        ui["fig"].canvas.draw_idle()
        plt.pause(0.01)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已退出")
    except Exception:
        import traceback
        traceback.print_exc()
        input("\n按回车键退出…")
