#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from typing import List, Tuple, Optional

# Speed of light / 1000 => meters per millisecond
C_M_PER_MS = 299792.458

# -------------------- Bit helpers --------------------

def getbitu(buf: bytes, pos: int, length: int) -> int:
    """Unsigned bits, big-endian bit numbering within bytes (MSB first)."""
    val = 0
    for _ in range(length):
        byte = buf[pos >> 3]
        bit = 7 - (pos & 7)
        val = (val << 1) | ((byte >> bit) & 1)
        pos += 1
    return val

def getbits(buf: bytes, pos: int, length: int) -> int:
    """Signed bits (two's complement)."""
    u = getbitu(buf, pos, length)
    sign = 1 << (length - 1)
    return u - (1 << length) if (u & sign) else u

def bits_to_ids(mask: int, width: int) -> List[int]:
    """Convert MSB-first mask to 1-based IDs (e.g., satID 1..64, sigID 1..32)."""
    ids = []
    for i in range(width):
        if (mask >> (width - 1 - i)) & 1:
            ids.append(i + 1)
    return ids

# -------------------- RTCM frame helpers --------------------

CRC24Q_TABLE = None

def _crc24q_init_table():
    global CRC24Q_TABLE
    poly = 0x1864CFB
    table = []
    for i in range(256):
        crc = i << 16
        for _ in range(8):
            crc = ((crc << 1) ^ poly) if (crc & 0x800000) else (crc << 1)
            crc &= 0xFFFFFF
        table.append(crc)
    CRC24Q_TABLE = table

def crc24q(data: bytes) -> int:
    global CRC24Q_TABLE
    if CRC24Q_TABLE is None:
        _crc24q_init_table()
    crc = 0
    for b in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ CRC24Q_TABLE[((crc >> 16) ^ b) & 0xFF]
    return crc & 0xFFFFFF

def iter_rtcm3_frames(path: str):
    with open(path, "rb") as f:
        b = f.read()
    i = 0
    n = len(b)
    while i + 6 <= n:
        if b[i] != 0xD3:
            i += 1
            continue
        if i + 3 > n:
            break
        length = ((b[i + 1] & 0x03) << 8) | b[i + 2]
        frame_len = 3 + length + 3
        if i + frame_len > n:
            break
        frame = b[i:i + frame_len]
        payload = frame[3:3 + length]
        crc_recv = (frame[-3] << 16) | (frame[-2] << 8) | frame[-1]
        crc_calc = crc24q(frame[:-3])
        if crc_recv == crc_calc:
            yield payload
        i += frame_len

# -------------------- MSM parsing --------------------

def msm_sys_from_type(msg_type: int) -> Optional[str]:
    if 1070 <= msg_type <= 1077:
        return "GPS"
    if 1080 <= msg_type <= 1087:
        return "GLO"
    if 1090 <= msg_type <= 1097:
        return "GAL"
    if 1120 <= msg_type <= 1127:
        return "BDS"
    return None

def is_msm_obs(msg_type: int) -> bool:
    sysname = msm_sys_from_type(msg_type)
    if sysname is None:
        return False
    msm = msg_type % 10
    return msm in (4, 5, 6, 7)

def _invalid_pattern_u(uval: int, bitlen: int) -> bool:
    # In MSM, many "invalid" patterns correspond to the most-negative value, i.e., sign bit set and all others 0.
    # e.g., 15-bit: 0x4000, 22-bit: 0x200000, 20-bit: 0x80000, 24-bit: 0x800000, 14-bit: 0x2000, 15-bit: 0x4000
    return uval == (1 << (bitlen - 1))

def parse_msm_summary(payload: bytes) -> None:
    """
    Print one-line MSM summary and then dump all decoded observation cells
    (PR/PH/Lock/Half/CNR/PRR where available).
    """
    if len(payload) < 6:
        return

    msg_type = getbitu(payload, 0, 12)
    if not is_msm_obs(msg_type):
        return

    sysname = msm_sys_from_type(msg_type)
    msm = msg_type % 10

    pos = 12
    staid = getbitu(payload, pos, 12); pos += 12

    # Epoch time:
    # - GLONASS MSM uses 3-bit day-of-week + 27-bit ms-of-day (commonly handled this way in decoders)
    # - others use 30-bit ms-of-week
    if sysname == "GLO":
        dow = getbitu(payload, pos, 3)
        tod = getbitu(payload, pos + 3, 27)
        epoch = (dow << 27) | tod
    else:
        epoch = getbitu(payload, pos, 30)
    pos += 30

    sync = getbitu(payload, pos, 1); pos += 1
    iod = getbitu(payload, pos, 3); pos += 3

    pos += 7  # reserved
    csi = getbitu(payload, pos, 2); pos += 2
    eci = getbitu(payload, pos, 2); pos += 2
    smind = getbitu(payload, pos, 1); pos += 1
    smint = getbitu(payload, pos, 3); pos += 3

    sat_mask = getbitu(payload, pos, 64); pos += 64
    sig_mask = getbitu(payload, pos, 32); pos += 32

    sats = bits_to_ids(sat_mask, 64)
    sigs = bits_to_ids(sig_mask, 32)
    nsat = len(sats)
    nsig = len(sigs)

    cell_mask_bits = nsat * nsig
    cell_mask = getbitu(payload, pos, cell_mask_bits); pos += cell_mask_bits
    ncell = bin(cell_mask).count("1")

    # Required minimal summary line (same style as you asked)
    print(f"RTCM {msg_type} staid={staid} epoch={epoch} nsat={nsat} nsig={nsig} ncell={ncell} sync={sync} sys={sysname}")

    # ---- Satellite data ----
    # MSM4/5/6/7: DF397(8)*Nsat always present
    df397 = []
    for _ in range(nsat):
        df397.append(getbitu(payload, pos, 8))
        pos += 8

    ext_sat_info = None
    if msm in (5, 7):
        ext_sat_info = []
        for _ in range(nsat):
            ext_sat_info.append(getbitu(payload, pos, 4))
            pos += 4

    df398 = []
    for _ in range(nsat):
        df398.append(getbitu(payload, pos, 10))
        pos += 10

    df399 = None
    if msm in (5, 7):
        df399 = []
        for _ in range(nsat):
            df399.append(getbits(payload, pos, 14))
            pos += 14

    # Build cell list in the same order as cell mask definition:
    # packed by columns: sat-major, then sig-minor (ascending IDs)
    cell_pairs: List[Tuple[int, int]] = []
    # cell_mask is MSB-first; easiest: iterate bit index 0..cell_mask_bits-1
    # where bit 0 corresponds to (sat0,sig0).
    for si in range(nsat):
        for qi in range(nsig):
            bit_index = si * nsig + qi
            bit = (cell_mask >> (cell_mask_bits - 1 - bit_index)) & 1
            if bit:
                cell_pairs.append((si, qi))

    # ---- Signal data (per cell) ----
    # DF400/DF405 fine pseudorange
    fine_pr = []
    if msm in (4, 5):
        for _ in range(ncell):
            u = getbitu(payload, pos, 15)
            fine_pr.append(None if _invalid_pattern_u(u, 15) else getbits(payload, pos, 15))
            pos += 15
    elif msm in (6, 7):
        for _ in range(ncell):
            u = getbitu(payload, pos, 20)
            fine_pr.append(None if _invalid_pattern_u(u, 20) else getbits(payload, pos, 20))
            pos += 20

    # DF401/DF406 fine phaserange
    fine_ph = []
    if msm in (4, 5):
        for _ in range(ncell):
            u = getbitu(payload, pos, 22)
            fine_ph.append(None if _invalid_pattern_u(u, 22) else getbits(payload, pos, 22))
            pos += 22
    elif msm in (6, 7):
        for _ in range(ncell):
            u = getbitu(payload, pos, 24)
            fine_ph.append(None if _invalid_pattern_u(u, 24) else getbits(payload, pos, 24))
            pos += 24

    # DF402/DF407 lock time indicator
    lock = []
    if msm in (4, 5):
        for _ in range(ncell):
            lock.append(getbitu(payload, pos, 4))
            pos += 4
    elif msm in (6, 7):
        for _ in range(ncell):
            lock.append(getbitu(payload, pos, 10))
            pos += 10

    # DF420 half-cycle ambiguity indicator
    half = []
    for _ in range(ncell):
        half.append(getbitu(payload, pos, 1))
        pos += 1

    # DF403/DF408 CNR
    cnr = []
    if msm in (4, 5):
        for _ in range(ncell):
            v = getbitu(payload, pos, 6)
            cnr.append(None if v == 0 else float(v))  # 1 dB-Hz steps
            pos += 6
    elif msm in (6, 7):
        for _ in range(ncell):
            v = getbitu(payload, pos, 10)
            cnr.append(None if v == 0 else (v * (2 ** -4)))  # 0.0625 dB-Hz
            pos += 10

    # DF404 fine phaserange rate (MSM5/7 only)
    fine_prr = None
    if msm in (5, 7):
        fine_prr = []
        for _ in range(ncell):
            u = getbitu(payload, pos, 15)
            fine_prr.append(None if _invalid_pattern_u(u, 15) else getbits(payload, pos, 15))
            pos += 15

    # ---- Print decoded observations ----
    # Reconstruct PR/PH/PRR following MSM reconstruction formulas.
    # Standard precision (MSM4/5): PR uses 2^-24, PH uses 2^-29
    # High precision (MSM6/7):     PR uses 2^-29, PH uses 2^-31
    pr_scale = 2 ** (-24) if msm in (4, 5) else 2 ** (-29)
    ph_scale = 2 ** (-29) if msm in (4, 5) else 2 ** (-31)

    # Keep output compact but complete (raw + reconstructed)
    for k, (si, qi) in enumerate(cell_pairs):
        sat_id = sats[si]
        sig_id = sigs[qi]

        nms = df397[si]
        rr_mod = df398[si]

        # invalid sat rough range?
        if nms == 255:
            pr_m = None
            ph_m = None
        else:
            # Pseudorange
            if fine_pr[k] is None:
                pr_m = None
            else:
                pr_m = C_M_PER_MS * (nms + (rr_mod / 1024.0) + (fine_pr[k] * pr_scale))

            # Phaserange
            if fine_ph[k] is None:
                ph_m = None
            else:
                ph_m = C_M_PER_MS * (nms + (rr_mod / 1024.0) + (fine_ph[k] * ph_scale))

        # PhaseRangeRate (only MSM5/7)
        prr = None
        if msm in (5, 7) and df399 is not None and fine_prr is not None:
            rough_rate = df399[si]
            if _invalid_pattern_u((rough_rate & 0x3FFF), 14) or fine_prr[k] is None:
                prr = None
            else:
                prr = float(rough_rate) + 0.0001 * float(fine_prr[k])  # m/s

        sat_tag = {"GPS":"G", "GLO":"R", "GAL":"E", "BDS":"C"}[sysname]
        sat_str = f"{sat_tag}{sat_id:02d}"

        # Print one cell per line
        # Raw fields included so you can check presence of lock/half etc.
        if prr is None:
            print(
                f"  {sat_str} sig={sig_id:02d} "
                f"PR={('%.3f' % pr_m) if pr_m is not None else 'NA'}m "
                f"PH={('%.3f' % ph_m) if ph_m is not None else 'NA'}m "
                f"Lock={lock[k]} Half={half[k]} "
                f"CNR={('%.4f' % cnr[k]) if cnr[k] is not None else 'NA'} "
                f"(raw: Nms={nms} Mod={rr_mod} fPR={fine_pr[k]} fPH={fine_ph[k]})"
            )
        else:
            print(
                f"  {sat_str} sig={sig_id:02d} "
                f"PR={('%.3f' % pr_m) if pr_m is not None else 'NA'}m "
                f"PH={('%.3f' % ph_m) if ph_m is not None else 'NA'}m "
                f"PRR={('%.4f' % prr) if prr is not None else 'NA'}m/s "
                f"Lock={lock[k]} Half={half[k]} "
                f"CNR={('%.4f' % cnr[k]) if cnr[k] is not None else 'NA'} "
                f"(raw: Nms={nms} Mod={rr_mod} fPR={fine_pr[k]} fPH={fine_ph[k]} fPRR={fine_prr[k]})"
            )

def main():
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <rtcm3_binary_file>")
        sys.exit(2)

    path = sys.argv[1]
    for payload in iter_rtcm3_frames(path):
        # Only satellite observation MSM for 4 systems
        msg_type = getbitu(payload, 0, 12)
        if is_msm_obs(msg_type):
            parse_msm_summary(payload)

if __name__ == "__main__":
    main()
