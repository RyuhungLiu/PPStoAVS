#!/usr/bin/env python3
"""合成首次烧录用的完整镜像：Bootloader + 程序 + 程序头页（布局见 src/flash_io.h、src/iap.h）。

用法：python scripts/mkfull.py [--bl build_bl/PPStoAVS-BL.bin] [--app build/PPStoAVS.bin] [--out build/PPStoAVS-full]
输出 <out>.hex（Intel HEX，给 WCH-LinkUtility）与 <out>.bin（0x0000 起，补 0xFF 到程序头页结束）。
程序头与 Bootloader 的校验一致（CRC-32/IEEE，即 zlib.crc32）；没有程序头的镜像会让 Bootloader 一直停在 BL 模式。
"""
import argparse
import os
import re
import struct
import zlib

BL_SIZE = 0x1800
APP_BASE = 0x1800
APP_MAX = 0xED00 - APP_BASE      # Flash 布局 2（v0.14.0 起）
HDR_OFFSET = 0xED80
PAGE = 128
HDR_MAGIC = 0x31505041  # 'APP1'


def fw_version(path=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "board.h")):
    m = re.search(r"#define\s+FW_VERSION\s+(0x[0-9A-Fa-f]+)", open(path, encoding="utf-8").read())
    return int(m.group(1), 16)


def header_page(app, version):
    head = struct.pack("<IIIHH", HDR_MAGIC, len(app), zlib.crc32(app) & 0xFFFFFFFF, version, 0)
    page = head + struct.pack("<I", zlib.crc32(head) & 0xFFFFFFFF)
    return page + b"\xFF" * (PAGE - len(page))


def ihex(segments):
    lines = []
    for base, data in segments:
        for i in range(0, len(data), 16):
            chunk = data[i:i + 16]
            rec = bytes([len(chunk), ((base + i) >> 8) & 0xFF, (base + i) & 0xFF, 0]) + chunk
            lines.append(":" + rec.hex().upper() + "%02X" % ((-sum(rec)) & 0xFF))
    lines.append(":00000001FF")
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bl", default="build_bl/PPStoAVS-BL.bin")
    ap.add_argument("--app", default="build/PPStoAVS.bin")
    ap.add_argument("--out", default="build/PPStoAVS-full")
    a = ap.parse_args()

    bl = open(a.bl, "rb").read()
    app = open(a.app, "rb").read()
    if len(bl) > BL_SIZE:
        raise SystemExit("Bootloader 超过 %d 字节" % BL_SIZE)
    if len(app) > APP_MAX or len(app) % 4:
        raise SystemExit("程序大小不合法：%d（最大 %d，须为 4 的倍数）" % (len(app), APP_MAX))

    hdr = header_page(app, fw_version())
    image = bytearray(b"\xFF" * (HDR_OFFSET + PAGE))
    image[0:len(bl)] = bl
    image[APP_BASE:APP_BASE + len(app)] = app
    image[HDR_OFFSET:HDR_OFFSET + PAGE] = hdr

    open(a.out + ".bin", "wb").write(image)
    open(a.out + ".hex", "w").write(ihex([(0, bl), (APP_BASE, app), (HDR_OFFSET, hdr)]))
    print("BL %d B, APP %d B, version 0x%04X, crc32 0x%08X -> %s.hex/.bin" % (len(bl), len(app), fw_version(), zlib.crc32(app) & 0xFFFFFFFF, a.out))


if __name__ == "__main__":
    main()
