#!/usr/bin/env python3
# Copyright(c) 2022 Intel Corporation. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
import os
import sys
import struct
import logging
import asyncio
import time
import subprocess
import ctypes
import mmap
import argparse

logging.basicConfig()
log = logging.getLogger("cavs-fw")
log.setLevel(logging.INFO)

PAGESZ = 4096
HUGEPAGESZ = 2 * 1024 * 1024
HUGEPAGE_FILE = "/dev/hugepages/cavs-fw-dma.tmp"

# Log is in the fourth window, they appear in 128k regions starting at 512k
WINSTREAM_OFFSET = (512 + (3 * 128)) * 1024

class HDAStream:
    # creates an hda stream with at 2 buffers of buf_len
    def __init__(self, stream_id, buf_len):
        self.stream_id = stream_id
        self.buf_len = buf_len
        self.base = hdamem + 0x0080 + (stream_id * 0x20)
        log.info("Mapping registers for hda stream")

        self.spib = Regs(hdamem + 0x0700)
        self.spib.SPBFCH  = 0x00
        self.spib.SPBFCTL = 0x04
        self.spib.SPIB = 0x08 + stream_id*0x80
        self.spib.freeze()

        self.regs = Regs(self.base)
        self.regs.CTL  = 0x00
        self.regs.STS  = 0x03
        self.regs.LPIB = 0x04
        self.regs.CBL  = 0x08
        self.regs.LVI  = 0x0c
        self.regs.FIFOW = 0x0e
        self.regs.FIFOS = 0x10
        self.regs.FMT = 0x12
        self.regs.FIFOL= 0x14
        self.regs.BDPL = 0x18
        self.regs.BDPU = 0x1c
        self.regs.freeze()

        self.dbg0 = Regs(hdamem + 0x0084 + (0x20*stream_id))
        self.dbg0.DPIB = 0x00
        self.dbg0.EFIFOS = 0x10
        self.dbg0.freeze()

        self.debug()
        log.info("Resetting hda stream %d at 0x%x", self.stream_id, self.base)
        self.reset()
        self.debug()

        log.info("Enable SPIB and set position")
        self.spib.SPBFCTL = (1 << stream_id)
        self.spib.SPIB = 0

        log.info("Enabling dsp capture (PROCEN) of stream %d", self.stream_id)
        hda.PPCTL |= (1 << self.stream_id)
        self.debug()

        log.info("Setting buffer list, length, and stream id")
        self.mem, self.buf_list_addr, self.n_bufs = self.setup_buf(buf_len)
        for i in range(0, self.buf_len*2):
            self.mem[i] = 0xff
        self.regs.CTL = (self.stream_id << 20 + 1) # must be set to something other than 0
        self.regs.BDPU = (self.buf_list_addr >> 32) & 0xffffffff
        self.regs.BDPL = self.buf_list_addr & 0xffffffff
        self.regs.CBL = buf_len
        self.regs.LVI = self.n_bufs - 1
        self.debug()
        log.info("Stream %d initialized", self.stream_id)


    def __del__(self):
        self.reset()

    def start(self):
        log.info("Starting stream %d", self.stream_id)
        self.debug()
        self.regs.CTL |= 2
        self.debug()
        log.info("Started stream %d", self.stream_id)

    def stop(self):
        log.info("Stopping stream %d", self.stream_id)
        self.debug()
        self.reset()
        self.debug()
        log.info("Stopped stread %d", self.stream_id)

    def setup_buf(self, buf_len):
        (mem, phys_addr) = map_phys_mem()

        log.info("Mapped 2M huge page at 0x%x for buf size (%d)"
                 % (phys_addr, buf_len))

        # create two buffers in the page of buf_len and mark them
        # in a buffer descriptor list for the hardware to use
        buf0_len = buf_len
        buf1_len = buf_len
        bdl_off = buf0_len + buf1_len
        mem[bdl_off:bdl_off + 32] = struct.pack("<QQQQ",
                                                phys_addr, buf0_len,
                                                phys_addr + buf0_len, buf1_len)
        log.info("Filled the buffer descriptor list (BDL) for DMA.")
        return (mem, phys_addr + bdl_off, 2)

    def debug(self):
        log.info("HDA %d: PPROC %d, CTL 0x%x, LPIB 0x%x, BDPU 0x%x, BDPL 0x%x, CBL 0x%x, LVI 0x%x",
                 self.stream_id, (hda.PPCTL >> self.stream_id) & 1, self.regs.CTL, self.regs.LPIB, self.regs.BDPU,
                 self.regs.BDPL, self.regs.CBL, self.regs.LVI)
        log.info("    FIFOW %d, FIFOS %d, FMT %x, FIFOL %d, DPIB %d, EFIFOS %d",
                 self.regs.FIFOW & 0x7, self.regs.FIFOS, self.regs.FMT, self.regs.FIFOL, self.dbg0.DPIB, self.dbg0.EFIFOS)
        log.info("    status: FIFORDY %d, DESE %d, FIFOE %d, BCIS %d",
                 (self.regs.STS >> 5) & 1, (self.regs.STS >> 4) & 1, (self.regs.STS >> 3) & 1, (self.regs.STS >> 2) & 1)

    def reset(self):
        # Turn DMA off and reset the stream.  Clearing START first is a
        # noop per the spec, but absolutely required for stability.
        # Apparently the reset doesn't stop the stream, and the next load
        # starts before it's ready and kills the load (and often the DSP).
        # The sleep too is required, on at least one board (a fast
        # chromebook) putting the two writes next each other also hangs
        # the DSP!
        log.info("Resetting stream %d", self.stream_id)
        self.debug()
        sd.CTL &= ~2 # clear START
        time.sleep(0.1)
        # set enter reset bit
        sd.CTL = 1
        while (sd.CTL & 1) == 0: pass
        # clear enter reset bit to exit reset
        sd.CTL = 0
        while (sd.CTL & 1) == 1: pass
        # hardware is ready to be used
        self.debug()
        log.info("Reset stream %d", self.stream_id)


def map_regs():
    p = runx(f"grep -iPl 'PCI_CLASS=40(10|38)0' /sys/bus/pci/devices/*/uevent")
    pcidir = os.path.dirname(p)

    # Platform/quirk detection.  ID lists cribbed from the SOF kernel driver
    global cavs15, cavs18, cavs25
    did = int(open(f"{pcidir}/device").read().rstrip(), 16)
    cavs15 = did in [ 0x5a98, 0x1a98, 0x3198 ]
    cavs18 = did in [ 0x9dc8, 0xa348, 0x02c8, 0x06c8, 0xa3f0 ]
    cavs25 = did in [ 0xa0c8, 0x43c8, 0x4b55, 0x4b58, 0x7ad0, 0x51c8 ]

    # Check sysfs for a loaded driver and remove it
    if os.path.exists(f"{pcidir}/driver"):
        mod = os.path.basename(os.readlink(f"{pcidir}/driver/module"))
        log.warning(f"Existing driver found!  Unloading \"{mod}\" module")
        runx(f"rmmod -f {mod}")

    # Disengage runtime power management so the kernel doesn't put it to sleep
    with open(f"{pcidir}/power/control", "w") as ctrl:
        ctrl.write("on")

    # Make sure PCI memory space access and busmastering are enabled.
    # Also disable interrupts so as not to confuse the kernel.
    with open(f"{pcidir}/config", "wb+") as cfg:
        cfg.seek(4)
        cfg.write(b'\x06\x04')

    # Standard HD Audio Registers
    global hdamem
    (hdamem, _) = bar_map(pcidir, 0)
    hda = Regs(hdamem)
    hda.GCAP    = 0x0000
    hda.GCTL    = 0x0008
    hda.SPBFCTL = 0x0704
    hda.PPCTL   = 0x0804

    # Find the ID of the first output stream
    hda_ostream_id = (hda.GCAP >> 8) & 0x0f # number of input streams
    log.info(f"Selected output stream {hda_ostream_id} (GCAP = 0x{hda.GCAP:x})")
    hda.SD_SPIB = 0x0708 + (8 * hda_ostream_id)
    hda.freeze()


    # Standard HD Audio Stream Descriptor
    sd = Regs(hdamem + 0x0080 + (hda_ostream_id * 0x20))
    sd.CTL  = 0x00
    sd.CBL  = 0x08
    sd.LVI  = 0x0c
    sd.BDPL = 0x18
    sd.BDPU = 0x1c
    sd.freeze()

    # Intel Audio DSP Registers
    global bar4_mmap
    (bar4_mem, bar4_mmap) = bar_map(pcidir, 4)
    dsp = Regs(bar4_mem)
    dsp.ADSPCS         = 0x00004
    dsp.HIPCTDR        = 0x00040 if cavs15 else 0x000c0
    dsp.HIPCTDA        =                        0x000c4 # 1.8+ only
    dsp.HIPCTDD        = 0x00044 if cavs15 else 0x000c8
    dsp.HIPCIDR        = 0x00048 if cavs15 else 0x000d0
    dsp.HIPCIDA        =                        0x000d4 # 1.8+ only
    dsp.HIPCIDD        = 0x0004c if cavs15 else 0x000d8
    dsp.SRAM_FW_STATUS = 0x80000 # Start of first SRAM window
    dsp.freeze()

    return (hda, sd, dsp, hda_ostream_id)

def setup_dma_mem(fw_bytes):
    (mem, phys_addr) = map_phys_mem()
    mem[0:len(fw_bytes)] = fw_bytes

    log.info("Mapped 2M huge page at 0x%x to contain %d bytes of firmware"
          % (phys_addr, len(fw_bytes)))

    # HDA requires at least two buffers be defined, but we don't care about
    # boundaries because it's all a contiguous region. Place a vestigial
    # 128-byte (minimum size and alignment) buffer after the main one, and put
    # the 4-entry BDL list into the final 128 bytes of the page.
    buf0_len = HUGEPAGESZ - 2 * 128
    buf1_len = 128
    bdl_off = buf0_len + buf1_len
    mem[bdl_off:bdl_off + 32] = struct.pack("<QQQQ",
                                            phys_addr, buf0_len,
                                            phys_addr + buf0_len, buf1_len)
    log.info("Filled the buffer descriptor list (BDL) for DMA.")
    return (phys_addr + bdl_off, 2)

global_mmaps = [] # protect mmap mappings from garbage collection!

# Maps 2M of contiguous memory using a single page from hugetlbfs,
# then locates its physical address for use as a DMA buffer.
def map_phys_mem():
    # Make sure hugetlbfs is mounted (not there on chromeos)
    os.system("mount | grep -q hugetlbfs ||"
              + " (mkdir -p /dev/hugepages; "
              + "  mount -t hugetlbfs hugetlbfs /dev/hugepages)")

    # Ensure the kernel has enough budget for one new page
    free = int(runx("awk '/HugePages_Free/ {print $2}' /proc/meminfo"))
    if free == 0:
        tot = 1 + int(runx("awk '/HugePages_Total/ {print $2}' /proc/meminfo"))
        os.system(f"echo {tot} > /proc/sys/vm/nr_hugepages")

    hugef = open(HUGEPAGE_FILE, "w+")
    hugef.truncate(HUGEPAGESZ)
    mem = mmap.mmap(hugef.fileno(), HUGEPAGESZ)
    log.info("type of mem is %s", str(type(mem)))
    global_mmaps.append(mem)
    os.unlink(HUGEPAGE_FILE)

    # Find the local process address of the mapping, then use that to extract
    # the physical address from the kernel's pagemap interface.  The physical
    # page frame number occupies the bottom bits of the entry.
    mem[0] = 0 # Fault the page in so it has an address!
    vaddr = ctypes.addressof(ctypes.c_int.from_buffer(mem))
    vpagenum = vaddr >> 12
    pagemap = open("/proc/self/pagemap", "rb")
    pagemap.seek(vpagenum * 8)
    pent = pagemap.read(8)
    paddr = (struct.unpack("Q", pent)[0] & ((1 << 55) - 1)) * PAGESZ
    pagemap.close()
    return (mem, paddr)

# Maps a PCI BAR and returns the in-process address
def bar_map(pcidir, barnum):
    f = open(pcidir + "/resource" + str(barnum), "r+")
    mm = mmap.mmap(f.fileno(), os.fstat(f.fileno()).st_size)
    global_mmaps.append(mm)
    log.info("Mapped PCI bar %d of length %d bytes."
             % (barnum, os.fstat(f.fileno()).st_size))
    return (ctypes.addressof(ctypes.c_int.from_buffer(mm)), mm)

# Syntactic sugar to make register block definition & use look nice.
# Instantiate from a base address, assign offsets to (uint32) named registers as
# fields, call freeze(), then the field acts as a direct alias for the register!
class Regs:
    def __init__(self, base_addr):
        vars(self)["base_addr"] = base_addr
        vars(self)["ptrs"] = {}
        vars(self)["frozen"] = False
    def freeze(self):
        vars(self)["frozen"] = True
    def __setattr__(self, name, val):
        if not self.frozen and name not in self.ptrs:
            addr = self.base_addr + val
            self.ptrs[name] = ctypes.c_uint32.from_address(addr)
        else:
            self.ptrs[name].value = val
    def __getattr__(self, name):
        return self.ptrs[name].value

def runx(cmd):
    return subprocess.check_output(cmd, shell=True).decode().rstrip()

def load_firmware(fw_file):
    try:
        fw_bytes = open(fw_file, "rb").read()
    except Exception:
        log.error(f"Could not read firmware file: `{fw_file}'")
        sys.exit(1)

    (magic, sz) = struct.unpack("4sI", fw_bytes[0:8])
    if magic == b'XMan':
        log.info(f"Trimming {sz} bytes of extended manifest")
        fw_bytes = fw_bytes[sz:len(fw_bytes)]

    # This actually means "enable access to BAR4 registers"!
    hda.PPCTL |= (1 << 30) # GPROCEN, "global processing enable"

    log.info("Resetting HDA device")
    hda.GCTL = 0
    while hda.GCTL & 1: pass
    hda.GCTL = 1
    while not hda.GCTL & 1: pass

    log.info("Powering down DSP cores")
    dsp.ADSPCS = 0xffff
    while dsp.ADSPCS & 0xff000000: pass

    log.info(f"Configuring HDA stream {hda_ostream_id} to transfer firmware image")
    (buf_list_addr, num_bufs) = setup_dma_mem(fw_bytes)
    sd.CTL = 1
    while (sd.CTL & 1) == 0: pass
    sd.CTL = 0
    while (sd.CTL & 1) == 1: pass
    sd.CTL = (1 << 20) # Set stream ID to anything non-zero
    sd.BDPU = (buf_list_addr >> 32) & 0xffffffff
    sd.BDPL = buf_list_addr & 0xffffffff
    sd.CBL = len(fw_bytes)
    sd.LVI = num_bufs - 1
    hda.PPCTL |= (1 << hda_ostream_id)

    # SPIB ("Software Position In Buffer") is an Intel HDA extension
    # that puts a transfer boundary into the stream beyond which the
    # other side will not read.  The ROM wants to poll on a "buffer
    # full" bit on the other side that only works with this enabled.
    hda.SPBFCTL |= (1 << hda_ostream_id)
    hda.SD_SPIB = len(fw_bytes)

    # Start DSP.  Host needs to provide power to all cores on 1.5
    # (which also starts them) and 1.8 (merely gates power, DSP also
    # has to set PWRCTL).  The bits for cores other than 0 are ignored
    # on 2.5 where the DSP has full control.
    log.info(f"Starting DSP, ADSPCS = 0x{dsp.ADSPCS:x}")
    dsp.ADSPCS = 0xff0000 if not cavs25 else 0x01fefe
    while (dsp.ADSPCS & 0x1000000) == 0: pass

    # Wait for the ROM to boot and signal it's ready.  This short
    # sleep seems to be needed; if we're banging on the memory window
    # during initial boot (before/while the window control registers
    # are configured?) the DSP hardware will hang fairly reliably.
    log.info("Wait for ROM startup")
    time.sleep(0.1)
    while (dsp.SRAM_FW_STATUS >> 24) != 5: pass

    # Send the DSP an IPC message to tell the device how to boot.
    # Note: with cAVS 1.8+ the ROM receives the stream argument as an
    # index within the array of output streams (and we always use the
    # first one by construction).  But with 1.5 it's the HDA index,
    # and depends on the number of input streams on the device.
    stream_idx = hda_ostream_id if cavs15 else 0
    ipcval = (  (1 << 31)            # BUSY bit
                | (0x01 << 24)       # type = PURGE_FW
                | (1 << 14)          # purge_fw = 1
                | (stream_idx << 9)) # dma_id
    log.info(f"Sending IPC command, HIPIDR = 0x{ipcval:x}")
    dsp.HIPCIDR = ipcval

    log.info(f"Starting DMA, FW_STATUS = 0x{dsp.SRAM_FW_STATUS:x}")
    sd.CTL |= 2 # START flag

    log.info(f"Waiting for firmware handoff, FW_STATUS = 0x{dsp.SRAM_FW_STATUS:x}")
    for _ in range(200):
        alive = dsp.SRAM_FW_STATUS & ((1 << 28) - 1) == 5 # "FW_ENTERED"
        if alive: break
        time.sleep(0.01)
    if not alive:
        log.warning(f"Load failed?  FW_STATUS = 0x{dsp.SRAM_FW_STATUS:x}")

    # Turn DMA off and reset the stream.  Clearing START first is a
    # noop per the spec, but absolutely required for stability.
    # Apparently the reset doesn't stop the stream, and the next load
    # starts before it's ready and kills the load (and often the DSP).
    # The sleep too is required, on at least one board (a fast
    # chromebook) putting the two writes next each other also hangs
    # the DSP!
    sd.CTL &= ~2 # clear START
    time.sleep(0.1)
    sd.CTL |= 1
    log.info(f"cAVS firmware load complete")

# This SHOULD be just "mem[start:start+length]", but slicing an mmap
# array seems to be unreliable on one of my machines (python 3.6.9 on
# Ubuntu 18.04).  Read out bytes individually.
def win_read(start, length):
    return b''.join(bar4_mmap[x + WINSTREAM_OFFSET].to_bytes(1, 'little')
                    for x in range(start, start + length))

def win_hdr():
    return struct.unpack("<IIII", win_read(0, 16))

# Python implementation of the same algorithm in sys_winstream_read(),
# see there for details.
def winstream_read(last_seq):
    while True:
        (wlen, start, end, seq) = win_hdr()
        if last_seq == 0:
            last_seq = seq if args.no_history else (seq - ((end - start) % wlen))
        if seq == last_seq or start == end:
            return (seq, "")
        behind = seq - last_seq
        if behind > ((end - start) % wlen):
            return (seq, "")
        copy = (end - behind) % wlen
        suffix = min(behind, wlen - copy)
        result = win_read(16 + copy, suffix)
        if suffix < behind:
            result += win_read(16, behind - suffix)
        (wlen, start1, end, seq1) = win_hdr()
        if start1 == start and seq1 == seq:
            return (seq, result.decode("utf-8"))

async def ipc_delay_done():
    await asyncio.sleep(0.1)
    dsp.HIPCTDA = 1<<31

ipc_timestamp = 0

# Super-simple command language, driven by the test code on the DSP
def ipc_command(data, ext_data):
    send_msg = False
    done = True
    if data == 0: # noop, with synchronous DONE
        pass
    elif data == 1: # async command: signal DONE after a delay (on 1.8+)
        if not cavs15:
            done = False
            asyncio.ensure_future(ipc_delay_done())
    elif data == 2: # echo back ext_data as a message command
        send_msg = True
    elif data == 3: # set ADSPCS
        dsp.ADSPCS = ext_data
    elif data == 4: # echo back microseconds since last timestamp command
        global ipc_timestamp
        t = round(time.time() * 1e6)
        ext_data = t - ipc_timestamp
        ipc_timestamp = t
        send_msg = True
    elif data == 5: # HDA INIT
        global host_in
        stream_id = ext_data & 0xff
        buf_len = (ext_data >> 8) & 0xffff
        log.info("HDA init stream %d with buf_len %d", stream_id, buf_len)
        host_in = HDAStream(stream_id, buf_len)
        log.info("HDA init stream %d done", ext_data & 0xff)
    elif data == 6: # HDA START
        log.info("HDA starting stream %d", ext_data & 0xff)
        host_in.start()
        log.info("HDA started stream %d", ext_data & 0xff)
    elif data == 7: # HDA VALIDATE
        log.info("HDA validating stream %d for ramp, mem type %s", ext_data & 0xff, type(host_in.mem))
        host_in.debug()
        is_ramp_data = True
        for i in range(0, host_in.buf_len*2):
            if i != host_in.mem[i]:
                is_ramp_data = False
            log.info("hda stream buffer %d: %d", i, host_in.mem[i])
        log.info("Is ramp data?")
        # TODO send message back
    elif data == 8: # HDA HOST IN RESET
        log.warning("HDA host in reset")
        del host_in
    else:
        log.warning(f"cavstool: Unrecognized IPC command 0x{data:x} ext 0x{ext_data:x}")

    dsp.HIPCTDR = 1<<31 # Ack local interrupt, also signals DONE on v1.5
    if cavs18:
        time.sleep(0.01) # Needed on 1.8, or the command below won't send!

    if done and not cavs15:
        log.info("Sending done")
        dsp.HIPCTDA = 1<<31 # Signal done
        log.info("Sent done")
    if send_msg:
        log.info("Sending Message")
        dsp.HIPCIDD = ext_data
        dsp.HIPCIDR = (1<<31) | ext_data

async def main():
    global hda, sd, dsp, hda_ostream_id
    try:
        (hda, sd, dsp, hda_ostream_id) = map_regs()
    except Exception:
        log.error("Could not map device in sysfs; run as root.")
        sys.exit(1)

    log.info(f"Detected cAVS {'1.5' if cavs15 else '1.8+'} hardware")

    if not args.log_only:
        load_firmware(args.fw_file)
        time.sleep(0.1)
        if not args.quiet:
            sys.stdout.write("--\n")

    last_seq = 0
    while True:
        await asyncio.sleep(0.03)
        (last_seq, output) = winstream_read(last_seq)
        if output:
            sys.stdout.write(output)
            sys.stdout.flush()
        if dsp.HIPCTDR & 0x80000000:
            log.info("ipc_command exit")
            ipc_command(dsp.HIPCTDR & ~0x80000000, dsp.HIPCTDD)
            log.info("ipc_command exit")
        if dsp.HIPCIDA & 0x80000000:
            log.info("ACKing done")
            dsp.HIPCIDA = 1<<31 # must ACK any DONE interrupts that arrive!
            log.info("ACKed done")


ap = argparse.ArgumentParser(description="DSP loader/logger tool")
ap.add_argument("-q", "--quiet", action="store_true",
                help="No loader output, just DSP logging")
ap.add_argument("-l", "--log-only", action="store_true",
                help="Don't load firmware, just show log output")
ap.add_argument("-n", "--no-history", action="store_true",
                help="No current log buffer at start, just new output")
ap.add_argument("fw_file", nargs="?", help="Firmware file")
args = ap.parse_args()

if args.quiet:
    log.setLevel(logging.WARN)

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(main())
