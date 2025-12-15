#!/usr/bin/env python3
"""
COMPE 571 – Programming Assignment 4
Virtual Memory Simulator

This file implements:
- Trace parsing (pid, virtual address, R/W)
- Virtual to physical address translation
- Per-process page tables
- Physical memory with 32 frames
- Page fault handling and disk access accounting
- RAND page replacement policy
- Placeholder locations for FIFO, LRU, and PER

Replacement algorithms other than RAND are unimplemented.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Iterable
import sys
import random


# ============================================================
# Assignment constants
# ============================================================

PAGE_SIZE = 512                  # bytes per page (2^9)
OFFSET_BITS = 9
VPN_BITS = 7                     # 16-bit virtual address
VADDR_BITS = 16

VPN_COUNT = 1 << VPN_BITS        # 128 virtual pages per process
PHYS_FRAMES = 32                 # 32 physical frames (16 KB total)


# ============================================================
# Data structures
# ============================================================

@dataclass
class PageTableEntry:
    valid: bool = False
    frame: Optional[int] = None
    dirty: int = 0
    ref: int = 0
    last_used: int = -1           # used by LRU


@dataclass
class FrameInfo:
    used: bool = False
    pid: Optional[int] = None
    vpn: Optional[int] = None
    load_time: int = -1           # used by FIFO


@dataclass
class Stats:
    page_faults: int = 0
    disk_accesses: int = 0
    dirty_writes: int = 0


# ============================================================
# Replacement policy interface
# ============================================================

class ReplacementPolicy:
    """
    All page replacement algorithms must implement choose_victim().
    """

    name: str = "BASE"

    def choose_victim(
        self,
        frames: List[FrameInfo],
        page_tables: Dict[int, List[PageTableEntry]],
        time: int
    ) -> int:
        raise NotImplementedError

    def on_page_loaded(self, frame_id: int, pid: int, vpn: int, time: int) -> None:
        return

    def on_page_access(self, pid: int, vpn: int, is_write: bool, time: int) -> None:
        return

    def on_ref_reset_period(self, page_tables: Dict[int, List[PageTableEntry]]) -> None:
        return


# ============================================================
# RAND replacement policy (implemented)
# ============================================================

class RandPolicy(ReplacementPolicy):
    """
    RAND: randomly select a victim frame from all currently used frames.
    """

    name = "RAND"

    def __init__(self, seed: Optional[int] = None):
        if seed is not None:
            random.seed(seed)

    def choose_victim(self, frames, page_tables, time) -> int:
        candidates = [i for i, fr in enumerate(frames) if fr.used]
        if not candidates:
            raise RuntimeError("RAND called with no used frames")
        return random.choice(candidates)


# ============================================================
# FIFO placeholder
# ============================================================

class FifoPolicy(ReplacementPolicy):
    """
    FIFO: replace the page that has been in memory the longest.
    """

    name = "FIFO"

    def __init__(self):
        pass

    def on_page_loaded(self, frame_id: int, pid: int, vpn: int, time: int) -> None:
        pass

    def choose_victim(self, frames, page_tables, time) -> int:
        # Consider only frames that are currently used/occupied
        candidates = [i for i, fr in enumerate(frames) if fr.used]
        if not candidates:
            raise RuntimeError("FIFO called with no used frames")

        # Pick the frame whose load_time is smallest (oldest in memory).
        # Tie-breaker: smaller frame index (min() handles that automatically
        # if we use (load_time, frame_id) as the key).
        victim_frame_id = min(
            candidates,
            key=lambda i: (frames[i].load_time, i)
        )
        return victim_frame_id
    


# ============================================================
# LRU placeholder
# ============================================================

class LruPolicy(ReplacementPolicy):
    """
    LRU: replace the least recently used page.
    Tie-breaking rules are defined in the assignment.
    """

    name = "LRU"

    def __init__(self):
        pass

    def choose_victim(self, frames, page_tables, time) -> int:
        candidates = [i for i, fr in enumerate(frames) if fr.used]
        if not candidates:
            raise RuntimeError("LRU called with no used frames")

        def last_used_for_frame(frame_id: int) -> tuple[int, int]:
            fr = frames[frame_id]
            pid = fr.pid
            vpn = fr.vpn

            # In a well-formed simulator, these should never be None
            # for a used frame, but we'll be defensive.
            if pid is None or vpn is None:
                # Treat as very recently used so it's unlikely to be chosen
                return (float("inf"), frame_id)

            pte = page_tables[pid][vpn]
            # Key is (last_used, frame_id): lower last_used first, then lower frame_id
            return (pte.last_used, frame_id)

        victim_frame_id = min(candidates, key=last_used_for_frame)
        return victim_frame_id


# ============================================================
# PER placeholder
# ============================================================

class PerPolicy(ReplacementPolicy):
    """
    PER: periodic reference bit reset with category-based replacement.
    """

    name = "PER"

    def __init__(self):
        pass

    def choose_victim(self, frames, page_tables, time) -> int:
        raise NotImplementedError


# ============================================================
# Virtual memory simulator
# ============================================================

class VMSimulator:
    def __init__(self, policy: ReplacementPolicy, per_reset_period: int = 200):
        self.policy = policy
        self.per_reset_period = per_reset_period

        self.time = 0
        self.refs_processed = 0
        self.stats = Stats()

        self.page_tables: Dict[int, List[PageTableEntry]] = {}
        self.frames: List[FrameInfo] = [FrameInfo() for _ in range(PHYS_FRAMES)]

    # --------------------------------------------------------
    # Trace parsing
    # --------------------------------------------------------

    def trace_iter(self, path: str) -> Iterable[Tuple[int, int, str]]:
        with open(path, "r", encoding="utf-8") as f:
            for line_num, line in enumerate(f, start=1):
                line = line.strip()
                if not line:
                    continue

                pid_str, addr_str, rw = line.split()
                pid = int(pid_str)
                vaddr = int(addr_str)
                rw = rw.upper()

                yield pid, vaddr, rw

    # --------------------------------------------------------
    # Address translation helpers
    # --------------------------------------------------------

    @staticmethod
    def split_vaddr(vaddr: int) -> Tuple[int, int]:
        vpn = vaddr >> OFFSET_BITS
        offset = vaddr & (PAGE_SIZE - 1)
        return vpn, offset

    def ensure_process(self, pid: int) -> None:
        if pid not in self.page_tables:
            self.page_tables[pid] = [PageTableEntry() for _ in range(VPN_COUNT)]

    # --------------------------------------------------------
    # Frame management
    # --------------------------------------------------------

    def find_free_frame(self) -> Optional[int]:
        for i, fr in enumerate(self.frames):
            if not fr.used:
                return i
        return None

    def evict_frame(self, frame_id: int) -> None:
        fr = self.frames[frame_id]
        victim_pid = fr.pid
        victim_vpn = fr.vpn

        pte = self.page_tables[victim_pid][victim_vpn]

        if pte.dirty == 1:
            self.stats.dirty_writes += 1
            self.stats.disk_accesses += 1

        pte.valid = False
        pte.frame = None
        pte.dirty = 0
        pte.ref = 0
        pte.last_used = -1

        fr.used = False
        fr.pid = None
        fr.vpn = None
        fr.load_time = -1

    def load_page(self, pid: int, vpn: int, frame_id: int) -> None:
        self.stats.disk_accesses += 1

        pte = self.page_tables[pid][vpn]
        pte.valid = True
        pte.frame = frame_id
        pte.ref = 1
        pte.last_used = self.time

        fr = self.frames[frame_id]
        fr.used = True
        fr.pid = pid
        fr.vpn = vpn
        fr.load_time = self.time

        self.policy.on_page_loaded(frame_id, pid, vpn, self.time)

    # --------------------------------------------------------
    # Memory access path
    # --------------------------------------------------------

    def access(self, pid: int, vaddr: int, rw: str) -> None:
        self.ensure_process(pid)

        if self.refs_processed > 0 and self.refs_processed % self.per_reset_period == 0:
            for table in self.page_tables.values():
                for pte in table:
                    if pte.valid:
                        pte.ref = 0
            self.policy.on_ref_reset_period(self.page_tables)

        vpn, _ = self.split_vaddr(vaddr)
        is_write = (rw == "W")

        pte = self.page_tables[pid][vpn]

        if not pte.valid:
            self.stats.page_faults += 1

            frame_id = self.find_free_frame()
            if frame_id is None:
                frame_id = self.policy.choose_victim(self.frames, self.page_tables, self.time)
                self.evict_frame(frame_id)

            self.load_page(pid, vpn, frame_id)

        pte.ref = 1
        pte.last_used = self.time
        if is_write:
            pte.dirty = 1

        self.policy.on_page_access(pid, vpn, is_write, self.time)

        self.refs_processed += 1
        self.time += 1

    def run_trace(self, path: str) -> Stats:
        for pid, vaddr, rw in self.trace_iter(path):
            self.access(pid, vaddr, rw)
        return self.stats


# ============================================================
# Program entry point
# ============================================================

def main(argv: List[str]) -> int:
    if len(argv) != 2:
        print("Usage: python vm_sim.py <trace_file>")
        return 1

    trace_path = argv[1]

    policy = RandPolicy(seed=0)
    #policy = FifoPolicy()
    #policy = LruPolicy()
    # policy = PerPolicy()

    sim = VMSimulator(policy=policy)
    stats = sim.run_trace(trace_path)

    print(f"Policy: {policy.name}")
    print(f"Page Faults:     {stats.page_faults}")
    print(f"Disk Accesses:   {stats.disk_accesses}")
    print(f"Dirty Writes:    {stats.dirty_writes}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
