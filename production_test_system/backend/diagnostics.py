"""Always-on diagnostics so a hang/livelock is debuggable *after the fact*.

Motivation: a production run once wedged with three bus-worker threads spinning
at 100% CPU.  We could not get Python stacks (macOS SIP blocks py-spy/lldb from
attaching to the Xcode-Python the venv was built on) and the in-memory log had
drained, so the root cause could not be pinned without restarting (which
destroys the evidence).  This module makes sure that never happens again:

* **On-demand thread dump** — ``GET /api/debug/stacks`` (and ``SIGUSR1``) print
  every thread's Python stack.  HTTP stayed responsive during the wedge, so this
  is the primary tool; ``SIGUSR1`` is a no-attach, SIP-immune backup.
* **Durable log** — ``data/diagnostics.log`` captures worker lifecycle/errors
  (teed from the app log) plus periodic vitals, so history survives a restart.
* **Watchdog** — a daemon thread logs vitals every minute and, on a sustained
  high-CPU signature (the livelock fingerprint), auto-dumps all stacks so the
  evidence is captured even if nobody is watching.

Everything here is read-only/observational: it never kills threads, cancels
work, or touches the bus.  See ``DEBUGGING.md`` for the operator runbook.
"""

from __future__ import annotations

import os
import sys
import threading
import time
import traceback
from typing import Callable, Optional

from . import config

# Persistent files live next to the database so they survive restarts.
DIAG_LOG_PATH = os.path.join(config.DATA_DIR, "diagnostics.log")
FAULT_LOG_PATH = os.path.join(config.DATA_DIR, "faulthandler.log")
_MAX_LOG_BYTES = 16 * 1024 * 1024  # rotate once past ~16 MB (keeps one .old)

_log_lock = threading.Lock()
_fault_fp = None  # kept alive for the lifetime of the process


def _ts() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S")


def log_line(msg: str) -> None:
    """Append a timestamped line to the persistent diagnostics log (best effort)."""
    try:
        with _log_lock:
            if (os.path.exists(DIAG_LOG_PATH)
                    and os.path.getsize(DIAG_LOG_PATH) > _MAX_LOG_BYTES):
                try:
                    os.replace(DIAG_LOG_PATH, DIAG_LOG_PATH + ".old")
                except OSError:
                    pass
            with open(DIAG_LOG_PATH, "a", buffering=1) as fp:
                fp.write("%s  %s\n" % (_ts(), msg))
    except Exception:
        pass  # diagnostics must never break the app


def dump_all_stacks() -> str:
    """Return every live thread's Python stack as text (no attach required)."""
    names = {t.ident: t.name for t in threading.enumerate()}
    frames = sys._current_frames()
    parts = ["===== thread stack dump @ %s (%d threads) =====" % (_ts(), len(frames))]
    for tid, frame in sorted(frames.items()):
        parts.append("\n--- Thread %s (id=%s) ---" % (names.get(tid, "?"), tid))
        parts.append("".join(traceback.format_stack(frame)).rstrip())
    return "\n".join(parts)


def install_faulthandler() -> None:
    """Enable faulthandler and dump all Python stacks to FAULT_LOG_PATH on SIGUSR1.

    Works even while a thread spins in a C extension (e.g. sqlite3) and needs no
    debugger attach, so it is immune to the SIP restriction that blocked py-spy.
    """
    global _fault_fp
    try:
        import faulthandler
        import signal
        _fault_fp = open(FAULT_LOG_PATH, "a", buffering=1)
        faulthandler.enable()  # dump on a hard crash (SIGSEGV/SIGABRT/...)
        # signal registration must happen on the main thread; guard it.
        faulthandler.register(signal.SIGUSR1, file=_fault_fp,
                              all_threads=True, chain=False)
        log_line("faulthandler armed: kill -USR1 %d -> stacks to %s"
                 % (os.getpid(), FAULT_LOG_PATH))
    except Exception as exc:
        log_line("faulthandler NOT armed: %r" % (exc,))


def start_watchdog(vitals_fn: Callable[[], str],
                   interval_s: float = 60.0,
                   cpu_alert_cores: float = 1.5,
                   alert_after: int = 3) -> None:
    """Log vitals every ``interval_s`` and auto-dump stacks on sustained high CPU.

    ``vitals_fn`` returns a short human string describing per-bus state.  CPU use
    is measured process-wide from ``os.times()`` (stdlib, no psutil); the
    livelock fingerprint is several cores pegged for several minutes, which
    normal operation (workers sleep between bus transactions) does not produce.
    A false trigger only costs one extra stack dump in the log -- it never
    interferes with the run.
    """
    def run() -> None:
        prev = os.times()
        prev_wall = time.monotonic()
        high_streak = 0
        dumped_this_episode = False
        log_line("watchdog started (pid=%d interval=%.0fs cpu_alert=%.1f cores)"
                 % (os.getpid(), interval_s, cpu_alert_cores))
        while True:
            time.sleep(interval_s)
            try:
                now = os.times()
                wall = time.monotonic()
                dt = max(1e-9, wall - prev_wall)
                cores = ((now.user + now.system) - (prev.user + prev.system)) / dt
                prev, prev_wall = now, wall
                try:
                    vit = vitals_fn()
                except Exception as exc:
                    vit = "vitals_fn error: %r" % (exc,)
                log_line("VITALS cpu_cores=%.2f threads=%d  %s"
                         % (cores, threading.active_count(), vit))
                if cores >= cpu_alert_cores:
                    high_streak += 1
                    if high_streak >= alert_after and not dumped_this_episode:
                        log_line("WATCHDOG ALERT: %.2f cores sustained for ~%.0fs "
                                 "-- auto-dumping stacks (likely livelock)"
                                 % (cores, alert_after * interval_s))
                        log_line(dump_all_stacks())
                        dumped_this_episode = True
                else:
                    high_streak = 0
                    dumped_this_episode = False
            except Exception:
                pass  # never let the watchdog die
    threading.Thread(target=run, name="diag-watchdog", daemon=True).start()
