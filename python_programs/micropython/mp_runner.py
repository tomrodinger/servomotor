"""
On-device test runner for ESP32-S3.

Runs each test file in /tests/ with sys.argv[:] = [test, '-p', 'MP_UART', '-a', 'X'].
Robust to UART leaks: forces servomotor.close_serial_port() between tests.

If invoked with the env variable `MP_RUN_RANGE` (e.g. "0-12"), runs only that
slice — used for chunked sweeps so we can machine.reset() between chunks and
avoid MemoryError fragmentation.

Persists per-test results to /results.txt so chunked runs accumulate.

Prints one '@@@ RESULT <name> <status> <dur>' line per test, plus a summary.
"""

import sys
import time
import gc
import os


# ---- Install compat shims so existing CPython tests run unmodified ----

# `random.choices` is missing on MicroPython. Register a sys.modules wrapper
# BEFORE any test imports random so `import random` resolves to our wrapper.
try:
    import mp_random_shim
    sys.modules["random"] = mp_random_shim
except Exception as _e:
    print("WARN: random shim install failed:", _e)

# `collections.defaultdict` is missing on MicroPython. Install a wrapper.
try:
    import mp_collections_shim
    sys.modules["collections"] = mp_collections_shim
except Exception as _e:
    print("WARN: collections shim install failed:", _e)

# `string` is missing on MicroPython — provided as /string.py.
# `os.path` cannot be monkey-patched (os is frozen); tests that need it
# have been edited to use portable string manipulation instead.

# `exit` (a CPython site-builtin) is missing on MicroPython. Inject it.
try:
    import builtins as _builtins
    if not hasattr(_builtins, "exit"):
        _builtins.exit = sys.exit
except Exception as _e:
    print("WARN: builtins.exit shim failed:", _e)


# ---- Runner ----

def _list_dir(path):
    try:
        return sorted(os.listdir(path))
    except OSError:
        return []


ALL_TESTS = [f for f in _list_dir("/tests") if f.startswith("test_") and f.endswith(".py")]

# Tests known to be MicroPython-incompatible by design.
SKIP = set()


def force_close_uart():
    try:
        import servomotor
        servomotor.close_serial_port()
    except Exception:
        pass
    try:
        from servomotor import communication as c
        if c.ser is not None:
            try:
                c.ser.close()
            except Exception:
                pass
            c.ser = None
    except Exception:
        pass


def run_one(test_file):
    print()
    print("@@@ RUN", test_file)
    sys.argv[:] = [test_file, "-p", "MP_UART", "-a", "X"]
    t0 = time.ticks_ms()
    status = "ERROR"
    err_msg = ""
    try:
        with open("/tests/" + test_file) as f:
            src = f.read()
        code = compile(src, test_file, "exec")
        gns = {"__name__": "__main__", "__file__": test_file}
        try:
            exec(code, gns)
            status = "PASSED"
        except SystemExit as e:
            code_val = 0 if e.args == () else e.args[0]
            status = "PASSED" if code_val in (None, 0) else "FAILED(exit=%s)" % code_val
    except Exception as e:
        status = "ERROR"
        err_msg = repr(e)
        sys.print_exception(e)
    finally:
        force_close_uart()
    dur = time.ticks_diff(time.ticks_ms(), t0) / 1000.0
    print("@@@ RESULT", test_file, status, "%.2fs" % dur, err_msg)
    gc.collect()
    return status, dur


def _append_result(name, status, dur):
    try:
        with open("/results.txt", "a") as f:
            f.write("%s|%s|%.2f\n" % (name, status, dur))
    except Exception as e:
        print("WARN: cannot persist result:", e)


def _read_done():
    """Return set of test names already recorded in /results.txt."""
    done = set()
    try:
        with open("/results.txt") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split("|", 1)
                done.add(parts[0])
    except OSError:
        pass
    return done


def main():
    chunk_size_env = None
    try:
        with open("/chunk_size.txt") as f:
            chunk_size_env = int(f.read().strip())
    except Exception:
        chunk_size_env = None

    chunk_size = chunk_size_env if chunk_size_env else len(ALL_TESTS)
    done = _read_done()
    pending = [t for t in ALL_TESTS if t not in SKIP and t not in done]

    if not pending:
        print()
        print("=" * 60)
        print("All tests already done. Reporting from /results.txt:")
        results = []
        with open("/results.txt") as f:
            for line in f:
                parts = line.strip().split("|", 2)
                if len(parts) == 3:
                    results.append((parts[0], parts[1], float(parts[2])))
        _print_summary(results)
        return

    chunk = pending[:chunk_size]
    print("Pending: %d  Running this chunk: %d  Already done: %d" % (len(pending), len(chunk), len(done)))
    overall_start = time.ticks_ms()
    for t in chunk:
        status, dur = run_one(t)
        _append_result(t, status, dur)
        time.sleep_ms(200)
    total = time.ticks_diff(time.ticks_ms(), overall_start) / 1000.0
    print()
    print("Chunk done in %.1fs. %d remaining after this chunk." % (total, len(pending) - len(chunk)))


def _print_summary(results):
    print()
    print("=" * 60)
    print("MicroPython test summary  (%d tests)" % len(results))
    print("=" * 60)
    passed = failed = errored = 0
    total_dur = 0.0
    for name, status, dur in results:
        print("  %-50s %8.2fs  %s" % (name, dur, status))
        total_dur += dur
        if status == "PASSED":
            passed += 1
        elif status.startswith("FAILED"):
            failed += 1
        else:
            errored += 1
    print("-" * 60)
    print("PASSED=%d  FAILED=%d  ERRORED=%d  TOTAL=%d  CUM=%.1fs" %
          (passed, failed, errored, len(results), total_dur))


main()
