#!/usr/bin/env python3
"""Measure the pops of a backfire recording, real or rendered, the same way.

  python scripts/pop_metrics.py <file.wav|.ogg|.flac> [...] [--start S] [--end S]
                                [--threshold-db 18] [--floor-db -30] [--list]
                                [--json out.json]

A pop is a 1 ms RMS window standing `--threshold-db` above the median 1 ms RMS
of the surrounding +/-1 s, at a local maximum, at least 20 ms after the
previous pop, and no more than `--floor-db` below the loudest 1 ms of the
measured window (fade-in and fade-out tails are not pops). An engine's own
firing pulses stand 12-16 dB above that median, so the default threshold keeps
them out. `--start`/`--end` only choose which pops are kept: the medians are
taken over the whole file, so the window's edges do not bias them.

Per pop:

  over_db     peak 1 ms RMS over the local median: the contrast with the engine
  rise_ms     10 -> 90 % of the 0.1 ms-smoothed |x| before its peak
  decay_ms    the peak -> 10 % of it
  band_db     per band (20-120, 120-500, 500-2k, 2k-8k, 8k-16k Hz), the band's
              RMS over the pop (-1 .. +10 ms) against the band's median 10 ms
              RMS over the surrounding +/-1 s, in dB

Energy shares by band are NOT reported: in every real recording measured, a
pop's energy, like the engine's, is >95 % below 2 kHz, so a share says little.
What makes a crack audible is how far a band rises above its own background.

Over the file: medians and the p10/p50/p90 intervals between pops. Absolute
levels are not comparable between a recording and a render (distance,
reverberation and gain are unknown on the real side); contrasts are.

Reads WAV through scipy; OGG/FLAC need the `soundfile` package.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage, signal
from scipy.io import wavfile

BANDS = [(20.0, 120.0), (120.0, 500.0), (500.0, 2000.0), (2000.0, 8000.0),
         (8000.0, 16000.0)]
BAND_NAMES = ["20-120", "120-500", "500-2k", "2k-8k", "8k-16k"]


def load(path: Path) -> tuple[np.ndarray, int]:
    if path.suffix.lower() == ".wav":
        rate, data = wavfile.read(path)
        scale = float(np.iinfo(data.dtype).max) \
            if np.issubdtype(data.dtype, np.integer) else 1.0
        data = data.astype(np.float64) / scale
    else:
        import soundfile  # optional: only for compressed references

        data, rate = soundfile.read(path, dtype="float64", always_2d=True)
    if data.ndim > 1:
        data = data.mean(axis=1)
    return data, int(rate)


def moving_rms(x: np.ndarray, width: int, hop: int) -> np.ndarray:
    if len(x) < width:
        return np.zeros(0)
    power = np.convolve(x * x, np.ones(width) / width, mode="valid")[::hop]
    return np.sqrt(np.maximum(power, 0.0))


def running_median(values: np.ndarray, half: int) -> np.ndarray:
    # Edge samples repeat rather than pad with zeros: a zero-padded median sinks
    # over the last second, and every firing pulse there reads as a pop.
    return ndimage.median_filter(values, size=2 * half + 1, mode="nearest")


def analyse(path: Path, start: float, end: float | None,
            threshold_db: float, floor_db: float) -> dict:
    x, rate = load(path)
    # The whole file is the context; only pops inside [start, end) are kept, so
    # a window's edge never shortens the background median.
    first = int(start * rate)
    last = len(x) if end is None else min(len(x), int(end * rate))
    x = x - np.mean(x)
    x = signal.sosfilt(signal.butter(2, 20.0, "highpass", fs=rate, output="sos"), x)

    ms = max(1, rate // 1000)
    rms = moving_rms(x, ms, ms)
    if len(rms) == 0:
        return {"file": str(path), "pop_count": 0, "pops": []}
    background = np.maximum(running_median(rms, 1000), 1e-12)
    over = 20.0 * np.log10(np.maximum(rms, 1e-12) / background)
    loud = 20.0 * np.log10(np.maximum(rms, 1e-12) / max(np.max(rms), 1e-12))
    peaks, _ = signal.find_peaks(over, height=threshold_db, distance=20)
    window = loud[first // ms:last // ms]
    loudest = float(np.max(window)) if len(window) else 0.0
    peaks = [p for p in peaks
             if first <= p * ms < last and loud[p] - loudest >= floor_db]

    nyquist = 0.5 * rate
    band_signals, band_backgrounds = [], []
    for low, high in BANDS:
        if low >= nyquist * 0.98:
            band_signals.append(None)
            band_backgrounds.append(None)
            continue
        sos = signal.butter(4, [low, min(high, nyquist * 0.95)], "bandpass",
                            fs=rate, output="sos")
        band = signal.sosfiltfilt(sos, x)
        band_rms = moving_rms(band, 10 * ms, 10 * ms)
        band_signals.append(band)
        band_backgrounds.append(np.maximum(running_median(band_rms, 100), 1e-15))

    smooth = max(1, rate // 10000)
    envelope = np.convolve(np.abs(x), np.ones(smooth) / smooth, mode="same")
    pops = []
    for index in peaks:
        lo = index * ms
        hi = min(len(x), lo + ms)
        peak_at = lo + int(np.argmax(envelope[lo:hi]))
        peak = envelope[peak_at]
        floor = max(peak_at - 10 * ms, 0)
        back = envelope[floor:peak_at + 1][::-1]
        below90 = int(np.argmax(back < 0.9 * peak))
        below10 = int(np.argmax(back < 0.1 * peak))
        rise_ms = (below10 - below90) / rate * 1000.0 if below10 > below90 else float("nan")
        ahead = envelope[peak_at:min(len(x), peak_at + 200 * ms)]
        fell = int(np.argmax(ahead < 0.1 * peak))
        decay_ms = fell / rate * 1000.0 if fell > 0 else float("nan")

        band_db = []
        for band, bg in zip(band_signals, band_backgrounds):
            if band is None:
                band_db.append(float("nan"))
                continue
            segment = band[max(0, peak_at - ms):peak_at + 10 * ms]
            level = math.sqrt(float(np.mean(segment * segment))) if len(segment) else 0.0
            ref = bg[min(len(bg) - 1, peak_at // (10 * ms))]
            band_db.append(20.0 * math.log10(max(level, 1e-15) / ref))
        pops.append({
            "time_s": round(peak_at / rate, 4),
            "over_db": round(float(over[index]), 2),
            "rise_ms": round(float(rise_ms), 3),
            "decay_ms": round(float(decay_ms), 2),
            "band_db": [round(v, 1) for v in band_db],
        })

    def median(values: list[float]) -> float:
        values = [v for v in values if not math.isnan(v)]
        return round(float(np.median(values)), 2) if values else float("nan")

    times = [p["time_s"] for p in pops]
    intervals = np.diff(times) * 1000.0 if len(times) > 1 else np.zeros(0)
    return {
        "file": str(path),
        "sample_rate_hz": rate,
        "seconds": round((last - first) / rate, 3),
        "pop_count": len(pops),
        "median_over_db": median([p["over_db"] for p in pops]),
        "median_rise_ms": median([p["rise_ms"] for p in pops]),
        "median_decay_ms": median([p["decay_ms"] for p in pops]),
        "median_band_db": [median([p["band_db"][i] for p in pops])
                           for i in range(len(BANDS))] if pops else [],
        "interval_ms_p10_p50_p90": [round(float(np.percentile(intervals, q)), 1)
                                    for q in (10, 50, 90)] if len(intervals) else [],
        "pops": pops,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("files", nargs="+", type=Path)
    parser.add_argument("--start", type=float, default=0.0)
    parser.add_argument("--end", type=float, default=None)
    parser.add_argument("--threshold-db", type=float, default=18.0)
    parser.add_argument("--floor-db", type=float, default=-30.0)
    parser.add_argument("--json", type=Path, default=None)
    parser.add_argument("--list", action="store_true", help="print every pop")
    args = parser.parse_args()

    results = []
    for path in args.files:
        if not path.is_file():
            print(f"error: {path} not found", file=sys.stderr)
            return 1
        result = analyse(path, args.start, args.end, args.threshold_db, args.floor_db)
        results.append(result)
        print(f"{path.name}: {result['pop_count']} pops in {result.get('seconds', 0)} s"
              f" @ {result.get('sample_rate_hz', 0)} Hz")
        if result["pop_count"]:
            print(f"  over {result['median_over_db']} dB  rise {result['median_rise_ms']} ms"
                  f"  decay {result['median_decay_ms']} ms")
            print("  band rise dB " + "  ".join(
                f"{name} {value:+.1f}" for name, value in
                zip(BAND_NAMES, result["median_band_db"])))
            print("  intervals p10/p50/p90 ms:", result["interval_ms_p10_p50_p90"])
        if args.list:
            for pop in result["pops"]:
                print("   ", pop)
    if args.json:
        args.json.write_text(json.dumps(results, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
