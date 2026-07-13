# Exhaust impulse responses

These recorded exhaust impulse responses are reused from **Engine Sim 2D
(es2d)** by AngeTheGreat (Ange Yaghi), released under the MIT license. They are
used here as the default convolution kernels that give the exhaust its body and
resonance; the per-engine excitation comes from EngineLab's own simulated
cylinder/runner pressure.

- `exhaust_default.wav` — es2d `smooth/smooth_39.wav` (generic fallback).

Per exhaust preset (selected in the UI); each is a distinct es2d recording so the
presets voice the muffler/system differently:

- `exhaust_street.wav`   — Street  (es2d `smooth/smooth_39.wav`).
- `exhaust_open.wav`     — Open    (es2d `archive/test_engine_16_eq_adjusted_16.wav`).
- `exhaust_turbo.wav`    — Turbo   (es2d `smooth/smooth_05.wav`).
- `exhaust_longtube.wav` — Long tube (es2d `smooth/smooth_20.wav`).
- `exhaust_moto.wav`     — Moto    (es2d `smooth/smooth_12.wav`).

Original copyright © 2022 AngeTheGreat, MIT license. See the es2d repository for
the full license text.
