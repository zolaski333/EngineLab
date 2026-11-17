# Exhaust impulse responses

These recorded exhaust impulse responses are reused from **Engine Sim 2D
(es2d)** by AngeTheGreat (Ange Yaghi), released under the MIT license. No
engine loads one by default: the physical renderer carries the pipes'
propagation and radiation itself, and an exhaust path with no
`impulse_response` is heard in free field. A file here is convolved only when
an exhaust path names it explicitly (`exhaust_paths[].impulse_response`, see
`docs/custom-exhaust.md`), as a measured downstream response, never as a
stand-in for the gas physics.

- `exhaust_default.wav` — es2d `smooth/smooth_39.wav`.

The other files are distinct es2d recordings, kept for a path that names one:

- `exhaust_street.wav`   — Street  (es2d `smooth/smooth_39.wav`).
- `exhaust_open.wav`     — Open    (es2d `archive/test_engine_16_eq_adjusted_16.wav`).
- `exhaust_turbo.wav`    — Turbo   (es2d `smooth/smooth_05.wav`).
- `exhaust_longtube.wav` — Long tube (es2d `smooth/smooth_20.wav`).
- `exhaust_moto.wav`     — Moto    (es2d `smooth/smooth_12.wav`).

Original copyright © 2022 Ange Yaghi (AngeTheGreat), MIT license. The full
license text is reproduced beside these files in
[LICENSE-es2d.txt](LICENSE-es2d.txt).
