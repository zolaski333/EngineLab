# IAME X30 125 cm³ (TaG, reed-valve kart engine): reference data

Nothing here comes from the simulator. Every number is read from one of the
sources below and says how it was read. What no source gives is listed at the
end as missing, not guessed.

## Sources

| Key | Document | What it gives |
| --- | --- | --- |
| KA | Karting Australia, "IAME X30 125cc Engine, updated 1 January 2024" (the CIK homologation form 78H/RH, re-homologated 1 August 2018), 77 pages. <https://kartingaustralia.mymedia.delivery/wp-content/uploads/2025/01/IAME-X30-125cc-Engine-Updated-1-January-2024-1.pdf> | Geometry, port timings and chords, reed block, both homologated exhausts |
| BIL | Bilsport (Norway), "IAME X30 2021" form 254/V. <https://bilsport.no/wp-content/uploads/2021/03/IAME-X30-2021-_2.pdf> | Carburettor, ignition, limiter |
| OB | Ortenzi F., Bossaglia S., "Modeling and Validation of a Two-Stroke Kart Engine", *Energies* 16 (2023) 4947, CC BY 4.0. <https://www.mdpi.com/1996-1073/16/13/4947> | The measured power curve (their Fig. 10) and the exhaust they modelled (their Fig. 9, the KA Type 2) |

## Engine (KA pp. 8-13, BIL p. 1)

- Bore 54.00 mm (54.28 max), stroke 54.40 mm, rod 102 mm centre to centre;
  swept 123.67 cm³.
- Combustion chamber 10.3 cm³ min (with the Australian insert; 12.8 cm³ min
  measured with the volumeter), squish 0.90 mm min (2.0 mm tin wire).
- Ports, from the cylinder development drawing (KA p. 11): one main exhaust
  port of chord B ≤ 36.5 mm with two small auxiliary exhaust ports beside it;
  two main transfers of chord C1 = C2 ≤ 30 mm and a third (rear) transfer of
  chord C3 ≤ 28.5 mm. Durations: exhaust (and auxiliaries) 177.5° max,
  main transfers 126 ± 2°, third transfer 127 ± 2°. So the exhaust opens
  about 91.25° ATDC and the transfers about 117° ATDC. 3 transfer ducts in
  the cylinder, 3 in the crankcase; the exhaust has 3 ports and 3 ducts.
- Piston with ring 128 g (one ring), pin 14 × 44 mm 28 g, rod 110 g min,
  crankshaft 2,150 g min, balance shaft 315 g (52 %).
- Induction: a reed valve into the crankcase (below). Carburettor Tillotson
  HW-27A, venturi 27 mm (BIL; the KA drawing p. 32 shows Ø 27.00 max,
  72.3 ± 0.3 long, Ø 29.10 max at the flange). Inlet conveyor (carburettor
  side) Ø 29.3 max, 37.5 deep, 12.5 min from the reed plane to the
  carburettor plane.
- Ignition: CDI with a 16,000 rpm limiter; the centrifugal clutch engages at
  4,000 rpm or below.

## Reed block (KA p. 22)

- A V block, 60 ± 1° included, two faces.
- Each face: one petal 75.5 mm max wide × 30 mm max long, slit into two
  tongues; two windows a face, 31.75 ± 0.5 mm (along the ridge) × 25.5 ± 0.5
  mm, separated by a 4 mm rib; block 76 ± 0.5 mm long.
- No stop plate is drawn.
- Petal material: glass-fibre laminate 0.30 mm min, or carbon 0.22 mm min
  (0.24 mm on the drawing).
- Not given: the free (bending) length between the clamp and the tip, the
  laminate's modulus and density, the petal's damping. The free length lies
  between the window length (25.5 mm) and the petal length (30 mm).

## Exhausts

**Type 1** (KA pp. 26, 31; volume 3,330 cm³ ± 5 %):

- Header: 65 ± 5 mm straight from the flange, centreline 166 ± 5 mm
  (developed), outlet Ø 42 ± 0.6 mm outside with a 1.5 mm wall, a 26 ± 1 mm
  slip joint.
- Pipe: inlet Ø 54 mm outside, 22 mm straight (B), a 218 mm cone (E) to
  Ø 100 mm outside (A), then 315 mm (F) holding a perforated reverse cone
  (8 rows of 8 holes, Ø 5.5 to 2.8 mm); 485 mm overall on the cone axis (D),
  180 mm (H); tailpipe Ø 24 mm outside (I), silencer 130 mm (K).

**Type 2** (KA p. 32 and OB Fig. 9; volume 4,250 cm³; the one OB modelled
and measured; all mm, "ext" = outside):

| Letter | Value | Letter | Value |
| --- | --- | --- | --- |
| A1 | 110 ± 1.5 ext (belly) | K | 170 |
| A2 | 102 ± 1.5 ext (end of the diffuser) | L | 42.5 ± 1.5 ext (port end) |
| A3 | 100 ± 1.5 ext (can) | M | 439 (bend, developed outer) |
| B1, B2 | 60, 60 | N | 341 (bend, developed inner) |
| C | 219 | O | 21 int |
| D1, D2 | 90, 109 | P | 50 |
| E | 23.5 ± 2 ext (reverse cone exit) | Q | 182 |
| F | 36 | S | 29 ± 1.5 (straight at the flange) |
| G | 35 ext | T | 690 (axis, bend end to rear) |
| H | 132 (silencer) | W | 170 |
| I | 21 int | | 68 holes Ø 4 in the reverse cone, wall 0.9 min |

A reading of the drawing (an interpretation, not a dimension): 29 mm
straight at about Ø 40.7 inside; a diverging bend of about 390 mm on its
centreline (the mean of M and N) from Ø 42.5 to Ø 102 outside; a 60 mm cone
to the Ø 110 belly; the belly about 100 mm long; then the perforated reverse
cone inside a Ø 100 can, down to Ø 23.5 outside; a 21 mm (inside) tailpipe
and a 132 mm silencer.

## Measured power (OB Fig. 10) — `measured-power.csv`

- The senior X30 (unrestricted) and the junior (a restrictor taking the
  header from Ø 39 to Ø 22.7 mm), power in kW against crank rpm, as
  measured on the authors' test bench.
- Read by eye from the published figure in a browser at full zoom, every
  500 rpm for the senior and at the readable points for the junior:
  **± 0.5 kW**. Blank junior cells were not read.
- Not stated in the article: the dyno type, the correction standard, the
  ambient conditions, the fuel, the spark advance and the carburettor
  setting of the measured runs (their model uses λ 0.85 and 22° BTDC). The
  raw data are "unavailable due to restrictions" (the article's data
  statement).
- OB fitted their own 1-D model to this curve (RMS about 0.5 kW): their model
  parameters are not a source for ours.

**How this curve may be used.** It is a comparison target only. No model
parameter of EngineLab may be fitted to it; a model parameter that has no
source stays at its literature or default value, and a comparison reports
the gap as measured (`.claude/CLAUDE.md`: no target-driven physics).

## Missing — blockers for a full comparison

- Port heights other than from the timings, the ports' corner radii, the
  auxiliary exhaust ports' chords, the transfer ducts' angles and areas.
- The crankcase volume (primary compression ratio).
- The reed's free length and laminate properties (above).
- Which exhaust the measured curve used is inferred from OB (they model
  Type 2); the article does not say so in words.
- The test conditions of the measured curve (above).

## The engine file and the comparison

`engines/iame_x30_senior_type2.engine.yaml` is the senior with the Type 2
exhaust, built from the numbers above only; its header names every value
no source gives and what stands in for it (a designer default, the
RD350LC's catalogue value, or an estimate). `comparison-points.csv` is the
senior curve in `EngineLabDynoSweepHarness --reference-file` form (the
16,000 rpm point lies on the limiter and is left out):

```
EngineLabDynoSweepHarness --catalog-root tests/reference-data/iame-x30 --reference-file tests/reference-data/iame-x30/comparison-points.csv
```

The 15 % tolerance is the harness's convention, not a claim; the results
are in `docs/journal.md` (2026-11-25, phase 7), not here.
