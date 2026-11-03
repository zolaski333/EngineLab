# Designed engines against their makers

`designed-engines.spec` builds engines with the designer from what a data sheet
publishes, as a user does; `designed-manufacturer-points.csv` holds the makers'
rated points they are judged on. This is the gate the catalogue cannot be: a
catalogue engine carries per-engine values fitted to its own reference points
(chamber turbulence 0.42-2.0, friction coefficients, edited spark curves), a
designed one carries none.

## Rules

- **Only data-sheet values** go in a spec: cycle, use, layout, cylinders, bank
  angle, bore, stroke, compression ratio, valves per side when the sheet says
  so, and the redline at the rated power speed / 0.9 (the designer's own rule
  read backwards). Nothing is ever set to move a result.
- **The control set is never used to decide a change**, only to check that a
  change decided on the tuning set generalises (VISION.md, success criterion).
- **What the designer cannot express is left out**, not approximated: the
  CP2's 270 deg and the CP4's crossplane cranks (DesignBench takes no crank
  offsets; they fire evenly here), the K20A's VTEC, the real exhausts' layout
  (4-2-1 and the like). Engines whose rated figures depend on a variable cam or
  variable intake length (BMW S 1000 RR ShiftCam, Ducati V4's variable
  trumpets) were not taken.

## Tuning set

| Engine | Data | Source |
|---|---|---|
| Honda CBR1000RR-R Fireblade SP (2020) | 81 x 48.5 mm, 13.0:1; 160 kW at 14,500, 113 Nm at 12,500 | [Honda Motor Europe — 2020 CBR1000RR-R Fireblade SP press kit](https://hondanews.eu/gb/en/motorcycles/media/pressreleases/196428/2020-honda-cbr1000rr-r-fireblade-sp) |
| Yamaha CP4 (MT-10) | 79.0 x 50.9 mm, 12.0:1; 118 kW at 11,500, 111 Nm at 9,000 | as `catalog-manufacturer-sources.md` |
| Suzuki Hayabusa (1999) | 81 x 63 mm; 128.7 kW at 9,800, 138.2 Nm at 7,000 | [Global Suzuki digital archive](https://www.globalsuzuki.com/motorcycle/smgs/digital-archive/2_bike/sports_078.php); 11.0:1 is not on Suzuki's page: secondary listings (Suzuki-sourced spec tables, 2003 GSX1300R) agree on 11.0, later years are 12.5 |
| Yamaha CP2 (MT-07) | 80.0 x 68.6 mm, 11.5:1; 54 kW at 8,750, 68 Nm at 6,500 | as `catalog-manufacturer-sources.md` |
| Yamaha CP3 (MT-09 890) | 78.0 x 62.1 mm, 11.5:1; 87.5 kW at 10,000, 93 Nm at 7,000 | as `catalog-manufacturer-sources.md` |
| Honda K20A (Accord Euro-R) | 86 x 86 mm, 11.5:1; 162 kW at 8,000, 206 Nm at 7,000 | as `catalog-manufacturer-sources.md` (VTEC not expressed) |
| GM LS3 | 103.25 x 92 mm, 10.7:1, two valves; 321 kW at 5,900, 575 Nm at 4,600 | as `catalog-manufacturer-sources.md` |
| Harley-Davidson Milwaukee-Eight 117 | 103.5 x 114.3 mm, 10.2:1, four valves, 45 deg V; 73 kW at 4,600, 162.7 Nm at 2,500 (J1349) | as `catalog-manufacturer-sources.md` |

## Control set

| Engine | Data | Source |
|---|---|---|
| GM LS7 (Corvette Z06) | 104.8 x 101.6 mm, 11.0:1, two valves; 505 hp (377 kW) at 6,300, 470 lb-ft (637 Nm) at 4,800, SAE certified | GM Powertrain data as reproduced by [Corvette Action Center — 2011 LS7](https://corvetteactioncenter.com/specs/c6/2011/2011-corvette-ls7-engine.html) (independent of GM) |
| Ducati 1299 Panigale (2015) | 116 x 60.8 mm, 12.6:1; 205 hp (151 kW) at 10,500, 144.6 Nm at 8,750 | Ducati's launch figures as reproduced by [Ultimate Motorcycling — 2015 1299 Panigale](https://ultimatemotorcycling.com/2014/11/03/2015-ducati-1299-panigale-first-look-205-hp-367-lbs/); the torque is quoted 144.6 Nm by spec listings, "just over 106 lb-ft" by magazines |
| Honda CBR1000RR (2008, SC59) | 76.0 x 55.1 mm, 12.3:1; 131 kW at 12,000, 112 Nm at 8,500 | bore, stroke and ratio from [Honda Japan factbook (SC59E)](https://www.honda.co.jp/factbook/motor/cbr1000rr/200807/018.html); the European rating from secondary listings (Honda's 2008 press kit PDF was not read) |

The control set's sources are weaker than the tuning set's (third parties
quoting the makers): a control figure that disagrees by a few percent is read
with that in mind.
