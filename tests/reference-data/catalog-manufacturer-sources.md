# Sources of the manufacturer reference points

The values in `catalog-manufacturer-points.csv` are net rated power and torque
figures published by the manufacturer. They never come from EngineLab output.
The common ±15 % threshold is the project's contract, not an uncertainty
attributed to the manufacturer data sheets.

| Catalogue family | Reference variant | Primary source |
|---|---|---|
| Yamaha CP2 | European MT-07, 54.0 kW at 8,750 rpm and 68.0 Nm at 6,500 rpm | [Yamaha Motor Europe — MT-07](https://www.yamaha-motor.eu/es/es/motorcycles/hyper-naked/pdp/mt-07/) |
| Yamaha CP3 | European MT-09 890, 87.5 kW at 10,000 rpm and 93.0 Nm at 7,000 rpm | [Yamaha Motor — MT-09 press release](https://global.yamaha-motor.com/jp/news/2020/1028/mt-09.html) |
| Yamaha CP4 | European MT-10, 118.0 kW at 11,500 rpm and 111.0 Nm at 9,000 rpm | [Yamaha Motor — MT Garage](https://global.yamaha-motor.com/showroom/mt/garage/) |
| VW EA288 | Golf 2.0 TDI 110 kW, peak power between 3,500 and 4,000 rpm and 320 Nm between 1,750 and 3,000 rpm | [Volkswagen Newsroom — EA288 Golf](https://www.volkswagen-newsroom.com/en/the-new-golf-das-auto-international-driving-presentation-2797/the-new-golf-powertrain-structure-engines-and-gearboxes-2835) |
| Honda K20A | Accord Euro-R, 162 kW and 206 Nm net | [Honda — Accord Euro-R launch](https://global.honda/jp/news/2002/4021010-accord.html) |
| GM LS3 | LS3 6.2 l, 430 hp at 5,900 rpm and 425 lb-ft at 4,600 rpm (rounded to SI in the CSV: 321 kW and 575 Nm) | [Chevrolet Performance — LS3](https://www.chevrolet.com/performance-parts/crate-engines/ls-lsx-engines/ls3-engine) |
| Toyota 2JZ-GTE | Export Supra A80, 320 hp and 315 lb-ft (rounded to SI in the CSV: 239 kW and 427 Nm) | [Toyota USA Newsroom — Supra history](https://pressroom.toyota.com/toyota-supra-icon-half-century-in-making/) |
| Subaru EJ25 | 2021 WRX STI, EJ257 2.5 l: 310 hp at 6,000 rpm and 290 lb-ft between 4,000 and 5,200 rpm (231 kW and 393 Nm in the CSV) | [Subaru of America — 2021 WRX/STI brochure, p. 6](https://www.subaru.com/content/dam/subaru/downloads/pdf/brochures/2021/wrx/2021_WRX_Brochure.pdf) |
| Audi I5 | RS 3 2.5 TFSI: 294 kW between 5,600 and 7,000 rpm, 500 Nm between 2,250 and 5,600 rpm, and 2.5 bar absolute boost | [Audi MediaCenter — RS 3 data sheet](https://uploads.audi-mediacenter.com/system/production/car_motorizations/468/file_en/1d135b0858d0dbde3c70b0690ae8652197730017/eTD-Audi-RS3-Limousine-TFSI_240814.pdf), [Audi MediaCenter — 50 years of the five-cylinder](https://www.audi-mediacenter.com/en/press-releases/50-years-of-the-audi-five-cylinder-16921) |
| Suzuki Hayabusa | 1999 GSX1300R, 1,298 cm³ and 81 × 63 mm bore × stroke: 128.7 kW at 9,800 rpm and 138.2 Nm at 7,000 rpm | [Global Suzuki — Hayabusa digital archive](https://www.globalsuzuki.com/motorcycle/smgs/digital-archive/2_bike/sports_078.php) |
| Big Twin | Milwaukee-Eight 117 Classic, 4.075 × 4.5 in bore × stroke: 120 lb-ft SAE J1349 at 2,500 rpm and 73 kW at 4,600 rpm (162.7 Nm in the CSV) | [Harley-Davidson — 2026 Super Glide](https://www.harley-davidson.com/us/en/motorcycles/super-glide.html) |
| Porsche flat-six | 911 type 964 Carrera 2, 3.6 l and 11.3:1 compression: 184 kW at 6,100 rpm and 310 Nm at 4,800 rpm | [Porsche Newsroom — 964 Carrera 2](https://newsroom.porsche.com/de_CH/2019/historie/porsche-klassik-911-carrera-964-c2-lappland-18951.html) |

The K20A and 2JZ-GTE engine speeds in the CSV are the rated points of their
variant data sheets (8,000/7,000 and 5,600/4,000 rpm). The historical pages
above confirm the net magnitudes but no longer show every column of the data
sheet in their current HTML; keep that distinction in any future claim.

The Subaru, Audi, Suzuki, Harley-Davidson and Porsche references are matched on
displacement, bore/stroke geometry and the explicitly modelled variant. The
`Merlin-like 19.8 V12 Scaled` and `Radial-like 6.5 R5` engines stay out of the
gate: the first is deliberately scaled down from the real 27-litre Merlin, and
the second does not stand for any precise certified model.
