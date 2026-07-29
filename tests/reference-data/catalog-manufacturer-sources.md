# Sources des points constructeur

Les valeurs de `catalog-manufacturer-points.csv` sont des puissances nettes et
couples nominaux annoncés par le constructeur. Elles ne proviennent jamais
d'une sortie d'EngineLab. Le seuil commun de ±15 % est le contrat du projet, pas
une incertitude attribuée aux fiches constructeur.

| Famille du catalogue | Variante de référence | Source primaire |
|---|---|---|
| Yamaha CP2 | MT-07 européenne, 54,0 kW à 8 750 tr/min et 68,0 Nm à 6 500 tr/min | [Yamaha Motor Europe — MT-07](https://www.yamaha-motor.eu/es/es/motorcycles/hyper-naked/pdp/mt-07/) |
| Yamaha CP3 | MT-09 européenne 890, 87,5 kW à 10 000 tr/min et 93,0 Nm à 7 000 tr/min | [Yamaha Motor — communiqué MT-09](https://global.yamaha-motor.com/jp/news/2020/1028/mt-09.html) |
| Yamaha CP4 | MT-10 européenne, 118,0 kW à 11 500 tr/min et 111,0 Nm à 9 000 tr/min | [Yamaha Motor — MT Garage](https://global.yamaha-motor.com/showroom/mt/garage/) |
| VW EA288 | Golf 2.0 TDI 110 kW, puissance maximale entre 3 500 et 4 000 tr/min et 320 Nm entre 1 750 et 3 000 tr/min | [Volkswagen Newsroom — EA288 Golf](https://www.volkswagen-newsroom.com/en/the-new-golf-das-auto-international-driving-presentation-2797/the-new-golf-powertrain-structure-engines-and-gearboxes-2835) |
| Honda K20A | Accord Euro-R, 162 kW et 206 Nm nets | [Honda — lancement Accord Euro-R](https://global.honda/jp/news/2002/4021010-accord.html) |
| GM LS3 | LS3 6,2 l, 430 hp à 5 900 tr/min et 425 lb-ft à 4 600 tr/min (arrondis SI du CSV : 321 kW et 575 Nm) | [Chevrolet Performance — LS3](https://www.chevrolet.com/performance-parts/crate-engines/ls-lsx-engines/ls3-engine) |
| Toyota 2JZ-GTE | Supra A80 export, 320 hp et 315 lb-ft (arrondis SI du CSV : 239 kW et 427 Nm) | [Toyota USA Newsroom — histoire de la Supra](https://pressroom.toyota.com/toyota-supra-icon-half-century-in-making/) |
| Subaru EJ25 | WRX STI 2021, EJ257 2,5 l : 310 hp à 6 000 tr/min et 290 lb-ft entre 4 000 et 5 200 tr/min (231 kW et 393 Nm dans le CSV) | [Subaru of America — brochure WRX/STI 2021, p. 6](https://www.subaru.com/content/dam/subaru/downloads/pdf/brochures/2021/wrx/2021_WRX_Brochure.pdf) |
| Audi I5 | RS 3 2.5 TFSI : 294 kW entre 5 600 et 7 000 tr/min, 500 Nm entre 2 250 et 5 600 tr/min et 2,5 bar absolus de suralimentation | [Audi MediaCenter — fiche technique RS 3](https://uploads.audi-mediacenter.com/system/production/car_motorizations/468/file_en/1d135b0858d0dbde3c70b0690ae8652197730017/eTD-Audi-RS3-Limousine-TFSI_240814.pdf), [Audi MediaCenter — 50 ans du cinq-cylindres](https://www.audi-mediacenter.com/en/press-releases/50-years-of-the-audi-five-cylinder-16921) |
| Suzuki Hayabusa | GSX1300R 1999, 1 298 cm³ et géométrie 81 × 63 mm : 128,7 kW à 9 800 tr/min et 138,2 Nm à 7 000 tr/min | [Global Suzuki — bibliothèque numérique Hayabusa](https://www.globalsuzuki.com/motorcycle/smgs/digital-archive/2_bike/sports_078.php) |
| Big Twin | Milwaukee-Eight 117 Classic, géométrie 4,075 × 4,5 pouces : 120 lb-ft SAE J1349 à 2 500 tr/min et 73 kW à 4 600 tr/min (162,7 Nm dans le CSV) | [Harley-Davidson — Super Glide 2026](https://www.harley-davidson.com/us/en/motorcycles/super-glide.html) |
| Porsche flat-six | 911 type 964 Carrera 2, 3,6 l et compression 11,3:1 : 184 kW à 6 100 tr/min et 310 Nm à 4 800 tr/min | [Porsche Newsroom — 964 Carrera 2](https://newsroom.porsche.com/de_CH/2019/historie/porsche-klassik-911-carrera-964-c2-lappland-18951.html) |

Les régimes K20A et 2JZ-GTE du CSV correspondent aux points nominaux de leurs
fiches de variante (8 000/7 000 et 5 600/4 000 tr/min). Les pages historiques
ci-dessus confirment les magnitudes nettes mais n'exposent plus toutes les
colonnes de la fiche technique dans leur rendu HTML actuel ; conserver cette
distinction dans toute affirmation future.

Les références Subaru, Audi, Suzuki, Harley-Davidson et Porsche sont appariées
à la cylindrée, à la géométrie d'alésage/course et à la variante explicitement
modélisées. Les moteurs `Merlin-like 19.8 V12 Scaled` et `Radial-like 6.5 R5`
restent exclus du gate : le premier est volontairement réduit par rapport au
Merlin réel de 27 litres et le second ne désigne aucun modèle certifié précis.
