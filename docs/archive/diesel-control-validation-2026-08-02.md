# Validation commande Diesel — 2026-08-02

## Défaut reproduit

Le Diesel catalogue utilisait deux limites distinctes qui n'étaient pas
présentées comme telles :

- `targetAirFuelRatio` était interprété par la physique comme un **plancher de
  fumée** (`AFR stœchiométrique × 1,16`, soit 16,99 pour l'EN 590) ;
- `InjectionConfig::fullLoadFuelLimit` était la vraie carte de quantité et de
  couple, en mg par cylindre et par cycle.

Le tuner ne publiait pourtant que la première, avec une table essence située
principalement entre 12 et 14,7 et un plafond d'édition à 18. Le plancher Diesel
écrasait donc presque chaque modification. En parallèle,
`EngineDiagnostics` appliquait le défaut essence `lambda > 1,03` au Diesel :
un fonctionnement normalement pauvre devenait une alerte critique permanente.

## Contrat livré

- `fuel.target_afr` devient, pour un Diesel, **Limite fumée AFR** avec une plage
  16,99–40 sur le TDI. Une valeur plus grande réduit le carburant ; la valeur
  réelle peut normalement être plus pauvre que cette limite.
- `fuel.diesel_quantity_mg_per_cycle` est une nouvelle courbe RPM éditable à
  chaud. Elle reprend exactement la carte de quantité auteur et commande le
  plafond d'injection par cylindre/cycle.
- Une ECU personnalisée qui ne publie pas cette nouvelle commande conserve la
  courbe `EngineConfig` comme fallback ; il n'y a pas de rupture silencieuse.
- La limite fumée est appliquée **après** les corrections transitoires et reste
  prioritaire sur une hausse de quantité.
- Le diagnostic essence « mélange trop pauvre » est désactivé uniquement pour
  la compression ignition. Un nouveau diagnostic Diesel signale l'autre côté :
  AFR réel inférieur au plancher fumée sous charge.
- L'interface affiche `AFR réel ≥ limite fumée`, adapte la jauge, et nomme le
  trim `TRIM LIMITE FUMÉE (+ = MOINS DE GAZOLE)`.

## Preuve d'autorité physique

Commande :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabDieselControlHarness.exe
```

Point tenu : VW EA288-like 2.0 TDI I4, 2 000 tr/min, plein gaz.

| Carte | Commande mg/cyl/cycle | RPM | Carburant mg/cycle moteur | AFR | Plancher fumée | Couple Nm | Faux diag pauvre |
|---|---:|---:|---:|---:|---:|---:|---:|
| réduite, ×0,65 | 34,15 | 1 995,32 | 136,50 | 26,90 | 16,99 | 216,96 | non |
| série, ×1,00 | 52,55 | 1 996,05 | 208,74 | 17,62 | 16,99 | 346,82 | non |
| hausse, ×1,20 | 63,06 | 1 996,18 | 212,54 | 17,22 | 16,99 | 353,37 | non |

La réduction de 35 % de la carte entraîne −34,6 % de carburant réellement
injecté et −37,4 % de couple. La hausse rencontre volontairement le plancher
fumée : son faible gain résiduel est la preuve que la protection indépendante
borne bien la commande, pas que la carte est ignorée.

Le harnais exige : tenue RPM ±3 %, ordre des commandes, autorité carburant et
couple, convergence de la hausse sur le plancher fumée, absence de dépassement
riche et absence du faux diagnostic essence. Tous les critères passent.

## Non-régression constructeur

Commande :

```powershell
ctest --test-dir out/build/windows-vs2022 -C Release --output-on-failure -R "^EngineLab.CatalogReference$"
```

Résultat complet : **24/24 points catalogue dans ±15 %**. Points Diesel :

| Mesure | Régime tenu | Simulé | Référence | Erreur | Résultat |
|---|---:|---:|---:|---:|---:|
| couple | 1 996,99 tr/min | 348,886 Nm | 320 Nm | +9,027 % | PASS |
| puissance | 4 001,82 tr/min | 111,689 kW | 110 kW | +1,535 % | PASS |

Tests ciblés Release :

- `EngineLab.VehicleDynamics` : sémantique de diagnostic essence/Diesel ;
- `EngineLab.EcuCalibration` : présence, publication atomique et lecture live
  de la courbe de quantité ;
- `EngineLab.DieselControl` : autorité physique mesurée ci-dessus.

Résultat : **3/3 en 8,63 s**.

## Limite assumée

La carte de quantité peut réduire directement le couple. Une hausse n'est pas
autorisée à rendre le mélange plus riche que la limite fumée : pour obtenir
plus de couple proprement il faut d'abord admettre davantage d'air, pas
neutraliser cette protection. Le modèle conserve donc deux commandes séparées
au lieu de faire croire qu'une « AFR cible » Diesel se comporte comme une
lambda essence en boucle fermée.
