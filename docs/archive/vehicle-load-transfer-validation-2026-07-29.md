# Validation du transfert de charge longitudinal — 2026-07-29

## Modèle livré

La limite d'adhérence n'utilise plus une charge verticale figée. À chaque
sous-pas mécanique de 1 ms, le modèle applique l'équilibre quasi-statique de
tangage :

```text
delta_Fz = masse * acceleration_longitudinale * hauteur_CG / empattement
```

Pour une accélération positive, `delta_Fz` est retranché à l'essieu avant et
ajouté à l'essieu arrière. Une transmission intégrale utilise la somme des
charges avant et arrière, soit `masse * g`. La charge résultante est bornée entre
zéro et le poids total avant d'appliquer `limite = mu * charge_normale`.

Le schéma 5 persiste maintenant :

- `driven_axle_layout`: `front`, `rear` ou `all` ;
- `driven_axle_weight_fraction`: charge statique de l'essieu moteur en 2RM ;
- `wheelbase_m` ;
- `center_of_gravity_height_m`.

Les variantes de véhicule du catalogue séparent désormais traction, propulsion,
transmission intégrale, moto et banc fixe. Les moteurs qui partageaient à tort
le même véhicule générique ont été séparés : K20A/traction, Audi/intégrale,
Porsche/propulsion arrière et EA288/traction.

## Témoin déterministe

`EngineLab.VehicleDynamics` lance quatre châssis strictement identiques
(1 200 kg, `mu = 1`, 50 % de charge statique, empattement 2,50 m, CG 0,52 m)
avec assez de couple pour réellement saturer les pneus.

| Motricité | direction | accélération | charge normale motrice | force pneu | vitesse après le témoin |
|---|---|---:|---:|---:|---:|
| avant | avant | +4,058 m/s² | 4 871 N | +4 871 N | +2,407 m/s |
| arrière | avant | +6,186 m/s² | 7 428 N | +7 428 N | +3,667 m/s |
| intégrale | avant | +9,796 m/s² | 11 768 N | +11 768 N | +5,811 m/s |
| arrière | marche arrière | -4,058 m/s² | 4 871 N | -4 871 N | -2,407 m/s |

Les quatre témoins publient `tractionLimited=yes`; le test ne peut donc pas
passer sans exercer la branche corrigée. Il vérifie également :

- le gain matériel d'une propulsion sur une traction lors d'un départ avant ;
- l'inversion du transfert en marche arrière ;
- l'invariance du poids total disponible en transmission intégrale ;
- les round-trips JSON et YAML du schéma 5 ;
- le rejet d'une géométrie où la hauteur du CG atteint l'empattement ;
- le chargement des 14 moteurs avec leur motricité de catalogue.

Commande de reproduction :

```powershell
ctest --test-dir out/build/windows-vs2022 -C Release `
  -R '^EngineLab\.VehicleDynamics$' --output-on-failure -V
```

`EngineLab.Core` reste vert après la migration de schéma et protège en plus les
invariants historiques de l'embrayage, du freinage et de l'énergie.
