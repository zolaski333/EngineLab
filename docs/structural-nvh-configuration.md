# Modes NVH structurels mesurés ou calculés

## Principe

Le rayonnement mécanique est toujours excité par les forces SI publiées par le
solveur : effort gazeux sur le piston, réactions axiale et latérale de palier,
et couple de réaction du vilebrequin. Une configuration NVH ne remplace donc pas
le moteur par des oscillateurs “musicaux” ; elle remplace uniquement le jeu de
modes réduit qui transforme ces efforts en vitesse de surface puis en pression
rayonnée.

Sans section `structural_nvh`, EngineLab conserve son modèle de famille
coque/poutre/plaques et publie la provenance `estimatedFamily`. Aucun des 14
moteurs livrés ne prétend actuellement posséder une mesure NVH constructeur.
Pour ce fallback, la coordonnée longitudinale de chaque cylindre vient de sa
position dans `banks[].cylinderIds`, jamais de son index global. Cette distinction
est indispensable aux configurations V et flat dont le stockage alterne les
bancs. Le correctif et ses A/B sont documentés dans
[`audio-lot4-structural-bank-topology-2026-07-29.md`](audio-lot4-structural-bank-topology-2026-07-29.md).

## Schéma 5

Exemple de forme, volontairement illustratif et **non utilisable comme
calibration d'un moteur réel** :

```yaml
structural_nvh:
  provenance: measured
  source: "rapport modal, montage et révision identifiables"
  modes:
    - name: "flexion verticale bloc 1"
      drive: bearing_axial
      frequency_hz: 1234.5
      damping_ratio: 0.025
      modal_mass_kg: 4.2
      radiating_area_m2: 0.18
      radiation_efficiency: 0.42
      surface_velocity_rms_scale: 0.50
      torque_radius_m: 0.06
      cylinder_participation: [1.0, -0.65, 0.65, -1.0]
```

Valeurs acceptées :

- `provenance`: `estimated_family`, `calculated_geometry` ou `measured` ;
- `drive`: `head_gas`, `bearing_axial`, `bearing_lateral` ou `torsion` ;
- une participation signée, normalisée à l'antinœud, par cylindre et dans
  l'ordre de `engine.cylinders` ;
- fréquence, amortissement, masse modale, aire rayonnante, efficacité de
  rayonnement, facteur RMS de forme et rayon équivalent de couple en unités SI.

`torque_radius_m` n'est consommé que par un mode `torsion`, mais il reste
persisté pour que tous les modes partagent un schéma fixe. Il n'existe aucun
champ de gain arbitraire.

## Conditions pour déclarer `measured`

Une campagne exploitable doit au minimum conserver :

1. le montage, les points d'impact/excitation et les accéléromètres ou le champ
   vibrométrique ;
2. les FRF ayant fourni fréquence et amortissement ;
3. la convention de normalisation de la forme modale et la masse modale
   cohérente avec elle ;
4. le calcul ou la mesure de l'aire/efficacité rayonnante ;
5. l'identifiant du rapport ou du dataset dans `source`.

La validation refuse :

- une provenance mesurée sans mode ;
- un jeu de modes sans source ;
- plus de 64 modes ;
- une forme qui n'a pas exactement une valeur par cylindre ;
- une forme nulle, non finie ou hors de l'intervalle `[-1, 1]` ;
- les paramètres non physiques ou hors bande.

Cette discipline empêche de transformer une estimation agréable à l'oreille en
fausse donnée constructeur.

## Preuve automatisée

`EngineLab.StructuralNvh` construit un mini-catalogue temporaire contenant un
mode de banc synthétique à 1 234,5 Hz. Le test prouve :

- le chargement par le même décodeur YAML que le catalogue de production ;
- la provenance et la source accessibles au runtime ;
- la transmission exacte de fréquence, amortissement, masse, aire et
  efficacité ;
- les round-trips JSON/YAML ;
- une réponse finie et non nulle à la force de palier à la résonance ;
- le rejet d'une fausse provenance mesurée et d'une forme incomplète ;
- le maintien du fallback `estimatedFamily` lorsque la section est absente.

Commande :

```powershell
ctest --test-dir out/build/windows-vs2022 -C Release `
  -R '^EngineLab\.StructuralNvh$' --output-on-failure -V
```
