# Validation de l'afterfire de décélération — 2026-08-01

## Problème corrigé

Le réseau savait oxyder du carburant imbrûlé, mais la DFCO supprimait toute
injection au lever de pied. Le seul chemin volontairement humide était le
rupteur. De plus, l'injection physique était conditionnée par le drapeau de
combustion cylindre : une commande `fuel=true, spark=false` pouvait donc être
annulée avant d'atteindre le cylindre.

## Contrat livré

- le défaut reste exact : `overrun_fuel_fraction: 0` conserve la DFCO propre ;
- une valeur positive n'est active que sur essence, afterfire activé, DFCO
  fermée, étincelle autorisée par le conducteur, papillon sous le seuil et RPM
  au-dessus du seuil ;
- l'armement exige auparavant plus de 20 % de demande conducteur au-dessus du
  seuil RPM. Un flare de démarrage ne peut donc pas déclencher le mode ;
- l'étincelle est coupée et la fraction de carburant reste exactement bornée :
  le reliquat d'enrichissement du coup de gaz ne peut pas la multiplier ;
- aucune impulsion sonore n'est programmée. La réaction exige toujours
  carburant, oxygène et température dans le réseau physique.

## Preuves Release

`EngineLab.Core` et `EngineLab.OfflineAudioExport` passent ensemble (`2/2`,
`0` échec). Les oracles ECU couvrent le défaut propre, l'armement après demande
conducteur, la commande partielle spark-cut, le seuil RPM et les round-trips
JSON/YAML/script.

Le binaire de test hors ligne a publié :

```text
Audio physics raw proof: afterfire=121.608 kW / 6479.05 mg, overrun_frames=267
Audio physics A/B: delta RMS=0.0627879, peak=0.82579,
cycle=0.858323..1.14009, afterfire=121.608 kW / 6479.05 mg,
overrun_frames=267
```

Ce sont trois preuves distinctes : `overrun_frames` démontre la décision ECU,
les kW et mg démontrent une réaction chimique réelle, le delta WAV démontre que
la chaîne acoustique reçoit une sortie différente. La masse et la puissance ne
sont pas une calibration constructeur ; le moteur laboratoire est volontairement
une estimation audible.

## Essai dans l'application

Sélectionner `Audio Physics Lab 689 Twin`, ouvrir **AUDIO HQ**, cliquer **DEMO
AUDIBLE**, dépasser 3 000 tr/min avec plus de 20 % d'accélérateur puis relâcher.
La ligne LIVE doit afficher `decel ACTIVE` pendant la stratégie. Les compteurs
kW/mg/s ne montent que si le réseau est assez chaud et oxygéné. **BYPASS** remet
la fraction à zéro et désactive le modèle.
