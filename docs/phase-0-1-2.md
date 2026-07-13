# Livraison des phases 0, 1 et 2

État vérifié le 12 juillet 2026. Ce document remplace les anciennes mentions
qui présentaient encore la bielle maîtresse radiale comme hors périmètre.

## Phase 0 — mesure déterministe

La cible `EngineLabComparisonHarness` exécute quatre scénarios fixes : I4, V8,
V-twin et radial cinq cylindres. Pour chaque scénario, elle produit :

- une trace CSV commune (régime, angle, couple, puissance, pressions, AFR,
  lambda, débit carburant, chaleur résolue, températures, énergie gazeuse et
  état du premier cylindre) ;
- un WAV diagnostic déterministe dérivé des événements de combustion ;
- un `summary.csv` avec régime, couple, AFR, carburant, pression de pointe et
  métriques audio ;
- un mode `--reference` qui échoue si les tolérances de régression sont
  dépassées.

Exécution :

```powershell
cmake --build build --config Release --target EngineLabComparisonHarness
build\tools\Release\EngineLabComparisonHarness.exe --output comparison-output
build\tools\Release\EngineLabComparisonHarness.exe `
  --output comparison-candidate --reference comparison-output\summary.csv
```

Le harnais ne contient pas encore de vérité terrain issue de l'exécutable ES2D
ni de mesures de banc réelles. Le WAV du harnais est un signal de comparaison
stable, pas un export hors ligne du renderer JUCE complet.

## Phase 1 — erreurs et cohérence physique

Les corrections livrées sont les suivantes :

- pause réellement totale : transmission, véhicule et distance ne progressent
  plus pendant une pause ; le pas transmission suit le même facteur temporel
  que le moteur ;
- admission canonique : `intake` pilote les anciens champs
  `plenumVolumeLitres`/`throttleDiameterMm`, qui sont normalisés et validés ;
- `intakePaths` réellement utilisés, avec un plénum conservatif par chemin et
  validation des `bank.intakeId` ;
- volume de runner calculé par `πr²L`, au lieu du volume constant de 0,18 L ;
- cohérence obligatoire `course = 2 × maneton` ;
- AFR affiché mesuré sur la vapeur réellement présente à l'étincelle ; débit et
  consommation calculés sur la masse effectivement mesurée par l'injecteur ;
- thermique alimentée par l'énergie réellement libérée par les réactions de
  chaque cylindre ; EGT issue des cellules d'échappement ;
- volume de flamme corrigé en ellipsoïde `4/3 πr²a` ;
- suppression du clamp arbitraire du couple alternatif ; effort gaz et effort
  d'inertie utilisent le même bras de levier cinématique ;
- suralimentation avec puissances compresseur/turbine observables, turbo borné
  par l'énergie disponible et compresseur mécanique prélevé au vilebrequin ;
- banc stabilisé pendant au moins deux cycles à bas régime, et non 0,10 s
  indépendamment du régime ;
- nombre de chemins d'échappement limité aux huit voies audio réellement
  supportées ;
- pression audio continue traitée par cylindre et spatialisée depuis la vraie
  banque, au lieu d'une moyenne mono et d'une parité d'index ;
- commande d'embrayage maintenue reconfigurable (`Y` par défaut), avec `Shift`
  conservé comme raccourci de compatibilité ; W/E passent à 10/20 %.

## Phase 2 — topologie mécanique commune

`MechanicalKinematics` est désormais la source de vérité commune. Elle fournit
à chaque cylindre position/course/vitesse/accélération du piston, angle de
bielle, coordonnées du maneton et de l'axe, volume de chambre et dérivée de
déplacement utilisée pour le couple.

La configuration JSON/YAML et le catalogue savent décrire :

- jusqu'à huit vilebrequins, avec origine, phase, rapport de rotation, inertie
  et frottement ;
- le rattachement de chaque maneton à un vilebrequin ;
- bielles conventionnelles, maîtresses et articulées ;
- rayon/angle d'articulation et cylindre maître ;
- hauteur de deck, hauteur de compression, décalage d'axe, volume de calotte,
  chambre de culasse et joint de culasse.

Le radial R5 livré utilise une bielle maîtresse et quatre bielles articulées.
La simulation et la vue radiale consomment le même état cinématique live. Les
anciens fichiers schema-v1 sans cette topologie sont migrés vers un vilebrequin
et un maneton par cylindre, sans branche spécifique dans le solveur.

## Ce qui reste volontairement non livré

- comparaison A/B automatisée contre ES2D et corpus de mesures réelles ;
- degrés de liberté dynamiques indépendants pour plusieurs vilebrequins
  (torsion, jeu d'engrenage, embrayage entre arbres). Les rapports configurés
  sont actuellement des contraintes cinématiques rigides ;
- réseau d'ondes gaz/acoustique 1D maillé ou CFD ;
- chimie multi-espèces, spray 3D et modèle thermique éléments finis ;
- export WAV du renderer complet depuis l'interface ;
- refonte UI/UX complète au niveau d'ES2D, scripting de configuration et import
  externe de profils de came ;
- calibration perceptuelle finale contre ES2D sur un protocole d'écoute commun.

Les phases 3 à 5 ajoutent le travail P·dV, la chaîne de transmission
énergétique, la distribution continue et la résonance Helmholtz, sans prétendre
fermer les limites ci-dessus. Leur contrat détaillé est documenté dans
[`phase-3-4-5.md`](phase-3-4-5.md).
