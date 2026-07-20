# Architecture thermoacoustique physique

Ce document décrit l’implémentation livrée et son contrat physique. La chaîne
d’échappement par défaut n’est plus un voicing de presets : sa source, sa
propagation et son rayonnement dérivent de grandeurs SI produites par la
simulation. Toute modification future doit préserver cette séparation.

## Chaîne active

```text
combustion et chambre 0D (GasCell)
        ↕ flux de Riemann conservatif à chaque soupape
réseau gaz quasi-1D non linéaire (DAG complet, bande de retour physique)
        → frontière SI instantanée p, ρ, c, ṁ, CdA
décomposition en caractéristiques p+ / p−
        ↔ guides d’onde + jonctions à admittance
charge passive de rayonnement de la sortie
        → pression libre à 1 m + retard r/c
        → IR mesurée explicitement fournie, facultative
```

Il n’existe aucun repli runner/collecteur 0D dans `EngineSimulator`. Une
topologie physique invalide empêche la construction du simulateur. Dès qu’une
frontière SI valide atteint le renderer, le chemin physique est verrouillé :
les voix d’échappement procédurales actives et planifiées sont retirées et une
perte ultérieure de télémétrie ne les réactive jamais.

## 1. Réseau gaz non linéaire

`EngineLabGasDynamics` transporte, par volume fini :

- la masse de chaque espèce (`O₂`, inerte, carburant, gaz brûlés) ;
- la quantité de mouvement axiale ;
- l’énergie totale.

Le noyau utilise un flux HLLC avec repli de sûreté HLLE, une reconstruction TVD,
SSP-RK2 et un pas CFL. Une tentative non physique est rejetée puis reprise avec
un pas réduit ; aucune masse ni énergie n’est créée par un plancher numérique.
Les pertes locales, le frottement de paroi et le transfert thermique sont des
termes sources déclarés. Les parois du réseau moteur sont actuellement
adiabatiques, car leur inertie thermique n’est pas encore un sous-système
conservé.

`ExhaustNetworkLayout` compile chaque composant auteur : tubes, catalyseurs,
silencieux, résonateurs et sorties deviennent des conduits ; merges et splitters
deviennent des volumes de jonction finis. Longueur, volume, section de connexion,
diamètre hydraulique, perte et coefficient de décharge gardent leur unité et
leur propriétaire. L’ordre du tableau de frontières ne change pas le résultat.

### Séparation d’échelles temps réel

Résoudre tout le spectre audible par volumes finis imposerait des dizaines de
milliers de mises à jour par seconde et par cellule. L’implémentation adopte une
décomposition multirate physique :

- le maillage non linéaire a une longueur de cellule maximale de 300 mm ; il
  résout le débit moyen, la contre-pression et les fondamentales d’allumage
  jusqu’à environ 330–360 Hz dans les gaz chauds ;
- un composant plus court reste un volume de contrôle conservatif unique, avec
  son volume, ses ports et ses pertes exacts ; son délai audio appartient au
  réseau caractéristique ;
- la frontière macro est intégrée au minimum 16 fois par période d’allumage,
  avec une fenêtre absolue de 500 µs au démarrage ;
- état conservatif, volume de chambre, `CdA` de soupape et ouverture de sortie
  sont intégrés dans le temps sur chaque fenêtre ;
- chaque échange macro reste bidirectionnel et ferme exactement les bilans de
  masse d’espèces et d’énergie ;
- le débit de Riemann instantané est néanmoins observé à chaque sous-pas
  mécanique pour ne pas décimer l’excitation audio.

Ce n’est pas un saut de trame ni un cache de waveform. C’est un couplage
partitionné de deux bandes dont les domaines de validité sont explicites.

## 2. Contrat simulation → audio

Pour chaque cylindre, `CylinderPressureSample` publie :

| Grandeur | Unité | Convention |
|---|---:|---|
| pression chambre | bar | absolue |
| pression au runner | kPa | absolue |
| débit massique | kg/s | positif cylindre → réseau, négatif en réversion |
| masse volumique | kg/m³ | état local réseau |
| célérité | m/s | état local réseau |
| conductance de soupape | m² | aire géométrique × coefficient de décharge |
| indice de chemin | — | route d’échappement compilée |
| validité thermoacoustique | booléen | toutes les grandeurs ci-dessus sont physiques |

Le runtime publie une trame par sous-pas mécanique lorsque l’audio est actif.
Les files SPSC sont bornées et le callback n’alloue pas.

## 3. Réseau caractéristique audible

Le renderer retire une moyenne lente de la pression et du débit, puis construit
les caractéristiques planes à partir de la frontière mesurée :

```text
Zc = ρ c / A
U′ = ṁ′ / ρ
p+ = 1/2 (p′ + Zc U′)
p− = 1/2 (p′ − Zc U′)
```

La réflexion au port n’est pas déduite d’un preset d’ouverture. Elle vient de la
linéarisation locale de la loi d’orifice autour du débit moyen. Les runners et
collecteurs sont des guides bidirectionnels ; les jonctions N-ports diffusent
les ondes selon les admittances `A/(ρc)`. Firing order, longueurs, sections et
température déterminent donc naturellement la phase, le croisement entre
cylindres et les résonances.

La haute bande caractéristique est linéaire et passive. Elle propage le signal
audio mais ses réflexions haute fréquence ne sont pas réinjectées dans la
chambre 0D ; le réseau non linéaire basse bande reste propriétaire de la
contre-pression physique.

## 4. Rayonnement et calibration

La sortie est terminée par `UnflangedPipeRadiation`, approximation causale de
Padé (1,2) de la solution de Levine–Schwinger ajustée par Silva et al. Le filtre
retourne la pression réfléchie dans le guide. La vitesse de volume nette à la
bouche puis son accélération donnent la pression monopolaire en champ libre.

Le renderer applique le retard acoustique air `r/c` jusqu’à un observateur à
1 m. La conversion numérique est explicite : 20 Pa RMS, soit 28,284 Pa crête,
correspondent à 0 dBFS (120 dB SPL pour la pression de référence 20 µPa). Il
n’existe pas de gain caché de « réalisme » sur le bus d’échappement physique.

Références :

- [H. Levine et J. Schwinger, *On the Radiation of Sound from an Unflanged
  Circular Pipe*](https://doi.org/10.1103/PhysRev.73.383), Physical Review 73
  (1948), 383–406 ;
- [F. Silva et al., *Approximation formulae for the acoustic radiation impedance
  of a cylindrical pipe*](https://doi.org/10.1016/j.jsv.2008.11.008), Journal of
  Sound and Vibration 322 (2009), 255–263.

## 5. Réponses impulsionnelles

Le champ libre est le défaut. L’application ne charge plus d’IR de preset,
d’IR générique ni d’IR synthétisée depuis la géométrie. Une convolution est
active uniquement si `exhaust_paths[].impulse_response` désigne explicitement
un WAV. Cette IR doit représenter une mesure aval — cabine, pièce, microphone
ou système complet identifié — et non remplacer une dynamique de gaz absente.

## 6. Ce qui a été volontairement retiré du chemin physique

Une fois la frontière SI active, l’échappement n’utilise plus :

- oscillateurs de blowdown ou de « crack » ;
- bruit aléatoire de jet ;
- jitter de débit ;
- FDN de silencieux ;
- coloration de preset, saturation de collecteur ou gain de transmission audio
  du DAG ;
- IR implicite ou générée.

Le code historique reste isolé pour certains harnais de compatibilité sans
frontière SI. Il n’est pas mélangé à la sortie livrée par `EngineRuntime`.

## 7. Compatibilité physique — réponse honnête

La physique précédente n’était pas suffisante pour ce saut qualitatif. Il a
fallu ajouter le réseau quasi-1D conservatif, les réservoirs cylindres finis, les
flux bidirectionnels aux soupapes, les jonctions globales, le débit signé et les
états SI locaux. La combustion 0D existante était structurellement compatible :
elle fournit déjà pression, énergie, composition et volume à la frontière.

Elle n’est cependant pas une validation absolue. Pour corréler un moteur réel,
il reste nécessaire de comparer pression cylindre, pression de runner, débit et
température à des mesures, puis d’améliorer au besoin combustion, transferts
thermiques, coefficients de soupape et géométrie.

Le son global n’est pas encore entièrement physique : admission, bloc,
distribution, démarreur et suralimentation conservent des couches procédurales.
Retirer honnêtement la mécanique synthétique exige un modèle modal réduit du
bloc/culasse/carters, excité par les forces gazeuses, inerties, réactions de
paliers et impacts de distribution. Revoicer quelques harmoniques ne remplirait
pas cette lacune.

Autres limites explicites : acoustique plane linéaire dans la haute bande,
sortie circulaire non bridée, correction d’écoulement moyen au rayonnement non
modélisée, positions 3D des sorties non publiées, modes transverses et acoustique
de coudes non résolus.

## 8. Carte du code pour les prochains agents

| Responsabilité | Fichiers principaux |
|---|---|
| volumes finis et thermodynamique | `src/gas-dynamics/*/FiniteVolumeDuct.*` |
| compilation du DAG | `src/gas-dynamics/*/ExhaustNetworkLayout.*` |
| couplage global/jonctions/soupapes | `src/gas-dynamics/*/ExhaustGasNetwork.*` |
| orchestration multirate et télémétrie | `src/simulation/src/EngineSimulator.cpp` |
| radiation passive | `src/audio/*/PipeRadiationModel.*` |
| caractéristiques et rendu | `src/audio/*/RealtimeEngineAudio.*` |
| chargement d’IR explicite | `src/app/src/MainComponent.cpp` |

Ne pas réintroduire un fallback silencieux si le réseau échoue. Une erreur de
configuration doit être observable ; une limite de résolution doit alimenter
`solverResolutionLimited`.

## 9. Validation obligatoire

Les tests couvrent notamment : état uniforme, tube à choc de Sod, positivité,
conservation espèce/énergie, volume unique, propagation, interfaces directes,
ordre des frontières, soufflage/réversion, rayonnement passif, déterminisme,
invariance aux presets/bruits/gains hérités, géométrie et invariance 48/96 kHz.

Avant livraison :

```powershell
cmake --build out/build/windows-vs2022 --config Release --parallel 4
ctest --test-dir out/build/windows-vs2022 -C Release --output-on-failure
```

Le harnais LS3 doit aussi rester sous les 4,167 ms de la boucle 240 Hz. La
mesure de référence de cette implémentation donne environ 2,9–3,1 ms de moyenne
à 3630 tr/min et 3,84–3,92 ms à haut régime, avec un p95 maximal observé de
4,139 ms sur trois passages.
