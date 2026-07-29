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
charge passive de rayonnement de chaque sortie
        → pression libre vers deux microphones + retard r/c + directivité
        → IR mesurée explicitement fournie, facultative
```

Il n’existe aucun repli runner/collecteur 0D dans `EngineSimulator`. Une
topologie physique invalide empêche la construction du simulateur. Dès qu’une
topologie SI valide est compilée, le chemin physique possède la sortie dès le
premier échantillon : il propage le silence jusqu’à la première frontière, puis
reste verrouillé. Les voix procédurales ne peuvent donc ni apparaître pendant
le démarrage, ni revenir lors d’une perte ultérieure de télémétrie.

## 1. Réseau gaz non linéaire

`EngineLabGasDynamics` transporte, par volume fini :

- la masse de chaque espèce (`O₂`, inerte, carburant, gaz brûlés) ;
- la quantité de mouvement axiale ;
- l’énergie totale.

Le noyau utilise un flux HLLC avec repli de sûreté HLLE, une reconstruction TVD,
SSP-RK2 et un pas CFL. Une tentative non physique est rejetée puis reprise avec
un pas réduit ; aucune masse ni énergie n’est créée par un plancher numérique.
Les pertes locales, le frottement de paroi et le transfert thermique sont des
termes sources déclarés. Les parois du réseau moteur possèdent désormais une
capacité thermique finie : l’échange gaz-métal conserve l’énergie combinée et
seule la convection externe rejette explicitement la chaleur vers l’ambiance.

Une soupape n’est pas traitée comme la continuation sans épaisseur d’un tube.
Son `CdA` alimente une loi de tuyère compressible isentropique, subcritique ou
étranglée selon le rapport de pression. La composition et l’enthalpie totale de
l’amont sont transportées dans les deux sens. Le débit sonique fait l’objet d’un
test analytique indépendant ; cette correction était nécessaire, car un flux de
tube HLLC sous-estimait fortement le soufflage d’un réservoir cylindre.

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
  avec une fenêtre absolue maximale de 250 µs à bas régime ;
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

Le débit de soupape possède désormais deux propriétaires spectraux explicites.
La paire pression/débit issue du réseau passe dans un passe-bas
Linkwitz–Riley d’ordre 4 à `0,45 × fréquence de couplage`. Le débit de Riemann
instantané passe dans le passe-haut complémentaire, calculé à la cadence
mécanique. Cette seconde branche n’est jamais associée à la pression plus lente :
elle devient une source de vitesse de volume au port, soit les caractéristiques
antisymétriques `(+Zc U/2, -Zc U/2)`, puis traverse la même impédance physique de
soupape que les ondes du runner. Les deux filtres ont une somme cohérente
all-pass ; il n’existe donc ni bande doublée, ni gain de timbre caché. Si la
frontière est déjà publiée pleine bande (`fréquence de couplage = 0`), la source
complémentaire est exactement nulle.

Cette branche instantanée reste elle-même un signal échantillonné par le solveur
mécanique. Deux sections Butterworth passe-bas bornent donc sa reconstruction à
`0,42 × cadence mécanique` (Linkwitz–Riley d’ordre 4). Sans cette borne haute,
les images de l’interpolation au-dessus du Nyquist mécanique étaient amplifiées
par la dérivée de rayonnement et produisaient des clics isolés — le grésillement
observé surtout sur le Merlin et le 2JZ. Ce filtre ne retire aucune fréquence
représentable par le producteur ; il interdit uniquement à l’audio d’inventer
une bande que la simulation n’a jamais échantillonnée.

## 4. Rayonnement et calibration

La sortie est terminée par `UnflangedPipeRadiation`, approximation causale de
Padé (1,2) de la solution de Levine–Schwinger ajustée par Silva et al. Le filtre
retourne la pression réfléchie dans le guide. La vitesse de volume nette à la
bouche puis son accélération donnent la pression monopolaire en champ libre.

Chaque sortie publie désormais sa position, son axe, son diamètre et son type de
terminaison (libre ou bridée). `FreeFieldObserver` calcule séparément les deux
distances sortie–microphone, les retards `r/c`, la décroissance `1/r` et la
directivité fréquentielle liée à `ka`. Une sortie centrée peut légitimement
rester presque mono ; deux sorties séparées acquièrent leur largeur par leurs
temps d’arrivée et non par un panoramique inventé.

Les pascals n’ont pas de correspondance universelle en dBFS : celle-ci
dépend nécessairement du microphone et du préamplificateur. La chaîne de capture
est donc un objet de calibration explicite (`AcousticMonitorCalibration`), avec
20 µPa comme pression SPL de référence et 144 dB SPL RMS à 0 dBFS par défaut.
Ce choix donne la marge d’un enregistrement moteur à fort niveau ; il est
modifiable indépendamment de la physique et du volume d’écoute. Il n’existe pas
de gain caché de « réalisme » sur le bus d’échappement physique, et le limiteur
de sécurité reste à gain unitaire dans les scénarios de validation.

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

Le débit de soupape corrigé a également révélé le retard d’un cycle de
l’injection indirecte pendant une remontée rapide de pression admission. Il ne
a pas été masqué par un enrichissement. `TransientChargeEstimator` conserve la
dernière masse d’air réellement piégée à la fermeture admission — donc le vrai
remplissage et les ondes du moteur — puis la projette par le seul rapport de
densité idéal-gaz `p/T` du plénum. L’oxygène déjà résolu dans la chambre reste une
borne inférieure et la boucle fermée conserve son rôle de correction.

Elle n’est cependant pas une validation absolue. Pour corréler un moteur réel,
il reste nécessaire de comparer pression cylindre, pression de runner, débit et
température à des mesures, puis d’améliorer au besoin combustion, transferts
thermiques, coefficients de soupape et géométrie.

L’échappement, l’admission et le rayonnement du bloc/culasses sont désormais des
chemins physiques pilotés par le solveur. La suralimentation est solver-driven
mais reste semi-empirique sur son rendement acoustique ; distribution,
démarreur et transmission ne disposent pas encore tous d’un modèle rayonnant
identifié. Ces limites ne sont remplacées par aucun oscillateur dans un chemin
physique déjà disponible.

Autres limites explicites : acoustique plane linéaire dans la haute bande,
correction d’écoulement moyen au rayonnement non modélisée, modes transverses et
acoustique de coudes non résolus, modes structurels estimés tant qu’aucune mesure
NVH n’est fournie.

## 8. Carte du code pour les prochains agents

| Responsabilité | Fichiers principaux |
|---|---|
| volumes finis et thermodynamique | `src/gas-dynamics/*/FiniteVolumeDuct.*` |
| compilation du DAG | `src/gas-dynamics/*/ExhaustNetworkLayout.*` |
| couplage global/jonctions/soupapes | `src/gas-dynamics/*/ExhaustGasNetwork.*` |
| orchestration multirate et télémétrie | `src/simulation/src/EngineSimulator.cpp` |
| anticipation physique de charge PFI | `src/simulation/*/TransientChargeEstimator.hpp` |
| radiation passive | `src/audio/*/PipeRadiationModel.*` |
| sorties et microphones stéréo | `src/audio/*/FreeFieldObserver.*` |
| réseau d’admission | `src/audio/*/AcousticIntakeNetwork.*` |
| rayonnement structurel | `src/audio/*/StructuralModalRadiator.*` |
| acoustique de suralimentation | `src/audio/*/ForcedInductionAcoustics.*` |
| calibration Pa → dBFS | `src/audio/*/AcousticMonitorCalibration.hpp` |
| raidissement de conduit (amplitude finie) | `src/audio/*/NonlinearDuctAcoustics.hpp` |
| caractéristiques et rendu | `src/audio/*/RealtimeEngineAudio.*` |
| chargement d’IR explicite | `src/app/src/MainComponent.cpp` |

Ne pas réintroduire un fallback silencieux si le réseau échoue. Une erreur de
configuration doit être observable ; une limite de résolution doit alimenter
`solverResolutionLimited`.

## 9. Validation obligatoire

Les tests couvrent notamment : état uniforme, tube à choc de Sod, positivité,
conservation espèce/énergie, volume unique, propagation, interfaces directes,
ordre des frontières, tuyère subcritique et sonique, soufflage/réversion,
projection de charge par densité, calibration SPL, rayonnement passif,
déterminisme, invariance aux presets/bruits/gains hérités, géométrie et
invariance 48/96 kHz.

Avant livraison :

```powershell
cmake --build out/build/windows-vs2022 --config Release --parallel 4
ctest --test-dir out/build/windows-vs2022 -C Release --output-on-failure
```

Le harnais LS3 doit aussi rester sous les 4,167 ms de la boucle 240 Hz.

## 10. État mesuré et limites connues

Cette section enregistre ce qui a été **mesuré**, y compris ce qui ne tient pas
le budget. Elle prime sur toute affirmation antérieure de ce document.

### Performance (`EngineLabPhysicsPerfHarness`, Release, 3 passages)

| Moteur | tr/min | moyenne | p50 | p95 | max |
|---|---|---|---|---|---|
| LS3 V8 | 3630 | 2,38–2,44 ms | 2,37–2,42 | 2,55–2,74 | 2,79–3,27 |
| LS3 V8 | 5940 | 3,22–3,32 ms | 3,19–3,25 | 3,48–3,75 | 3,83–**5,09** |
| Merlin V12 | 1760 | 3,95–4,02 ms | 3,93–4,01 | **4,17–4,26** | 4,40–4,63 |
| Merlin V12 | 2880 | **4,94–5,08 ms** | 5,01–5,15 | **5,26–5,39** | 5,50–6,05 |

Le LS3 tient le budget en moyenne mais son maximum atteint 5,09 ms, soit 22 %
au-dessus de la cadence. **Le Merlin V12 dépasse le budget sur la moyenne** à
2880 tr/min et son p95 dépasse déjà à 1760 tr/min : ce moteur ne soutient pas
le temps réel. Ne pas citer les seules moyennes LS3 comme preuve de conformité.

### Chemin audio

La voix de production est la somme de chemins physiques séparément mesurables :
réseau d’échappement complet, réseau d’admission, modes structurels et, lorsque
présente, suralimentation solver-driven. Les étapes correspondantes sont
détaillées aux §20–23. Les voix à oscillateurs historiques restent disponibles
uniquement pour les harnais de compatibilité dépourvus de configuration
physique ; le catalogue et `EngineRuntime` exigent zéro échantillon de ce chemin.

### Niveaux mesurés aux observateurs publiés

Le plein échelle du moniteur reste fixé à 134 dB SPL. Le rendu emploie la position
de microphone publiée par chaque moteur ; la garde de puissance extrapole cette
pression à un mètre par la même loi `1/r` avant d’appliquer 90–130 dB SPL. Elle ne
confond donc pas un observateur lointain avec une source faible. Le harnais impose
en plus que la somme des
couches physiques reste sous le genou du limiteur (`0,82`) avec gain de sécurité
strictement unitaire. Un changement de calibration ou un AGC ne peut donc pas
faire passer un moteur mal dimensionné.

## 11. Cadence de couplage : ce qui a été mesuré, et pourquoi elle n'a pas bougé

La frontière d'échappement qui excite le guide d'ondes est échantillonnée à la
cadence de couplage multirate. Tout contenu spectral au-dessus de la moitié de
cette cadence ne peut pas venir de la frontière : c'est une image de
reconstruction. `EngineState` publie désormais `exhaustCouplingFrequencyHz` et
`exhaustNetworkSubstepFrequencyHz` pour que la comparaison soit directe au lieu
d'être devinée, et le harnais de rendu les rapporte par moteur.

Deux corrections ont été distinguées, et une seule a été retenue.

**Retenue — cohérence de la frontière.** Pression, densité et vitesse du son
étaient reconstruites en mélangeant deux nœuds du réseau, tandis que le débit
massique venait du seul nœud le plus récent. La décomposition caractéristique
`0.5 (p' ± Zc U')` exige que les deux décrivent le même instant du même champ.
L'écart valait `(1 − phase) (to.p − from.p)` : une dent de scie cadencée au
couplage, dont les harmoniques dépassent largement le Nyquist de couplage. Elle
était injectée dans le terme source, donc aucune correction du guide d'ondes ne
pouvait l'atteindre. Corrigée en publiant deux débits aux contrats distincts.
Coût CPU nul.

**Écartée — relèvement de la cadence.** Acoustiquement, cela fonctionne. Mesuré
en couplant à chaque sous-pas mécanique, tous les pics repassent sous le Nyquist
de couplage pour la première fois (674–1583 Hz, donc des modes réellement
représentables), la fraction de haute bande de l'I4 tombe de 3.8 % à 0.5 % et
celle du V8 de 7.9 % à 0.8 %.

Le budget l'interdit. Le LS3 passe de ~3.76 ms à ~4.55 ms pour une trame de
4.167 ms à 3630 tr/min, soit de dedans à dehors ; à 5940 tr/min il va de 5.13 à
7.14 ms. Le surcoût n'est pas le sous-cyclage CFL — dont le nombre total de
sous-pas dépend du temps physique parcouru, pas du nombre d'appels — mais le
travail fixe par couplage : moyennage de frontière sur tous les cylindres et
mise en place de l'avance. À stride 1 ce coût fixe se paie à chaque sous-pas.

Une borne absolue sur la cadence a aussi été essayée, pour épargner les moteurs
à beaucoup de cylindres. Elle **dégrade** : le V8 remonte à 36.1 dB @ 5754 Hz,
une image de premier rang sur son propre couplage. Le contenu de la bouffée de
détente suit bien la cadence d'allumage, donc la règle par période d'allumage
est physiquement fondée et une borne en Hz absolus la casse. Ne pas réessayer
sans traiter d'abord le coût fixe par couplage.

Le chemin praticable est donc de réduire ce coût fixe, pas de relever la cadence
telle quelle.

### Limite honnête qui subsiste

Les pics étroits ne sont pas éliminés. Après la correction de cohérence, le
harnais mesure encore 16–32 dB au-dessus du plancher local, pire cas V8 à
31.9 dB @ 3833 Hz, et plusieurs restent au-dessus du Nyquist de couplage : il
demeure de l'imagerie que cette correction n'explique pas. La bande réellement
physique reste bornée à ~1.1–2 kHz par le couplage. C'est une limite
structurelle, pas un réglage.

Aucun test de non-régression ne garde cette correction. Le seul invariant propre
envisagé — « pas d'énergie au-dessus du Nyquist de couplage » — est faux en
toute rigueur, la terminaison de soupape étant non linéaire et créant
légitimement des harmoniques. Un seuil inventé aurait été pire que rien.

## 12. Le peigne de trame : la vraie origine du « métallique » (résolu)

La section 11 laissait des pics « au-dessus du Nyquist de couplage » inexpliqués.
Leur origine est désormais établie et corrigée. Les pics dominants de tout le
catalogue tombaient sur des multiples exacts de 240.07 Hz — la cadence de trame
— identiques d'un moteur à l'autre (4081 Hz sur le V8 **et** l'I2), avec des
bandes latérales au taux d'allumage de chaque moteur. Le repliage synchrone à
200 échantillons (fold) a mesuré la composante verrouillée trame à ~−32 dB du
RMS total.

Attribution par élimination, chaque étape mesurée : couches synthétiques
coupées (`--mute-combustion/-mechanical/-intake`) → persiste ; IR remplacée par
un Dirac → persiste ; refits par bloc gelés → persiste ; dyno du harnais lissé
à 2 Hz → persiste ; sous-blocs de rendu de 100 (`--audio-chunk`) → le peigne ne
suit pas la taille de bloc. Conclusion forcée : le peigne entre par la
télémétrie — la **solution du réseau elle-même** était modulée à la trame.

Cause : la vidange du réseau était forcée au dernier sous-pas de chaque trame.
Quand le nombre de sous-pas n'est pas multiple du stride (V8 : 49 sous-pas,
stride 2), la dernière fenêtre de moyennage est tronquée — le même motif
irrégulier répété à 240 Hz. Quatre corrections, par ordre d'élimination :
filtre de reconstruction anti-imagerie suivant la cadence publiée ;
reconstruction à délai constant (625 µs) sur anneau de nœuds horodatés ;
rampes à 10 Hz sur les délais pilotés par télémétrie de trame ; et la décisive,
une **grille d'intégration réseau libre** (accumulateurs membres, vidange par
durée accumulée, plus jamais par fin de trame).

Résultat en configuration d'usine : plus aucun pic dominant sur le peigne.
I4 16.5 dB @ 2241 Hz (mode réel), V8 34.6 dB @ 983 Hz (4e harmonique
d'allumage — contenu moteur), I2 12.1 dB @ 678 Hz, Radial 17.7 dB @ 523 Hz.
Haute bande 0.2–0.6 % (contre 3–15 % en début de chantier). La suppression de
la fenêtre dégénérée rend en outre ~20 % de CPU : LS3 3.73 → 2.89 ms à
3630 tr/min, 5.13 → 4.15 ms à 5940 — de retour dans le budget.

Résidu documenté : le radial garde une raie faible à 32×240 Hz, sa grille de
couplage étant réellement commensurable avec la trame (8 vidanges par trame
exactement) ; énergie haute bande 0.0 %. La composante verrouillée trame
restante est concentrée à 240/480 Hz — la réponse authentique du moteur aux
commandes par trame — et non plus dans l'aigu.

## 13. « Étouffé » et « tous pareils » : ce qui a été mesuré, et la distinction qui compte

Deux plaintes d'écoute distinctes ont été confondues au départ, et il faut les
garder séparées parce que **leurs causes n'ont rien à voir** :

- **« étouffé / muffled »** — le haut du spectre manque. Cause réelle : la bande
  physique était bornée par le couplage.
- **« tous les moteurs sonnent pareil »** — le caractère par moteur ne ressort
  pas. Ce n'est **pas** un problème de bande passante.

### La mesure qui a tranché « tous pareils »

La corrélation de forme spectrale (log-spectre, 80–6000 Hz) entre moteurs vaut
0.55–0.74. Le réflexe est de conclure « ils se ressemblent trop ». C'est faux, et
la mesure qui le prouve est le **spread par tiers d'octave** : dans chaque bande,
l'écart entre le moteur le plus fort et le plus faible est de **15 à 30 dB**
(`scratchpad overlay.py`). Les moteurs diffèrent énormément bande par bande. La
corrélation de 0.55–0.74 ne capture que la **tendance commune** — tout roule vers
l'aigu — qui est physiquement universelle et correcte. Il n'y a **aucune**
résonance commune parasite (la seule bande à faible spread, 160 Hz, est à 14 dB ;
tout le reste ≥ 17 dB). Donc : pas de bug d'homogénéisation à corriger.

Piège à éviter pour le prochain agent : **la corrélation de forme spectrale
monte quand tous les moteurs deviennent plus brillants « de la même façon »**.
Élargir la bande (§14) et ajouter le raidissement (§15) font *monter* cette
corrélation de ~0.02–0.03 chacun (attribution isolée par A/B, `meancorr.py`),
alors même qu'ils *améliorent* le son. C'est un artefact de métrique, pas une
régression : la métrique pénalise « tout le monde a gagné de l'aigu » même quand
cet aigu est souhaitable. Ne pas « corriger » cette hausse.

### Ce que la différenciation est réellement

Le caractère par moteur vit surtout dans le **motif d'allumage** (l'enveloppe),
pas dans la forme spectrale stationnaire. Deux sondes le confirment :

- **Spectre de modulation d'enveveloppe au ralenti** (`character.py`) : chaque
  moteur a une signature de pics distincte (K20 62.5 Hz, LS3 196 Hz, Hayabusa
  82 Hz, Big Twin 20.5 Hz…). Ces « rythmes » d'échappement sont bien distincts.
- **Burble du V8 crossplane** (`burble.py`) : l'énergie de modulation
  sous-allumage rapportée à l'allumage vaut **0.09 pour le LS3** contre **0.00**
  pour les I4 à allumage régulier — le grondement « rageur » issu de l'allumage
  inégal par banc (intervalles 180/270/180/90° dans chaque banc) est présent.

Conclusion mesurée : les moteurs **sont** différenciés ; l'étouffement masquait
cette différence. Retirer l'étouffement (§14, §15) rend la différence audible
sans avoir à « forcer » une différenciation artificielle — ce qui aurait été un
hack.

## 14. Doubler la bande physique à bas régime (cap de couplage 500 → 250 µs)

`EngineSimulator::step` borne l'intervalle de couplage par
`maximumLowSpeedCouplingSeconds`. Ce cap ne mord **que là où la règle par période
d'allumage est plus lente que lui** : au ralenti, à bas régime, et sur toute la
plage des moteurs à peu de cylindres. C'est exactement le régime où l'auditeur
entendait « étouffé », parce que le Nyquist de couplage y valait ~1 kHz et que la
reconstruction anti-imagerie lissait chaque front de détente au même endroit pour
tous les moteurs. À 250 µs le Nyquist passe à ~2 kHz.

**Pourquoi ceci ne rouvre pas le mur de budget de la §11.** La §11 a écarté le
*stride 1* (coupler à chaque sous-pas mécanique), qui paie le coût fixe par
couplage à chaque sous-pas et fait exploser le LS3 à haut régime. Le cap 250 µs
est différent : au régime maxi, la règle par période d'allumage donne déjà un
intervalle plus court que 250 µs (LS3 à 5940 tr/min : ~158 µs), donc **le cap ne
mord pas là où le budget est tendu**. Mesuré : à 5940 tr/min le nombre de sous-pas
de couplage est identique à 250 et 500 µs (`perf-*.csv`, colonne substeps
inchangée) — le travail physique à haut régime est le même. Le cap n'ajoute des
vidanges qu'au ralenti/bas régime, où il reste ~3–5 % du budget de trame. Un point
de mesure explicite à `idle_rpm × 1.1` a été ajouté à `PhysicsPerfHarness` pour
que cette région, où le cap agit, soit suivie indépendamment des points de charge.

## 15. Raidissement de front : la propagation à amplitude finie dans les conduits

`src/audio/include/enginelab/audio/NonlinearDuctAcoustics.hpp` (nouveau) ajoute la
seule non-linéarité de propagation que le chemin physique n'avait pas. Les niveaux
en conduit derrière une bouffée de détente sont de l'ordre du kilopascal
(150–175 dB SPL) : la vitesse locale d'une onde simple y dépend de l'amplitude
(`dx/dt = c + β·u`, `β = (γ+1)/2`). Les crêtes rattrapent les creux, les fronts
se raidissent, et ce raidissement peuple les harmoniques au-dessus de la bande de
télémétrie — l'origine physique du « bark »/crackle d'un échappement libre, le
même mécanisme que le cuivrage des cuivres (Hirschberg 1996, Msallam 2000).

Implémentation : une **lecture à retard modulé par l'amplitude** sur les lignes de
guide d'ondes existantes (runners et collecteur). Un échantillon lu à un retard
nominal `D` est relu à `D · delayScale(p')`, où `delayScale = 1/(1 + β·p'/(ρc²))`.
Pur gauchissement temporel : ne crée pas d'énergie (passif par construction),
exactement transparent quand `p' → 0`, capture de choc implicite par
l'interpolation fractionnaire. Le milieu (`ρc²`) vient de l'état de gaz déjà suivi
par runner et par collecteur ; les runners hérités publient `ρc² = 0` et lisent
donc exactement au retard nominal (bit-identique à l'ancien chemin).

**Non-vacuité et référence littérature.** Le test `nonlinearDuctAcousticsRegression`
(dans `RealtimeRegressionTests.cpp`) vérifie ce que la théorie au **premier ordre**
garantit, pas la sortie du simulateur : la loi de croissance du 2e harmonique
`B₂/B₁ → σ/2` (terme dominant de la série de Fubini), une cascade harmonique
présente et décroissante, la croissance avec le niveau, la passivité, et un
contrôle linéaire (milieu désactivé → pas de distorsion). **Piège documenté** : le
schéma à sonde unique est d'ordre 1 ; il reproduit le 2e harmonique mais
sous-génère le 3e (~44 % de Fubini, mesuré `fubini_probe.cpp`). Ne pas resserrer le
test sur une correspondance Fubini exacte du 3e harmonique — ce schéma ne peut pas
la livrer, et un seuil calé sur la sortie mesurée violerait la règle « références
issues de la littérature ». Une correspondance exacte exigerait un solveur de
caractéristiques à capture de choc, hors sujet pour un effet de bande audio.

Effet mesuré (limiter, `character.py`) : le contenu 1.5–4 kHz bondit (Hayabusa
10.9 → 34.2 %, K20 2.8 → 37.8 %, EJ25 1.5 → 25.0 %) et le 4 kHz+ suit. Effet sur la
différenciation : neutre (§13). Ajouter la carte : `raidissement de conduit` →
`src/audio/*/NonlinearDuctAcoustics.hpp`.

## 16. Voix du Merlin V12 : stacks courts au lieu d'un collecteur long

`parts/exhausts.yaml`, preset `aircraft_manifold` : les primaires de 980 mm dans
un collecteur de 128 mm enterraient le crack de détente dans un boom grave et
coûtaient ~3× les mailles FV (poussant le V12 hors budget). Un vrai Merlin
échappe chaque cylindre par son propre stack d'éjection court (~150 mm) sans
silencieux. La géométrie corrigée (primaire 150 mm, restriction 0.02) est **plus**
fidèle, pas un maquillage. Mesuré : contenu 500–1500 Hz du Merlin ~triplé, punch
transitoire le plus élevé du catalogue (3.13). Le V12 reste le moteur le plus
sombre — c'est en partie physique pour un 19.8 L tournant à 3800 tr/min — mais il
crache désormais au lieu de bourdonner.

## 17. Le silencieux est acoustiquement inerte sur le chemin physique (mesuré — corrigé en §19)

Constat le plus important pour qui veut différencier les moteurs de route :
**`muffler_restriction` ne filtre rien dans le chemin physique livré.**

La branche physique de `RealtimeEngineAudio::render` (`sampleUsesPhysicalExhaust`,
`RealtimeEngineAudio.cpp` §« Collector -> outlet characteristic ») enchaîne
exactement : ligne aller → `DuctWallLoss` → raidissement → `PipeRadiationModel`
→ ligne retour → observateur → calibration → `continue`. Elle ne lit ni
`pathOpenness`, ni `collectorReflection`, ni le FDN. Le `continue` saute toute la
branche héritée, qui est la *seule* à consommer `openness` (lignes ~1188, ~1239,
~1267/1270) et la seule à appeler `processMufflerFdn`.

Conséquences, dans l'ordre d'importance :

1. **Aucun élément silencieux n'existe dans le guide d'ondes.** Le nœud
   `muffler` du DAG (`ExhaustGraph.cpp`) n'apporte qu'une *longueur* (450 mm) et
   une perte de charge pour la physique. Acoustiquement, chaque moteur est un
   tuyau droit terminé par une charge de rayonnement non baffée. Pas de chambre
   d'expansion, pas de discontinuité de section, pas de perte par transmission.
2. **Le FDN, lui, est indépendant de la géométrie.** `referenceFdnSamples`
   (1493/2111/2791/3557) est constant pour tous les moteurs ; seul
   `presetFdngain_` varie, et il vient de la table de voicing, pas du YAML.
   De toute façon il n'est pas atteint quand la télémétrie est présente.
3. **`openness` a une dynamique dérisoire même là où il agit.** Calculé sur les
   neuf presets de `parts/exhausts.yaml`, `1/sqrt(1 + 1.35*K)` vaut 0.80 à 0.87
   pour sept d'entre eux ; seuls `aircraft_manifold` (0.99) et `radial_collector`
   (0.95) s'en écartent. Ses usages sont des interpolations plates
   (`0.42 + 0.10*o`, `0.13 + 0.05*o`, `1.12 − 0.30*o`).

### La mesure qui tranche

`scratchpad/muffdiff.py` compare deux rendus `EngineLabAbClipRenderer` à graine
identique, seul `muffler_restriction` changeant : LS3 0.30 → 0.95 et K20
0.25 → 0.95, c'est-à-dire d'un échappement sport à un conduit quasi bouché.

| moteur | segment | écart max par tiers d'octave | écart global |
|---|---|---|---|
| K20 | ralenti / montée / limiter | ≤ 0.52 dB | +0.02 / +0.04 / +0.07 dB |
| LS3 | ralenti / montée / limiter | ≤ 1.39 dB | −0.16 / −0.00 / +0.16 dB |

Un vrai silencieux de série, c'est 20 à 30 dB de perte par transmission dans la
bande d'allumage. Le résidu mesuré ici est de la contre-pression qui remonte par
la physique, plus le jitter d'ordonnancement de §18 — pas de l'acoustique.

**Ce que cela explique.** Les moteurs jugés réussis à l'écoute (Merlin V12,
flat-6, radial) sont précisément ceux dont l'échappement réel *est* un tuyau
quasi ouvert : le modèle est fortuitement juste pour eux. Tous les moteurs de
route sont rendus comme des tuyaux droits, d'où la convergence vers un timbre
générique. Ce n'est donc **pas** une erreur de valeurs dans `parts/exhausts.yaml` :
c'est un composant absent du modèle. Corriger les chiffres du YAML ne peut rien
donner tant qu'un élément de perte par transmission n'existe pas dans le guide
d'ondes.

## 18. Budget temps réel de bout en bout : l'audio va bien, la physique déborde

Mesuré sur AMD Ryzen 7 8840U (8c/16t, portable récent), build Release.

**Rappel audio** (`EngineLabAudioRenderHarness`, bloc 256 à 48 kHz, budget
5333 µs, fil physique concurrent) — le callback est confortable :

| moteur | callback p95 | charge |
|---|---|---|
| K20 I4 | 804 µs | 15 % |
| 2JZ I6 | 877 µs | 16 % |
| LS3 V8 | 1543 µs | 29 % |
| Merlin V12 | 1755 µs | 33 % |

**La boucle physique, elle, rate ses échéances.** `EngineRuntime::run` tourne à
240 Hz, soit 4167 µs par pas. Le même harnais rapporte désormais les dépassements
du runtime (`physicsOverruns`) pendant que l'audio rend :

| moteur | pas en retard / 1440 | retard max |
|---|---|---|
| 2JZ I6 | 21 (1.5 %) | 0.7 ms |
| K20 I4 | 134 (9 %) | 1.2 ms |
| Merlin V12 (3100 tr/min) | 162 (11 %) | 1.9 ms |
| **LS3 V8 (6500 tr/min)** | **498 (35 %)** | **17.5 ms** |

`EngineLabPhysicsPerfHarness` au banc, pleine charge, médiane du p50 sur trois
passages (budget 4167 us/pas) :

| moteur | ralenti | mi-régime | haut régime |
|---|---|---|---|
| I2 | 7 % | 17 % | 25 % |
| V8 EL-50 | 24 % | 63 % | **96 %** |
| LS3 V8 | 26 % | 69 % | **99 %** |
| Merlin V12 | 49 % | 72 % | 77 % |

**Correctif de mesure, à retenir.** Une première version de ce tableau donnait le
V12 à 158-164 % du budget et le disait hors budget au ralenti. C'était un CSV
capturé **avant** le passage du Merlin aux stacks courts (§16), qui a divisé son
coût par deux comme le commit l'annonçait. Le V12 n'est pas le cas critique ; le
V8 à haut régime l'est. Ne pas comparer un CSV de perf à travers un changement de
géométrie d'échappement.

### Où va le temps (sonde de phase temporaire, part de la trame)

| moteur | cylindres | réseau FV | reste | sous-pas FV/trame |
|---|---|---|---|---|
| I2 | 53 % | 16 % | 31 % | 31.1 |
| LS3 V8 | 41 % | 29 % | 30 % | 31.5 |
| Merlin V12 | 20 % | **52 %** | 28 % | **60.7** |

Trois lectures :

1. **Le cas qui borde le budget (V8 à haut régime) est dominé par la physique
   par cylindre (41 %)**, déjà parallélisée. Le réseau FV n'y est que 29 %.
2. **Le V12 est l'inverse** : le réseau FV y prend la moitié de la trame, avec
   presque le double de sous-pas acceptés du V8 alors qu'il tourne deux fois
   moins vite. C'est le maillage des 12 runners plus le collecteur large.
3. **Un ~30 % de "reste" existe sur les trois moteurs**, indépendamment de leur
   taille : environ 800 us par trame sur le V8. C'est du travail hors cylindres
   et hors réseau (transmission, télémétrie, génération d'événements), et il
   n'est pas parallélisé. C'est la piste la moins explorée.

Rien n'est *perdu* (`droppedPressure=0`, `boundaryDropouts=0`) : le flux de
télémétrie arrive en retard et irrégulièrement, pas amputé. Mais c'est ce flux,
et lui seul, qui excite la chaîne d'échappement — le rendu du LS3 n'est
d'ailleurs pas reproductible d'un passage à l'autre (RMS 0.073 à 0.090 sur trois
rendus identiques), ce que ne montrent pas les moteurs qui tiennent leur cadence.

**À retenir pour la suite : le goulot est le fil physique, jamais le callback
audio.** Toute optimisation qui vise le rendu audio se trompe de cible ; c'est le
coût par sous-pas de `EngineSimulator::step` (et le nombre de sous-pas) qu'il
faut réduire. Le pool `SubstepParallel` est déjà correct — il ne spinne qu'entre
sous-pas d'une même trame et part sur variable de condition ensuite — donc le
levier n'est pas là non plus.


## 19. Le silencieux physique : chambre d'expansion (correction de §17)

`src/audio/include/enginelab/audio/ExpansionChamberMuffler.hpp`. Deux jonctions
de Kelly-Lochbaum et une ligne de retard bidirectionnelle sur le conduit
collecteur -> sortie. La perte par transmission est la forme fermée de Munjal,

    TL = 10 log10 [ 1 + 1/4 (m - 1/m)^2 sin^2(kL) ],   m = S_chambre / S_conduit

et `expansionChamberMufflerRegression` pilote l'élément par sinus pour vérifier
la loi, ses bandes passantes, sa passivité et sa transparence. Non vacuité
vérifiée : inverser un signe de jonction fait tomber le test.

**Garantie de transparence.** Sans chambre configurée, `process` est une
connexion traversante exacte. Vérifié dans le rendu livré, pas seulement dans le
test : le Merlin mesure **+0.00 dB dans chaque bande et chaque segment**.

### Deux corrections que seule la mesure a trouvées

- **L'absorption inventée était le vrai problème.** Une première version
  dissipait dans la chambre via un gain et un pôle pilotés par
  `muffler_restriction`. Cette correspondance était inventée (la restriction est
  un coefficient de perte de charge, pas d'absorption) et, **dans la boucle de
  réaction collecteur <-> sortie, une perte de 2 dB par traversée se compose et
  effondre la résonance basse fréquence** : l'EJ25 perdait 11 dB sur son
  fondamental en montée. La normalisation de loudness remontait alors le clip et
  démasquait le plancher HF préexistant du rendu -- 40 % de l'énergie au-dessus
  de 4 kHz. La chambre ne fabriquait pas de souffle : elle effaçait le moteur.
  L'élément est désormais purement réactif ; tout découle de deux aires et d'une
  longueur.
  **Piège pour la suite : ne pas remettre de perte large bande dans cette
  boucle sans mesurer le grave.**
- **Fausse piste, notée pour ne pas être refaite.** La première hypothèse était
  le retard appliqué en marche d'escalier au bloc (le piège documenté en §12).
  Le rampage a été ajouté -- c'est l'invariant que suivent déjà les retards de
  runner et de réflexion -- mais **mesuré, il ne change rien** sur ces clips. Il
  est conservé comme prévention, pas comme correctif.

### Géométrie et effet mesuré

Rapports d'expansion volontairement modestes (TL crête 1.6 à 7.0 dB, pas les
10-13 dB d'un gros corps d'origine) : c'est une chambre **unique**, et un vrai
silencieux multi-chambres est conçu pour que ses creux évitent les ordres
d'allumage. À profondeur d'origine, le modèle creusait un notch sur le
fondamental de l'EJ25 et divisait par deux le haut du spectre du K20 et du LS3.

Au limiteur, contenu 1.5-4 kHz sans -> avec chambre : Hayabusa 41.9 -> 43.2 %,
Big Twin 12.1 -> 13.1, K20 34.8 -> 28.3, LS3 11.2 -> 9.2. L'écart-type
inter-moteurs par tiers d'octave monte au ralenti (8.01 -> 8.24 dB) et au
limiteur (6.31 -> 6.48 dB). Coût CPU : nul (LS3 p95 29.8 % contre 28.9 % avant).

### Réaudit du plancher HF — 2026-07-29

Le soupçon ci-dessus a été rejoué sur le binaire Release actuel, et il n'est
plus reproductible. Sur une montée gouvernée en quatre points, l'EJ25 mesure
1,2 % d'énergie au-dessus de 4 kHz au point haut, et 0,4 % au point précédent ;
à mi-régime, le harnais court mesure 0,5 %. L'ancien chiffre de 20,9 % décrivait
donc un état antérieur du renderer et ne doit plus piloter une calibration.

L'audit de la couche turbo a néanmoins trouvé trois défauts structurels
indépendants du niveau :

- compresseur et turbine employaient la même suite de bruit, l'une négative de
  l'autre. Avec mêmes débit et géométrie, les deux sources s'annulaient
  exactement ;
- la puissance, la vitesse d'arbre et les débits arrivaient par marches de
  télémétrie à 240 Hz, ce qui produisait des discontinuités d'amplitude et leurs
  bandes latérales ;
- le débit total était rayonné une première fois par la turbine, puis à nouveau
  par la wastegate multipliée par son ouverture. Une wastegate ouverte dupliquait
  donc de la masse au lieu de partager le débit entre deux aires parallèles.

`ForcedInductionAcoustics` possède désormais quatre générateurs déterministes
indépendants, reconstruit la télémétrie par échantillon (5 ms ; dump valve
0,75 ms à l'attaque et 12 ms au relâchement), et partage le débit selon
`A_turbine / (A_turbine + ouverture*A_wastegate)`. La somme des deux branches
reste exactement le débit mesuré. La vitesse de jet turbine emploie la section
de passage `turbine_flow_area_mm2`; le diamètre d'exducer reste nécessaire pour
déclarer une source acoustique réelle.

Le test ajouté a d'abord échoué sur l'annulation exacte, puis passe avec les
tests de continuité et de conservation. Sur les clips complets normalisés, le
changement reste ciblé : corrélation avant/après 0,99821 sur le 2JZ et 0,99803
sur l'EJ25, différence RMS alignée 5,98 % et 6,28 %. Les moteurs sans
suralimentation sont bit-identiques et la part >4 kHz reste pratiquement
inchangée (2JZ 0,701 -> 0,703 %, EJ25 0,326 -> 0,327 %) : le correctif retire
des artefacts de source sans éclaircir artificiellement tout le moteur.

## 20. Réseau acoustique complet compilé depuis le DAG

`AcousticExhaustNetwork` compile désormais l'`ExhaustGraph` exact en réseau
d'ondes temps réel. Chaque conduit conserve sa longueur et sa section dans une
ligne bidirectionnelle avec pertes de paroi et propagation à amplitude finie ;
les merges et splitters utilisent une diffusion N-ports pondérée par les
admittances. Chaque sortie possède sa propre charge de rayonnement et son retard
jusqu'à l'observateur. Une topologie 4-vers-1-vers-2 reste donc six conduits et
deux sorties, au lieu d'être réduite à un runner moyen et un collecteur moyen.

La chambre d'expansion est elle aussi issue de sa géométrie publiée : son volume
et sa longueur donnent sa section interne. `muffler_restriction` reste une perte
de charge de l'écoulement moyen ; elle n'est volontairement pas transformée en
gain acoustique large bande sans loi physique d'absorption.

> **Correction (§25).** Cette section affirmait que « le diamètre de connexion
> conserve les deux discontinuités réelles ». C'était faux : `connectionAreaM2`
> est calculé par `ExhaustNetworkLayout` mais n'a jamais été lu par
> `AcousticExhaustNetwork`, qui ne voit que `flowAreaM2`. Les deux vraies
> discontinuités viennent maintenant du tronc de collecteur, réalisé depuis la
> longueur authorée de la jonction — voir §25.

Toute la mémoire des conduits, jonctions, sorties et retards est réservée dans
`prepare()`. `process()` n'alloue pas et le harnais de rendu refuse maintenant
un moteur de production si le DAG complet n'est pas actif. Le réseau réduit
historique ne subsiste que comme chemin de compatibilité pour les producteurs
et tests qui ne fournissent pas encore d'`ExhaustGraph`.

Limite assumée : les jonctions sont des diffuseurs acoustiques instantanés,
sans compliance concentrée propre. Les volumes finis qui ont une longueur
publiée deviennent bien des conduits ; modéliser ultérieurement un plénum
compact sans longueur exigera un élément de compliance dédié, pas un gain de
voicing.

## 21. Rayonnement modal du bloc et des culasses

`StructuralExcitationSample` publie à chaque sous-pas mécanique, par cylindre et
en unités SI, la force gazeuse sur le piston, la force d'inertie de l'ensemble
alternatif, leur réaction signée au palier, la poussée latérale et le couple de
réaction au vilebrequin. Ces grandeurs partagent exactement l'horodatage de la
pression cylindre ; le renderer les interpole donc sans reconstruire une force
depuis le régime ou le niveau audio.

`StructuralModalRadiator` fait évoluer 8 à 24 oscillateurs amortis selon

    q'' + 2*zeta*omega*q' + omega^2*q = F_modal / m_modal

avec une transition analytique exacte pour une force tenue sur un échantillon.
La pression à l’observateur publié vient ensuite de la puissance rayonnée par la
vitesse RMS de surface du mode, son aire et l'efficacité de rayonnement du
piston bafflé. Le passage vitesse d’antinœud → vitesse RMS emploie la forme
modale (poutre/torsion ou plaque), sans gain de calibration caché. Le
chemin de production ne contient plus le sinus de vilebrequin, le cliquetis
bruité ou le « piston slap » façonné qui tenaient auparavant lieu de structure.

Sans section `structural_nvh`, les fréquences sont explicitement marquées
`estimatedFamily` : bloc assimilé à une coque creuse, culasses à des plaques
minces, dimensions déduites de l'alésage, de la course, de la bielle et de la
famille d'implantation. Ce modèle est le meilleur compromis temps réel avec les
données disponibles, mais il ne doit pas être présenté comme une corrélation
NVH constructeur.

Le schéma 5 peut maintenant remplacer ce jeu par des modes `measured` ou
`calculatedGeometry`. Chaque mode porte fréquence, amortissement, masse modale,
aire et efficacité rayonnantes, facteur RMS de forme, type d'effort et
participation signée par cylindre. Une source traçable est obligatoire et il
n'existe aucun gain de voicing. Aucun moteur du catalogue livré ne revendique
encore une mesure : l'absence de données réelles reste visible au runtime.
Format et protocole :
[`structural-nvh-configuration.md`](structural-nvh-configuration.md).

Les tests imposent silence exact sans force, paramètres physiques finis,
amplification à la résonance calculée et décroissance de l'énergie avec un
amortissement positif. `EngineLab.StructuralNvh` ajoute le chargement catalogue,
les round-trips, l'excitation du mode mesuré et les gardes de provenance. Le
harnais catalogue impose aussi que ce chemin soit réellement actif dans le
câblage de l'application.

## 22. Admission complète et suralimentation

`CylinderPressureSample` transporte désormais le débit massique instantané
signé de chaque soupape d'admission, la pression, la masse volumique, la vitesse
du son et la section conductrice réellement employée par le solveur. Le débit
publié est l'intégrale des deux demi-pas symétriques divisée par la durée du
sous-pas : la source audio et le bilan de masse décrivent donc exactement le
même échange, sans reconstruire une impulsion depuis le régime moteur.

`AcousticIntakeNetwork` compile chaque chemin d'admission en runners
bidirectionnels individuels, compliance WDF de plénum, étranglement de papillon
à admittance variable, compliance optionnelle de boîte à air, conduit d'entrée
avec pertes thermovisqueuses, puis charge de rayonnement de pavillon/embouchure
et retard jusqu'à l'observateur. Les volumes, longueurs, diamètres et sections
viennent exclusivement de `EngineConfig`. Une valeur nulle de boîte à air ou de
conduit signifie que la pièce est absente ; elle ne déclenche aucune géométrie
inventée. Le réseau ne transporte que la perturbation acoustique : une moyenne
glissante à 5 Hz retire le débit conservé, si bien qu'un débit parfaitement
stationnaire depuis `reset()` produit un silence exact.

`ForcedInductionAcoustics` emploie les ordres de passage écrits dans la
configuration : nombre de pales du compresseur et de la turbine, nombre de
lobes d'un compresseur volumétrique, vitesse d'arbre et rapport d'entraînement.
La pression des raies est issue de la puissance d'arbre résolue et d'un
rendement acoustique explicite. Les composantes larges bandes du compresseur,
de la turbine, de la wastegate et de la dump valve suivent une loi de jet
compact en U^8, centrée par un Strouhal de 0,2, avec débit corrigé, débit
d'échappement et sections physiques.

Les quatre sources broadband disposent de suites de bruit déterministes mais
indépendantes. Le débit d'échappement est partagé entre turbine et wastegate
proportionnellement à leurs aires effectives ; il n'est jamais compté deux fois.
Les grandeurs de télémétrie sont lissées à cadence audio afin que les mises à
jour du thread physique ne deviennent pas une modulation à 240 Hz.

La dump valve n'est plus une enveloppe déclenchée par une fermeture de pédale.
Le solveur l'ouvre lorsque le rapport de pression entre le réservoir de sortie
compresseur et le collecteur dépasse le seuil configuré ; son débit est calculé
par une loi d'orifice compressible, sous-critique ou étranglée. Seul ce débit
peut exciter son rayonnement. Le réservoir amont reste toutefois le réservoir
zéro-dimensionnel du modèle de boost : il n'existe pas encore de conduit de
suralimentation discrétisé ni de CFD de roue.

La fréquence des ordres de pales/lobes et les puissances thermodynamiques sont
calculées. Les nombres de pales, diamètres et coefficients ajoutés aux moteurs
« like » du catalogue sont explicitement des estimations de famille. Les
niveaux absolus de suralimentation restent donc semi-empiriques ; une carte
compresseur et des mesures acoustiques propres au turbo pourraient remplacer
ces paramètres sans modifier l'architecture. Les tests imposent fréquence de
passage exacte, pression proportionnelle à la racine de la puissance, silence
sans puissance/débit, et absence exacte de wastegate ou dump valve lorsque son
débit physique est nul.

## 23. Sorties physiques, directivité et observateur stéréo

Le schéma moteur 4 ajoute, en unités SI, un couple de microphones, la célérité
locale facultative, ainsi que pour chaque sortie sa position, son axe, son
diamètre et sa terminaison libre/bridée. YAML, JSON, catalogue, script et graphe
compilé transportent ces valeurs sans conversion implicite. Les documents plus
anciens migrent vers une géométrie de champ libre documentée.

`FreeFieldObserver` est un opérateur causal par sortie et par canal. Il réserve
ses lignes de retard dans `prepare()`, puis applique :

- la distance géométrique exacte et le temps d’arrivée `r/c` ;
- la décroissance sphérique `1/r` ;
- une directivité dépendante de l’angle et de `ka`, séparée en bandes basse et
  haute autour de `ka = 1` ;
- le comportement avant/arrière propre à une terminaison libre ou bridée.

L’échappement et l’admission renvoient donc directement une pression stéréo en
pascals. Structure et suralimentation emploient la distance moyenne du même
couple de microphones tant que leur configuration ne publie pas encore une
position de source complète. Une IR de pièce ou de cabine reste un élément aval
explicitement mesuré ; le champ libre est toujours le défaut.

## 24. Validation catalogue et suppression des anciens chemins

La validation Release rend désormais **chaque moteur du catalogue**, et pas
seulement quelques fixtures synthétiques. Pour chaque rendu, elle impose :

- activation réelle des réseaux échappement/admission et du rayonnement modal ;
- zéro événement, frontière ou échantillon de pression perdu ;
- zéro échantillon issu du chemin procédural de compatibilité ;
- valeurs finies, absence de plateau d’écrêtage et dynamique non impulsionnelle ;
- gain du limiteur de sécurité exactement unitaire et pic avant limiteur `< 0,82` ;
- niveau SPL plausible aux microphones publiés et équilibre spectral borné.

Les observateurs de pression par couche (`exhaust`, `intake`, `structure`) et le
pic pré-limiteur sont des mesures atomiques en lecture seule : ils n’agissent
jamais sur le rendu. Les tests analytiques couvrent en outre décroissance `1/r`,
directivité arrière bridée, rejet des images au-dessus du Nyquist mécanique,
passivité et conservation des éléments réseau.

Le chemin procédural n’a pas été supprimé aveuglément : il reste isolé pour une
API de compatibilité sans graphe physique. En production, la présence d’un
graphe compilé lui retire la propriété de la sortie dès le premier échantillon.
Cette séparation permet encore un diagnostic A/B explicite sans maintenir deux
voix concurrentes dans l’application livrée.

## 25. Audit échappement : la loi de paroi, le tronc de collecteur, le milieu par conduit

Audit du seul système d'échappement, puis correction. Cinq défauts, tous mesurés
avant et après. Ce sont des changements de voicing assumés.

### 25.1 `DuctWallLoss` n'appliquait pas la loi qu'elle implémente

La classe dérive l'atténuation de Kirchhoff-Rayleigh en `exp(-k*sqrt(f))` puis
l'approximait par **un simple pôle calé à 1 kHz**. Un pôle ne peut pas suivre
`sqrt(f)` : sa pente file vers 6 dB/octave alors que la cible est une inclinaison
très douce (0,2 à 2,5 dB sur toute la bande audio pour un conduit du catalogue).
Mesuré contre sa propre loi, la chaîne LS3 livrée sur-atténuait de **1,0 dB à
2 kHz, 4,2 dB à 4 kHz et 11,1 dB à 8 kHz**, par traversée simple, dans un réseau
bidirectionnel. Pire, le pôle ajusté tombait entre 0,31 et 0,67 pour *tous* les
moteurs : le haut de la bande était façonné par le coin du filtre, pas par la
géométrie — exactement ce qui aplatit les différences entre moteurs.

Remplacé par un **shelf un pôle / un zéro**. En normalisant le gain continu, le
module carré se réduit à une fonction de Möbius de `s = sin^2(w/2)` :

```
|H(w)|^2 = (1 + Z s) / (1 + P s),   Z = 4z/(1-z)^2,  P = 4p/(1-p)^2
```

Caler deux points est donc un système **linéaire 2x2**, et les paramètres
déformés s'inversent en forme close, `z = (sqrt(1+Z) - 1)^2 / Z`. Ni itération ni
recherche, et la passivité s'écrit `0 <= Z <= P`. Calé à 2,5 et 15 kHz, le shelf
suit la loi exacte à **0,35 dB** de 80 Hz à Nyquist, contre 11,9 dB au pire pour
le pôle seul. Au-delà d'environ 9,4 dB de perte par traversée au point bas,
aucun shelf passif du premier ordre ne joint les deux points ; `fit()` retombe
alors sur le pôle pur. Le conduit le plus long et le plus étroit du catalogue est
à 7 % de ce seuil.

### 25.2 Le mécanisme qui manquait : la coupure du mode plan

Corriger 25.1 a retiré un amortissement dont le réseau dépendait sans le dire :
K20 et Hayabusa ont attaqué le limiteur de sécurité. Le mécanisme manquant est
celui que `DuctWallLoss` citait déjà comme excuse pour sur-atténuer.

Au-dessus de `f_c = 1,8412 c / (2 pi a)` un conduit circulaire porte des modes
d'ordre supérieur, et chaque discontinuité y diffuse de l'énergie du mode plan —
où elle ne suit plus la ligne à retard. `DuctModeCutoff` applique un
**Butterworth d'ordre 4 à `f_c`, une fois par traversée**. Aucune profondeur
réglable : rayon et célérité sont les seules entrées. Ordre 4 parce que la
section vit dans la boucle collecteur-sortie où toute perte par traversée
s'accumule (elle est à 0,017 dB une octave sous la coupure, là où l'ordre 2
serait à 0,264 dB), et parce que l'apparition modale est réellement abrupte. Le
filtre est un état-variable TPT, stable pour tout `g > 0`, donc l'interpolation
par échantillon de la coupure est sûre par construction.

La bande est maintenant fixée par la géométrie, et le catalogue la balaye :
**2,1 à 2,9 kHz dans une chambre d'expansion contre 7 à 11 kHz dans un primaire**.

### 25.3 Le collecteur n'existait pas dans le guide d'ondes

`ExhaustNetworkLayout` routait tout `merge`/`splitter` vers une jonction sans
jamais lire son `lengthMm`, et `AcousticExhaustNetwork` traitait une jonction
comme un point sans étendue. Le compilateur historique authore pourtant un
collecteur de 120 mm au diamètre de collecteur pour **les dix moteurs du
catalogue**, et rien n'en arrivait à l'audio : quatre primaires diffusaient
directement dans le corps du silencieux. Mesuré sur le LS3, un primaire voyait
une réflexion de **-0,843 au lieu de -0,693**, et l'entrée de chambre un rapport
d'expansion de **2,19 au lieu de 3,49** — une perte par transmission de Munjal
de 2,4 dB là où la géométrie en décrit 5,5.

Une bifurcation est un point de diffusion **et** un tuyau. La longueur authorée
est publiée en `CompiledExhaustJunction::trunkLengthM` et réalisée comme conduit
du côté de la bifurcation qui porte exactement une connexion — le tuyau commun
d'un collecteur est en aval d'un merge, celui d'une sortie double en amont d'un
splitter. Avec plus d'une connexion des deux côtés la longueur n'est
attribuable à aucun côté et la jonction reste un point.

Le réseau volumes finis est **délibérément inchangé** : il modélise une jonction
comme un plénum bien mélangé et replie l'étendue dans `volumeM3`, ce qui est le
bon choix localisé aux fréquences qu'il résout. Un guide d'ondes ne le peut pas,
parce que la longueur y est un retard. Les deux discrétisations lisent
maintenant la même géométrie authorée.

### 25.4 Un seul milieu par chemin, pris au point le plus chaud

Le renderer prenait un `(rho, c)` par chemin, construit depuis l'état **au
port**, et l'appliquait à tous les conduits. Le solveur résolvait la température
par conduit depuis toujours : `outletSamples()` n'était lu que pour la
comptabilité de masse, et `ducts()` pas du tout.

Mesuré sur le K20 au-dessus de 3000 tr/min, état au port contre état par conduit :

| élément | c (m/s) | rho (kg/m3) |
|---|---:|---:|
| port (ce que tout conduit recevait) | 562,8 | 0,505 |
| primaires | 561,7 / 580,7 / 568,0 / 571,9 | |
| chambre | 554,0 | 0,444 |
| tuyau de sortie | 545,8 | 0,459 |

Cela **corrige l'estimation de l'audit lui-même**, qui supposait un gradient
bien plus raide. L'erreur de retard est d'environ 3 % par conduit et 6,4 % sur
la chaîne, pas 20-25 % — soit ~20 Hz sur le peigne d'une chambre de 400 mm, pas
165 Hz. L'erreur importante était ailleurs : dans l'impédance caractéristique
`rho*c`, qui fixe la diffusion aux jonctions — 284 au port contre 246 en chambre
et 250 en sortie, donc **12 à 14 % d'erreur sur tous les coefficients de
diffusion en aval**, cumulée au rapport d'expansion à l'entrée de chambre.

### 25.5 Reconstruction de frontière : ordre 8

Le filtre anti-imagerie était un Linkwitz-Riley d'ordre 4 à 0,45x la cadence de
couplage, ce que sa propre docstring décrivait comme laissant la première raie
d'image à seulement 28 dB — au-dessus du seuil « métallique » de 20 dB que le
harnais utilise lui-même. Passé à l'ordre 8 avec le coin porté à 0,47x :

| | 0,25x couplage | 0,5x couplage | 1,0x couplage |
|---|---:|---:|---:|
| LR4 à 0,45x | -0,79 dB | -8,0 dB | -28,1 dB |
| LR8 à 0,47x | -0,06 dB | -8,4 dB | -52,5 dB |

Meilleur des deux côtés à la fois : un filtre plus raide peut placer son coin
plus près du bord de bande. La somme des deux moitiés reste all-pass, donc
aucune des deux bandes physiques n'a eu besoin d'être réaccordée.

### 25.6 Ce qui n'est toujours pas résolu, et pourquoi ce n'est pas gaté

Le harnais mesure et **affiche** maintenant `outOfBandResonance`, la proéminence
d'une raie étroite au-dessus du Nyquist de couplage. Elle n'est délibérément
**pas** transformée en critère de succès.

L'argument tentant — « au-dessus du Nyquist de couplage la frontière ne porte
aucune information, donc rien ne peut y être un mode » — est vrai de la
*frontière* et faux du *réseau* : les modes du guide d'ondes ne s'arrêtent pas
là, et la source complémentaire de débit de soupape les excite. Mesuré sur le
quatre-cylindres de référence, couper cette source fait tomber le pic de 4107 Hz
de **31,2 à 22,1 dB** : ce n'est donc ni purement une image ni purement un mode.
Et ce moteur de référence, en géométrie par défaut sans chambre, est un
échappement droit ouvert — qui résonne réellement.

Passer la reconstruction à l'ordre 8 ne l'a pas déplacé, ce qui **écarte** le
chemin de reconstruction comme cause principale. Piste restante, non poursuivie
ici : le milieu au port, la conductance de soupape et la levée sont des flux
bloqués d'ordre zéro à la cadence de couplage qui **multiplient** la frontière au
lieu de s'y ajouter, et que le filtre de reconstruction ne voit donc jamais.

Ce qui est gatable sainement, c'est le filtre lui-même, et sa régression exige
maintenant 40 dB sur la première image et 80 dB sur la seconde (contre 24 et 40).

### 25.7 Correction : `ExpansionChamberMuffler` n'est pas du code mort

L'audit affirmait que `RealtimeEngineAudio.cpp:1431` (`if (useCompiledTopology)
continue;`) rendait cet élément inaccessible. C'est trop fort :
`useCompiledTopology = sampleUsesPhysicalExhaust && acousticExhaustNetwork_`, et
`sampleUsesPhysicalExhaust` est faux tant qu'aucun cylindre n'a de frontière
thermoacoustique valide. Le chemin réduit est donc un vrai repli, pas du code
mort. Il reste que le catalogue livré rapporte `legacySamples=0` partout : les
chiffres de §19 décrivent le repli, pas la voix livrée.

### 25.8 Mesures livrées

Base de comparaison : l'état avant cette passe.

- Résonance au ralenti du Big Twin **27,0 dB -> 15,7 dB**, et elle n'est plus une
  raie étroite à 439 Hz. Plancher de queue d'échappement **-70,0 -> -80,3 dB**.
- Tous les pics étroits situés au-dessus du Nyquist de couplage de leur moteur
  ont disparu : LS3 19,1 dB@4156 Hz -> 498 Hz, K20 18,3 dB@4128 -> 967 Hz,
  Flat-6 14,6 dB@4312 -> 492 Hz. C'étaient bien du contenu hors modèle.
- Différenciation en progrès sur quatre paires du catalogue sur six ; V8 contre
  radial 0,308 -> 0,228, I2 contre radial 0,679 -> 0,616.
- La brillance monte sur les moteurs à conduits étroits et baisse sur ceux à
  chambre large : c'est la géométrie qui travaille.

### 25.9 Tests ajoutés

Tous non vacués (vérifiés en cassant délibérément le code testé) :

- `ductWallLossRegression` borne désormais l'erreur d'ajustement **sur toute la
  bande** au lieu du seul point où l'ajustement est exact par construction, et
  vérifie l'équation aux différences contre le module analytique. Un simple pôle
  rate la nouvelle borne de 1,6 à 6,6 dB sur chaque conduit du catalogue.
- `ductModeCutoffRegression`, ancré sur la valeur de manuel : un conduit de
  50 mm dans l'air coupe à 4 kHz.
- `branchTrunkDelayRegression` mesure l'attaque causale à la sortie avec et sans
  tronc de 400 mm : **35 échantillons mesurés contre 35,9 prédits**.
- `ductMediumRegression` exige que donner à chaque conduit l'état froid
  reproduise exactement le réseau à qui l'on dit que tout le chemin est froid.
- `areaStepScatteringRegression` cale la diffusion d'un saut de section sur la
  forme close `T(m) = 4m/(1+m)^2` — la limite basse fréquence de la perte par
  transmission de Munjal — à 0,05 près pour m = 2, 4 et 9.

## 26. Source turbulente au débouché

Le réseau physique reconstruisait le blowdown et le rayonnement de conduit, mais
ne créait aucune source de mélange turbulent au contact du jet chaud et de
l’air extérieur. `ExhaustJetNoise` remplit uniquement ce rôle :

- débit moyen réparti entre les débouchés par aire ;
- vitesse `m_dot/(rho*A)` et puissance `K*rho*A*U^8/c^5` ;
- centre spectral à `St = 0,2` ;
- modulation causale par le débit volumique audio de la terminaison ;
- borne subsonique et borne `4,5 ×` sur l’excursion instantanée ;
- observateur stéréo, directivité, distance et IR identiques au débouché ;
- aucune rétroaction vers le solveur gaz.

Le multiplicateur moteur de puissance `100` est une valeur authored, isolée du
coefficient de jet propre `1e-4`. Il ne prétend pas être une constante physique
mesurée. Un contrôle nul coupe toute la couche pour les comparaisons sonores et
CPU. La validation complète, dont les calibrations refusées, est dans
[`audio-lot3-outlet-turbulence-2026-07-29.md`](audio-lot3-outlet-turbulence-2026-07-29.md).
