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
1 m. Les pascals n’ont pas de correspondance universelle en dBFS : celle-ci
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
| anticipation physique de charge PFI | `src/simulation/*/TransientChargeEstimator.hpp` |
| radiation passive | `src/audio/*/PipeRadiationModel.*` |
| calibration Pa → dBFS | `src/audio/*/AcousticMonitorCalibration.hpp` |
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

La voix par défaut est désormais **entièrement** le rayonnement physique
calculé. Ont été retirés du chemin physique : les voix à oscillateurs
(échappement *et* combustion) et le bus de pression chambre, qui ne comportait
aucune fonction de transfert vers l’observateur.

Limites réelles, à ne pas présenter comme résolues :

- **Bruit structurel absent.** Le rayonnement du bloc et de la culasse sous les
  forces de piston et de paliers n’est pas modélisé. Rien ne le remplace : le
  contenu de combustion audible provient uniquement de l’échappement.
- **Sortie monophonique.** Les positions des sorties d’échappement ne font pas
  partie de la géométrie publiée, donc tous les chemins rayonnent vers un
  unique observateur documenté à 1 m. Une vraie stéréo demande ces positions et
  un couple de microphones ; inventer un panoramique serait une décoration.
- **Résonances étroites non résolues.** Le harnais mesure encore des pics
  isolés de 18 à 39 dB au-dessus du plancher spectral local (pire cas : V8 à
  4081 Hz). Leur origine n’est pas entièrement expliquée. Les pertes de paroi
  et la terminaison à orifice les ont réduites sans les supprimer.
- **Admission non physique.** Aucun réseau d’admission 1D.

### Niveaux mesurés à l’observateur 1 m

107–117 dB SPL RMS selon le moteur en charge, pointes ~128 dB ; ralenti Big
Twin ~102 dB. Le plein échelle du moniteur est fixé à 134 dB SPL, dérivé de ces
mesures. Les seuils du harnais sont exprimés en SI (90–130 dB SPL en charge,
≥ 85 dB au ralenti) précisément pour qu’aucun réglage de gain ne puisse les
satisfaire à la place du modèle.

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
