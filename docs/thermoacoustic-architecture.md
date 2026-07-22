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

### Limite honnête qui subsiste

L'EJ25 garde un plancher HF élevé en montée (20.9 % au-dessus de 4 kHz contre
7.1 % sans chambre) : ce plancher **préexiste** à la chambre, celle-ci le
démasque en retirant du grave. C'est la prochaine chose à regarder sur ce
moteur, et c'est un problème de couche turbo, pas de silencieux.
