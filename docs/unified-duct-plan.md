# Plan : conduit unifié

Écrit le 2026-08-03, après le rapport utilisateur « ÉCHAP PRO fait tomber le
temps réel quoi que je fasse », « l'afterfire ne fait rien » et « un échappement
court sans silencieux ne change presque rien », puis l'examen du prototype
ES3D v0.4.0a fourni par l'utilisateur.

Ce document est le plan de référence. Il survit au compactage de contexte.

> ## VERDICT DE LA PHASE 0 (2026-08-03) : LE DIAGNOSTIC EST FAUX, LE PLAN S'ARRÊTE
>
> La phase 0 était une porte : « elle doit reproduire le grief. Si elle montre
> un gros écart, le diagnostic est faux et le plan s'arrête ici. »
> **Elle a montré un gros écart. Les phases 1 à 5 ne doivent pas être
> réalisées telles qu'écrites.** Le détail est en fin de document, section
> « Ce que la phase 0 a réellement mesuré ». Le reste du document est conservé
> tel quel parce qu'un plan réfuté est une donnée : il dit quelle hypothèse a
> été éliminée et à quel prix.

## Le diagnostic, en une phrase

EngineLab résout la géométrie d'échappement dans un maillage volumes finis à
**300 mm** qui, de l'aveu de sa propre documentation, ne porte que **330-360 Hz**,
et fait porter la bande audio par un **second** modèle (réseau caractéristique,
FDN, voice layer, élément silencieux) qui n'est que *paramétré* par cette
géométrie. Changer l'échappement change donc surtout le modèle qui ne fait pas
le son. C'est la cause structurelle de « peu importe la géométrie, ça revient au
même », et la raison pour laquelle des mois de corrections sur l'échappement
n'ont rien changé d'audible : elles portaient sur le mauvais modèle.

## Ce que fait ES3D (mesuré en lisant ses scripts, pas en l'exécutant)

Un seul primitif, `tube`, utilisé **à l'identique** pour l'admission et
l'échappement :

```
tube(
    segment_length: 3.0 * units.cm,   // maillage UNIFORME, un seul nombre
    area: function()                   // section le long de l'abscisse
        .add_sample(0.0, 10.0 * units.cm2)
        .add_sample(0.8, muffler_area)  // le silencieux EST cette bosse
        .add_sample(1.0, 15.0 * units.cm2),
    path: path()...                    // polyligne 3D : longueur + rayonnement
)
```

- La seule différence entre son démo « avec silencieux » et « ligne droite » est
  `muffler_area` : **55 cm² contre 15 cm²**. Un nombre.
- `segment_length` porte le commentaire `// Do not set below 3 cm` : le coût CFL
  est un bouton assumé, pas une conséquence cachée de la géométrie.
- **Aucun collecteur** : zéro occurrence de merge/collector/junction dans tout le
  prototype. Ses topologies sont 1 cylindre → 1 tube, ou 4 cylindres → 4 tubes
  indépendants.
- **Aucun plénum**, aucune boîte à air. La trompette est la rampe 28 → 10 cm².
- Absent aussi : ECU, cartes AFR/avance, cliquetis, rupteur, coupure décel,
  régulateur de ralenti, turbo, film de paroi, NVH structurel, garnissage
  poreux, FDN, afterfire.
- Exposé à l'utilisateur : `courant_number` (0,5), `physics_frequency_divider`,
  `convolution_reverb` avec une IR de lieu réel (Koli, OpenAIR).

## Ce qui est décidé

**Garder** les jonctions. ES3D n'en a pas ; nous en avons besoin. Le burble d'un
V8 crossplane naît d'intervalles d'allumage inégaux se mélangeant dans un
collecteur partagé — quatre tubes indépendants ne peuvent pas le produire. Et
composer un 4-2-1 sur une banque et un 4-1 sur l'autre est la fonctionnalité
n°1 de l'utilisateur. Un raccord 1-D est de la mécanique des fluides standard
(GT-Power, Ricardo WAVE, AVL Boost, OpenWAM) et notre code de jonction marche.

Garder aussi : banc, dynamique véhicule, tuner ECU, catalogue. Rien de tout cela
ne coûte le son.

**Ce qu'il faut abandonner, ce ne sont pas les jonctions, ce sont les composants
typés.** Chaque composant (`pipe`, `merge`, `muffler`, `catalyst`, `outlet`)
invente aujourd'hui sa propre longueur et son propre maillage — c'est exactement
ce qui a produit la cellule de 5 mm corrigée en `a9ff0dd`. Avec une section
continue, un silencieux est une bosse, un mégaphone une rampe, un changement de
diamètre une marche, et **la jonction devient le seul objet topologique**. La
classe de bug disparaît par construction au lieu d'être corrigée.

## Phases

Chaque phase se termine par une mesure qui autorise ou interdit la suivante.

### Phase 0 — l'instrument qui manque, et l'afterfire

Rien ne mesure aujourd'hui **la sensibilité du son à la géométrie**, qui est
pourtant le grief central. Sans cet instrument, la réécriture ne serait pas
prouvable.

1. `EngineLabGeometrySensitivityHarness` : sur UN moteur, rendre N variantes
   d'échappement (court/long, avec/sans silencieux, petit/gros diamètre) par le
   vrai chemin temps réel, et publier l'écart par tiers d'octave entre variantes.
2. **Enregistrer la ligne de base avec l'architecture actuelle.** Elle doit
   reproduire le grief : peu d'écart. Si elle montre un gros écart, le
   diagnostic ci-dessus est faux et le plan s'arrête ici.
3. Afterfire : publier l'**état d'armement** dans le panneau diagnostics
   (`enabled`, fraction carburant, régime contre `overrunMinimumRpm`, papillon
   contre `overrunMaximumThrottle` = 0,02, température de paroi contre le seuil).
   Suspect actuel, non prouvé : le papillon commandé par l'app ne descend jamais
   sous 2 %. Ne pas « corriger » avant de voir l'état.

Sortie : une ligne de base chiffrée, et la raison réelle du silence de
l'afterfire.

### Phase 1 — le primitif unifié, en parallèle, sur un moteur

Un `Duct` : longueur de cellule uniforme autorée, section en fonction de
l'abscisse, chemin donnant longueur et position de rayonnement. Branché sur le
port et sur le rayonnement, sans réseau caractéristique.

Construit **à côté** de l'existant, derrière un drapeau, sur un seul moteur
(CP2 : deux cylindres, rendu en 5,9 s, la boucle la plus courte).

Portes :
- physique conservée : VE et couple dans la tolérance de l'oracle 30 mm/RK2 ;
- **sensibilité géométrique nettement supérieure à la ligne de base de la
  phase 0** — c'est la porte qui décide de tout ;
- facteur temps réel mesuré, protocole A/B du `CLAUDE.md` (témoin nul,
  contrebalancé, minimum sur N, jamais contre un chiffre écrit).

**Point de décision.** Si la sensibilité ne monte pas, on s'arrête et on n'a
brûlé qu'un moteur.

### Phase 2 — les jonctions dans le solveur unifié

Nœud à N branches avec volume, reprenant la physique de jonction existante.

Portes : un 4-1 et un 4-2-1 doivent **différer de façon mesurable** ; le burble
du V8 crossplane doit survivre (référence : 10× un I4 régulier).

### Phase 3 — bascule de l'échappement, et suppression des compensations

Retirer le réseau caractéristique, le FDN, la voice layer, l'élément silencieux
dédié. Ils existent tous pour compenser un maillage qui ne portait pas l'audio.

C'est **délibérément un changement de voicing**, donc la seule phase qui exige
une écoute humaine. Les 24 points constructeur et les 36 tests restent des
portes dures.

### Phase 4 — l'admission sur le même primitif, et ce qui finance tout

L'admission consomme **75 à 84 %** du pas de calcul, et cette complexité est
presque entièrement le **plénum partagé** (escalier de Gauss-Seidel, couplage
multirate, prédiction concurrente). ES3D n'a pas de plénum et sonne mieux.

Runners par cylindre sur le même primitif. Le plénum devient **une jonction avec
volume**, un nœud de plus dans le même solveur, et non un sous-système couplé
avec son propre schéma d'itération.

C'est cette phase qui paie le maillage fin d'échappement, et c'est elle qui
règle le V8 sous le temps réel.

Portes : les 24 références constructeur à ±15 %, les 16 ralentis, l'invariant
« un moteur à l'arrêt se stabilise à l'ambiant » (référence série : 0,002 kPa).

### Phase 5 — rendre le compromis à l'utilisateur

Exposer `courant_number` et un diviseur de fréquence physique, comme ES3D. Cela
laisse l'utilisateur arbitrer précision contre fluidité au lieu de subir 0,5×
sur un V8.

## Risques

- **La phase 3 change le son volontairement.** Aucune mesure ne peut la valider
  seule ; il faudra écouter.
- **Le maillage fin coûte.** 300 → 30 mm, c'est 10× de cellules d'échappement,
  soit ~1,6× du coût total avant la phase 4. L'ordre des phases fait que le
  surcoût existe entre la phase 1 et la phase 4 : ne pas livrer dans cet
  intervalle.
- **La phase 4 touche la calibration.** VE et couple sont réglés autour du
  modèle d'admission actuel. C'est la phase la plus susceptible de demander une
  re-calibration, et la raison pour laquelle elle vient après la preuve sonore.
- **Les ralentis sont des attracteurs marginaux** : un ULP fait basculer un
  moteur entre se stabiliser et caler. Attendre des casses de ralenti à chaque
  phase, ne pas chasser l'ULP, bisecter puis corriger la fragilité.

## Ce que la phase 0 a réellement mesuré (2026-08-03)

`EngineLabGeometrySensitivityHarness` rend le MÊME moteur par le vrai chemin
temps réel avec plusieurs échappements et publie l'écart par tiers d'octave.
« forme » est l'écart RMS par bande après retrait de l'offset large bande :
c'est le changement de TIMBRE, indépendant du niveau.

| moteur | forme mix | forme échap. seul | bandes masquées | AGC | limiteur |
|---|---|---|---|---|---|
| CP2 twin atmo | 8,20 dB | 12,52 dB | **0 / 28** | 1,000 | 0 |
| LS3 V8 atmo | 7,17 dB | 11,91 dB | **2 / 28** | 1,000 | 0 |
| 2JZ I6 turbo | 4,11 dB | 14,35 dB | **24 / 28** | 1,000 | 0 |

**Le maillage 300 mm porte la géométrie.** L'échappement répond de 11,9 à
14,4 dB de forme, avec des bandes individuelles à 18, 23 et 32 dB. L'hypothèse
« la géométrie est résolue dans un modèle qui ne fait pas le son » est réfutée :
le modèle qui fait le son EST celui qui résout la géométrie, et il l'entend
très bien. Une réécriture en conduit unifié n'aurait pas corrigé le grief.

Trois hypothèses ont été éliminées par la même mesure :

- **L'AGC lent et le limiteur ne retirent rien.** Gain minimal 1,000 et zéro
  échantillon limité sur les 21 rendus. Écartés, mesurés.
- **La couche combustion à −205 dB n'est pas une panne.** Elle est coupée
  volontairement sur le chemin physique (`RealtimeEngineAudio.cpp`, garde
  `if (!acousticExhaustNetwork_)`), avec le motif écrit dans le code : c'était
  du façonnage perceptuel présenté comme de la physique. Le contenu de
  structure passe désormais par `StructuralModalRadiator`.
- **Le masquage n'est PAS général.** Il est propre au 2JZ, où l'admission est
  à −10,1 dB de l'échappement et la mécanique à −8,4 dB, contre −28,9 et
  −22,6 dB sur le CP2. C'est un problème d'équilibre sur un moteur, pas une
  cause architecturale.

### La vraie cause : le silencieux ne silence pas

Mesure à un seul facteur — on retire UNIQUEMENT le corps du silencieux, rien
d'autre ne bouge. C'est une perte d'insertion, comparable à la littérature :

| moteur | chambre autorée | rendu | physique (Pa observateur) |
|---|---|---|---|
| CP2 | 84 × 250 mm | +2,10 dB | +1,25 dB |
| LS3 | 142 × 400 mm | +3,80 dB | +3,91 dB |
| 2JZ | 132 × 470 mm | −0,72 dB | **+0,06 dB** |

Littérature : **+20 à +30 dB**. Retirer entièrement le silencieux le plus gros
du catalogue change la pression à l'observateur de 0,06 dB. C'est exactement le
grief de l'utilisateur, chiffré : *« un échappement court sans silencieux, dans
la vraie vie ça fait un bordel pas possible, alors que là rien ne se passe. »*

Ce n'est pas une panne de l'élément — il a de l'autorité et suit la courbe de
Munjal. Sonde sur le 2JZ, en ne changeant que le diamètre de chambre :

| chambre | m = S_c/S_p | Pa observateur | écart |
|---|---|---|---|
| aucune | — | 20,3 | +0,06 dB |
| 132 mm (autorée) | 4,25 | 20,2 | référence |
| 198 mm | 9,6 | 10,7 | −5,5 dB |
| 304 mm | 22,5 | 8,8 | −7,2 dB |

**Une chambre d'expansion UNIQUE à un rapport automobile réaliste (m ≈ 4) ne
peut pas silencer** : sa perte de transmission de crête est
10·log₁₀[1 + ¼(m − 1/m)²] = 7,0 dB, et elle vaut exactement 0 dB à toutes ses
bandes passantes, espacées de c/(2L). Le modèle est juste ; c'est le composant
qui n'est pas celui d'une voiture.

Le garnissage absorbant Delany-Bazley a été mesuré aussi : il existe et
fonctionne, mais aucun moteur routier du catalogue ne l'autorise — seul le
preset de labo `cp2_absorptive_lab` le fait. Autorisé sur le 2JZ
(24 kPa·s/m², 35 mm, 28 % d'aire ouverte), il vaut **−1,7 dB**.

> **État historique, remplacé le 20 août 2026.** Les presets routiers portent
> maintenant des valeurs `estimated-family` et le corps brut n'est plus
> confondu avec le noyau de débit. Le volume annulaire possède une compliance
> passive bornée. Les mesures courantes et les limites sont consignées dans
> `docs/passive-muffler-implementation-2026-08-20.md`.

**Le tableau complet d'autorité du silencieux sur le 2JZ**, pression crête à
l'observateur, un seul facteur changé à chaque ligne :

| configuration | Pa | écart |
|---|---|---|
| aucun silencieux | 20,3 | +0,06 dB |
| chambre autorée 132 mm (m = 4,25) | 20,2 | référence |
| + garnissage absorbant | 16,5 | −1,7 dB |
| chambre 198 mm (m = 9,6) | 10,7 | −5,5 dB |
| chambre 304 mm (m = 22,5) | 8,8 | −7,2 dB |

**La perte d'insertion in situ sature vers 7 dB** quelle que soit la force
appliquée à l'élément réactif, alors que sa perte de transmission anéchoïque
atteint 21 dB à m = 22,5. Un élément purement réactif redistribue l'énergie
entre transmis et réfléchi ; dans une boucle collecteur-sortie réfléchissante,
ce qui est renvoyé revient. **C'est précisément pour cela qu'un vrai silencieux
est réactif ET absorbant.** Aucune des deux moitiés ne suffit seule, et c'est
la mesure qui le dit, pas une intuition.

Et la correction naïve est **déjà réfutée** dans `parts/exhausts.yaml` : creuser
la chambre unique jusqu'à la profondeur d'un vrai silencieux « a mis un cran de
16 dB sur la fondamentale de plage d'utilisation de l'EJ25 et coupé de moitié le
haut du K20 et du LS3 ». Le même fichier nomme le correctif correct : *« un vrai
silencieux multi-chambres est conçu précisément pour que ses crans ne tombent
pas sur un ordre d'allumage »*.

### Ce qu'il faut faire à la place

1. **Cascade de chambres** dans `ExpansionChamberMuffler`, longueurs décalées
   de façon non harmonique, pour que les bandes passantes de l'une soient
   couvertes par les crans des autres. Reste purement réactif
   (Kelly-Lochbaum, sans perte), donc le second échec documenté — une
   absorption large bande qui se compose dans la boucle collecteur-sortie —
   est structurellement impossible.
2. **Autorer le garnissage absorbant** sur les moteurs routiers. Cette action a
   depuis été réalisée avec des valeurs explicitement marquées estimées ; elle
   ne constitue toujours pas une calibration OEM.
3. **Équilibre des couches sur le 2JZ**, où l'admission et la mécanique
   masquent l'échappement dans 24 bandes sur 28.

## Ce qui reste ouvert et non diagnostiqué

- La marge LS3 reste proche du temps réel dans le scénario produit le plus
  coûteux ; le gate courant passe, mais la cible de confort 1,10x reste ouverte.
- **Plafond compresseur corrigé le 20 août 2026.** Le double clamp 1,16/1,12
  effaçait jusqu'à 23,7 kW. Le bilan turbine/compresseur/palier est maintenant
  conservatif et visible. Il reste à obtenir une vraie carte compresseur avec
  provenance avant de prétendre modéliser surge, choke ou rendement hors point.
  Voir `docs/turbo-shaft-energy-implementation-2026-08-20.md`.
