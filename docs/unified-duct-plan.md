# Plan : conduit unifié

Écrit le 2026-08-03, après le rapport utilisateur « ÉCHAP PRO fait tomber le
temps réel quoi que je fasse », « l'afterfire ne fait rien » et « un échappement
court sans silencieux ne change presque rien », puis l'examen du prototype
ES3D v0.4.0a fourni par l'utilisateur.

Ce document est le plan de référence. Il survit au compactage de contexte.

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

## Ce qui reste ouvert et non diagnostiqué

- L'afterfire (phase 0 le tranche).
- Le V8 encore sous le temps réel dans certains cas après `a9ff0dd`.
- Le plafond de vitesse compresseur (saturation 2,154 sur le 2JZ).
