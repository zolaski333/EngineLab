# Architecture thermoacoustique physique

Ce document fixe le contrat de la nouvelle chaîne gaz/acoustique. Il ne décrit
pas un preset sonore : il définit les grandeurs physiques qui ont le droit de
devenir du son, leur propriétaire et les validations exigées avant de retirer le
chemin historique.

## Objectif

Le rendu cible doit être la conséquence de la simulation, y compris lorsque la
charge, le régime, la température, le croisement de soupapes ou la géométrie de
l'échappement changent. Le renderer peut représenter la propagation entre la
sortie et le microphone ainsi que la pièce d'écoute. Il ne doit pas reconstruire
un « faux » blowdown avec des oscillateurs, du bruit ou une enveloppe accordée à
la main.

Le chemin physique est :

```text
combustion 0D du cylindre
        ↕ flux conservatif à la soupape
réseau gaz 1D non linéaire, composant par composant
        ↕ impédance de rayonnement
pression acoustique au microphone
        → propagation spatiale / réponse de la pièce
```

Chaque flèche avant le microphone est bidirectionnelle ou conserve explicitement
la masse et l'énergie. Une réponse de pièce reste un traitement acoustique aval ;
elle ne corrige jamais une dynamique de gaz insuffisante.

## Responsabilités des modules

### Thermodynamique cylindre

`GasCell` reste, dans un premier temps, le volume de contrôle 0D du cylindre. Il
fournit pression, température, composition et énergie. La soupape d'échappement
ne transfère plus un débit calculé vers un runner 0D indépendant : elle devient
une condition de bord conservative du réseau 1D. Le flux retourné au cylindre
doit inclure les inversions de débit et la contre-pression instantanée.

La loi de combustion actuelle est compatible avec cette première étape. Elle
calcule déjà une histoire de pression et d'énergie utile au blowdown. Une
validation par pression cylindre mesurée améliorerait l'exactitude absolue, mais
n'est pas une condition structurelle au passage de 0D à 1D.

### Dynamique des gaz 1D

Le module `EngineLabGasDynamics` résout les équations d'Euler quasi-1D sous
forme volumes finis : masses des espèces, quantité de mouvement axiale et
énergie totale. Il utilise un flux de Riemann capturant les chocs, une
reconstruction TVD et un pas borné par CFL. Frottement de paroi et transfert
thermique sont des termes sources explicites ; une restriction géométrique
n'est jamais convertie en gain audio.

Le graphe d'échappement est compilé sans réduction acoustique :

- les tubes, catalyseurs et sorties deviennent des conduits discrétisés ;
- les merges, splitters, résonateurs et chambres de silencieux deviennent des
  jonctions ou volumes de contrôle ;
- longueurs, sections, volumes, pertes et coefficients de décharge conservent
  leur sens physique ;
- toutes les cellules et zones de travail sont allouées à la construction.

### Rayonnement et renderer

Une sortie publie au minimum la pression statique, la température, le débit
massique, la vitesse axiale, la section et la direction. Le rayonnement convertit
pression et vitesse de volume en pression acoustique en respectant la charge
d'impédance de l'ouverture et la distance du microphone.

Le renderer physique peut encore appliquer :

- retard et atténuation de propagation dans l'air ;
- directivité de la bouche ;
- HRTF, réponse de cabine ou de pièce ;
- protection de sortie transparente contre les valeurs non finies.

Il ne peut pas ajouter d'oscillateur accordé au régime, de « crack » aléatoire,
de résonateur de silencieux synthétique ni de bruit de jet servant à masquer
une bande manquante. Ces couches peuvent survivre temporairement derrière un
chemin de comparaison explicite, jamais dans le rendu physique par défaut.

### Son mécanique et structurel

La pression cylindre seule ne décrit pas le bruit rayonné par le bloc. Une
implémentation honnête exige un modèle modal réduit du bloc, de la culasse et des
carters, excité par les efforts calculés : force gazeuse sur piston, efforts
d'inertie, réactions de paliers, impacts de distribution et contacts. Le signal
de chaque mode est ensuite rayonné par sa surface et sa directivité.

Ce volet demande donc une extension de la physique mécanique actuelle. Garder
les sinusoïdes de vilebrequin ou le bruit de distribution et les rebaptiser
« physique » violerait ce contrat.

## Invariants numériques

Toute évolution du solveur doit conserver les propriétés suivantes :

1. Une solution uniforme reste uniforme à l'arrondi près.
2. Sans frontière ouverte ni source thermique, masses d'espèces et énergie
   totale sont conservées.
3. La densité, l'énergie interne et la pression restent strictement positives
   sans ajout arbitraire de masse ou d'énergie.
4. Le sens et le temps de transit d'une onde suivent les caractéristiques
   `u ± c`.
5. Une discontinuité forte converge vers la solution de Riemann de référence
   sans oscillations non physiques.
6. Le bilan aux soupapes et jonctions est calculé depuis un état gelé commun,
   afin que l'ordre d'itération ne crée aucun débit.
7. Une correction de positivité réduit ou rejette le pas fautif ; elle ne
   « clamp » jamais une espèce en créant de l'inventaire.

Les tests analytiques du module sont obligatoires et indépendants du voicing
audio.

## Temps réel et déterminisme

La boucle hôte reste à 240 Hz, mais le réseau 1D choisit ses sous-pas d'après
le CFL local. La topologie, les cellules, les faces et les buffers RK sont
précompilés. Aucun verrou, allocation, I/O ou parcours du graphe auteur ne doit
se produire dans la boucle chaude.

Les réductions aux jonctions emploient un ordre stable. Le même moteur, les
mêmes commandes et le même pas produisent les mêmes trames bit à bit sur une
même cible. Le budget de performance est mesuré sur le V8 de référence ; le
nouveau réseau remplace l'ancien chemin runner/collector au lieu de s'y ajouter
durablement.

## Migration et critères de retrait du chemin historique

1. Valider le noyau 1D seul : uniforme, tube à choc de Sod, conservation,
   transit acoustique, réflexion et positivité.
2. Compiler le DAG et tester chaque type de composant, les branches et les
   bilans de jonction.
3. Raccorder les soupapes en aller-retour et vérifier phasing, scavenging,
   contre-pression, stabilité et déterminisme sur le catalogue.
4. Publier les sorties physiques puis remplacer le blowdown DSP par le
   rayonnement.
5. Comparer l'ancien et le nouveau chemin par mesures de pression, spectre,
   cohérence de phase, niveau, performance et écoute en aveugle.
6. Activer le nouveau chemin par défaut seulement quand tous les tests sont
   verts, puis supprimer les couches synthétiques devenues sans propriétaire.
7. Ajouter le modèle structurel modal avant de retirer les anciennes couches
   mécaniques.

Un drapeau de comparaison pendant cette migration sert à mesurer et revenir en
arrière en cas de régression. Il n'est pas une excuse pour mélanger les deux
modèles dans la sortie livrée.

