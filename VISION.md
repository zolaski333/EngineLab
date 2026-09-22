# EngineLab — Vision

Ce document fixe l'intention du projet. Il change rarement, et seulement par
décision du propriétaire. Tout le reste (code, docs, priorités) doit s'y
conformer.

## Ce que doit être EngineLab

Un simulateur de moteur quatre temps **dont le son est le plus proche possible
de la réalité**, pour n'importe quel moteur que l'utilisateur construit.

1. **Le son d'abord.** Quelqu'un qui connaît le moteur réel doit pouvoir le
   reconnaître à l'écoute : pas seulement « un bicylindre », mais « un CP2 ».
   La méthode est libre : physique, synthèse, ou les deux. Ce qui compte est le
   résultat audible, pas l'élégance du modèle.
2. **Une physique correcte dans les grandes lignes.** Couple et puissance
   plausibles (de l'ordre de ±15 % des valeurs constructeur), réactions
   crédibles à l'accélérateur, au rapport, à l'échappement. La physique sert le
   son et le ressenti ; elle n'est pas une fin en soi.
3. **Temps réel sur un PC de milieu de gamme.** Machine de référence :
   Intel i5-10600 (6 cœurs / 12 threads), 16 Go de RAM.

## Contrainte fondamentale : aucun enregistrement par moteur

Un moteur construit par l'utilisateur (par exemple un W16) doit sonner de façon
réaliste **à partir de sa seule description** : géométrie, ordre d'allumage,
admission, échappement. L'utilisateur ne doit jamais avoir à fournir un
enregistrement de son moteur.

- Les enregistrements réels servent à **calibrer le modèle générique** pendant
  le développement. Ils ne sont jamais un ingrédient obligatoire du rendu.
- Des éléments sonores **génériques**, non propres à un moteur précis, sont
  autorisés dans le rendu : texture de bruit mécanique par famille, claquement
  type d'une pétarade, réponse d'un environnement d'écoute, etc.

## Critère de réussite

Le son est jugé **contre des enregistrements réels au régime connu**, jamais
contre la sortie du simulateur lui-même.

- **Objectif :** écart de timbre simulation/réel au même régime et à la même
  charge, mesuré par un outil dédié.
- **Subjectif :** écoute à l'aveugle.

Jalons :

1. **Moteur pilote : Yamaha CP2 (MT-07), échappement Arrow.** Enregistrements
   faits par le propriétaire sur sa propre moto, régime connu.
2. **Généralisation :** un second moteur très différent doit s'améliorer
   **sans réglage spécifique**. C'est la preuve que la calibration sur le CP2 a
   amélioré le modèle générique et ne s'est pas contentée d'imiter une moto.

## Non-objectifs

- Une reproduction 1:1 d'un moteur réel.
- De la physique pour elle-même : un modèle plus fidèle qui ne change rien
  d'audible ou de ressenti n'est pas prioritaire.
- Un outil d'analyse thermodynamique ou de calibration de vrai véhicule.
- Ajouter un composant (silencieux garni, catalyseur, résonateur…) sans écart
  audible mesuré qui le justifie.

## Règle de décision

Avant tout travail, répondre à une question :

> **Quel écart mesuré entre la simulation et la réalité ce travail réduit-il ?**

Sans réponse, le travail attend. Exceptions : un bug qui casse le programme,
et la performance temps réel sur la machine de référence.
