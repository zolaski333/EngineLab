# Validation des sources d'échappement observables — 2026-08-01

## Décision d'architecture

Le blowdown et la spatialisation n'avaient pas besoin d'un nouveau modèle : la
production utilise déjà la paire caractéristique pression/débit à la soupape,
le DAG acoustique complet et une position/axe/charge/observateur par sortie.
Le défaut restant était diagnostique : `exhaust_dry` additionnait l'onde de
pression et la turbulence de jet, empêchant de savoir laquelle corriger après
une écoute.

## Livraison

Deux sous-stems HQ sont ajoutés :

- `stem_exhaust_pressure_wave.wav` : onde de réseau, donc blowdown, réflexions
  et afterfire réellement propagés ;
- `stem_exhaust_jet.wav` : source turbulente de sortie indépendante.

Le gain de jet est appliqué dans la décomposition. Les deux signaux conservent
les délais, distances et directivités stéréo de chaque sortie. Ils sont marqués
diagnostiques et exclus de la somme prémaster, qui continue d'utiliser le seul
bus `exhaust_dry`.

## Invariants exécutables

- activer les huit stems ne change aucun échantillon du master ;
- les six stems de mix reconstruisent toujours le prémaster ;
- `pressure_wave + exhaust_jet` reconstruit `exhaust_dry` en float ;
- le fallback historique met tout le signal dans `pressure_wave` et garde le
  jet exactement silencieux au lieu d'inventer une séparation ;
- le manifeste inventorie les deux fichiers et publie l'erreur de somme.

Validation ciblée Release : `EngineLab.RealtimeRegression` et
`EngineLab.OfflineAudioExport`, `2/2` passés, `0` échec. La qualité perceptive
reste à juger en écoute A/B ; ces stems rendent cette écoute localisable et ne
prétendent pas calibrer le niveau absolu d'une source sans microphone réel.

La passe finale publie une erreur de décomposition float de
`2,32831e-10`. Après encodage indépendant des WAV PCM24, l'erreur de somme des
stems de mix reste à `1,19209e-7`, soit un LSB au niveau de représentation.
