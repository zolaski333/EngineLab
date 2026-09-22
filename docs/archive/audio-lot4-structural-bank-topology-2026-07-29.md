# Lot audio 4A — coordonnées modales par banc

Date : 29 juillet 2026  
État : corrigé, A/B mesuré, aucun gain de voicing ajouté

## Défaut

Le modèle structurel estimé connaît les bancs et leurs listes de cylindres, mais
calculait la position longitudinale d’un cylindre avec :

```
index_du_vecteur % cylindres_par_banc
```

Cette formule n’est juste que si les cylindres sont stockés banc après banc. Les
V et les flat du catalogue sont normalement stockés `1, 2, 3, 4...`, avec des
bancs explicites `1, 3, 5, 7...` et `2, 4, 6, 8...`. Le modulo plaçait donc les
cylindres d’un même rang longitudinal sur des antinœuds différents et repliait
d’autres rangs l’un sur l’autre.

Ce défaut touchait les modes estimés de flexion du bloc, de plaque de culasse et
de torsion. Les modes authored ou mesurés n’étaient pas concernés : leur vecteur
de participation par cylindre est déjà autoritaire.

## Correction

`StructuralModalRadiator` construit désormais, pour chaque cylindre, sa
coordonnée normalisée depuis sa position réelle dans
`EngineConfig::banks[].cylinderIds`. Aucune fréquence, masse modale, aire,
efficacité de rayonnement, force d’excitation ou valeur de gain n’a changé.

L’ancien mapping reste accessible par un contrôle nul de harnais :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabAudioRenderHarness.exe `
  --structural-bank-comparison LS3 `
  --output out/validation/audio-lot4-structure-2026-07-29/ls3
```

Les dossiers `flat-index` et `bank-topology` viennent du même binaire et du même
scénario déterministe.

## A/B Release

| Moteur | Différence RMS relative | Pic différence | Pic structure avant | Pic structure après | Cosinus spectral |
|---|---:|---:|---:|---:|---:|
| LS3 V8 | 8,21 % | 0,04364 | 10,215 Pa | 10,194 Pa | 0,999835 |
| Flat-6 | 0,83 % | 0,01022 | 3,923 Pa | 3,950 Pa | 0,999998 |
| Merlin V12 | 1,04 % | 0,00375 | 1,015 Pa | 1,106 Pa | 0,999545 |

Le V8 est logiquement le plus affecté : son ordre de stockage alterné et ses
quatre positions par banc maximisaient l’erreur de modulo. Les niveaux
structurels restent du même ordre ; le changement porte sur la phase et la
participation, pas sur une amplification globale.

Les trois couples A/B ont :

- topologie échappement et structure modale actives ;
- zéro perte d’événement ou de pression ;
- zéro frontière invalide, fallback, leveler ou échantillon non fini ;
- pré-limiteur maximal inférieur à `0,59`.

Preuves :
`out/validation/audio-lot4-structure-2026-07-29/{ls3,flat6,merlin}`.

## Régressions analytiques

Le test `structuralModalRadiatorRegression` impose maintenant :

- sur un V8 symétrique, les cylindres 1 et 2 au même rang longitudinal donnent
  une réponse structurelle exactement identique avec la topologie de banc ;
- le contrôle nul reproduit une différence non nulle, ce qui prouve que le test
  détecte le défaut historique ;
- sur un I4 à un seul banc, ancien et nouveau mapping restent bit-identiques ;
- les tests préexistants de résonance, amortissement, silence et paramètres
  physiques continuent de passer.

## Limite

Le fallback demeure une estimation de famille à coque/poutre/plaques. Il ne
devient pas une mesure NVH par cette correction. Des positions de sources
structurelles et des formes modales mesurées restent nécessaires pour une
spatialisation mécanique complète.
