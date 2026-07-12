# Passe ciblée ES2D — gaz, combustion, injection et audio

Date de référence : 12 juillet 2026.

Ce document trace la passe d'implémentation issue de l'analyse comparative
jointe. Il distingue le code livré des écarts volontairement laissés à une
prochaine passe.

## Livré dans cette passe

### Bugs et stabilité

- suppression du blend permanent couple moyen/couple pression ;
- couple moteur piloté par la pression cylindre résolue ;
- correction de l'accélération piston par estimation au milieu du sous-pas ;
- séparation conservée entre phase thermodynamique et géométrie radiale ;
- couple de démarreur recalibré pour franchir les pics de compression résolus ;
- asservissement du banc basé sur un régime filtré, sans être réinitialisé par
  l'ondulation cylindre par cylindre ;
- suppression de la limite arbitraire de 35 % par transfert gazeux ;
- correction de l'avancement de réaction : moles absolues au lieu d'une
  fraction répétée de la masse restante.

### Physique des gaz et combustion

- momentum 2D, pression totale directionnelle et conservation des deux axes ;
- transfert d'enthalpie, énergie cinétique macroscopique et équilibre de
  pression borné ;
- vitesse du son locale et reconversion de l'excès cinétique en chaleur ;
- capacités thermiques et `gamma` variables avec la composition ;
- modèle de flamme Metghalchi-Keck modulaire ;
- turbulence, dilution, rendement et propagation ellipsoïdale dans une chambre
  dont le volume évolue ;
- friction piston Stribeck par cylindre.

### Injection

- débit dépendant de la pression rail/volume ;
- injection indirecte avec film liquide persistant et vaporisation thermique ;
- injection directe avec chaleur latente et refroidissement de charge ;
- paramètres complets dans le catalogue, JSON, YAML et validation ;
- distinction carburant mesuré, vaporisé, présent à l'étincelle et consommé.

### Audio

- convolution partitionnée longue au lieu d'une FIR directe de 512 taps ;
- routage d'une IR mono/stéréo par chemin d'échappement ;
- lecture et ré-échantillonnage hors callback ;
- IR de secours dérivée de la géométrie pour chaque chemin ;
- mélange pression/dérivée, jitter, bruit d'air, leveler et anti-alias ;
- conservation du moteur de voix stéréo, des délais physiques et du FDN.

## Hors périmètre de cette passe

Conformément à la demande, les éléments suivants n'ont pas été implémentés :

- remplacement de la vue JUCE 2D par des meshes ou un rendu 3D ;
- scripting Piranha/Lua ou expressions conditionnelles dans les configs ;
- nouvelle contrainte rigide complète de transmission/embrayage ;
- bielle maîtresse et bielles articulées pour les moteurs radiaux ;
- export/enregistrement WAV depuis l'UI ;
- import de profils de soupape depuis un fichier CSV externe ;
- diagnostics OBD-II et nouvelles normes de correction dyno.

## Limites restantes dans le périmètre physique

- le réseau gazeux reste un réseau de volumes 0D et non un solveur CFD/ondes 1D
  maillé ;
- la chimie reste une réaction globale essence/oxygène ;
- le spray et le film sont des modèles agrégés, sans gouttelettes 3D ;
- le knock et les transferts thermiques restent semi-empiriques ;
- huit chemins convolutifs maximum et 262 144 échantillons chargés par IR ;
- l'égalité perceptuelle absolue avec un exécutable ES2D n'est pas revendiquée
  sans scénario A/B exécuté sur les deux applications.

Ces limites sont explicites : elles ne sont ni masquées par un preset ni
contournées par une exception propre à un moteur.
