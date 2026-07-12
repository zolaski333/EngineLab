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
- calcul simultané et conservatif des flux admission/cylindre/échappement
  pendant le croisement des soupapes, sans biais lié à l'ordre d'appel ;
- knock par intégrale d'auto-inflammation Livengood-Wu du gaz de fin de
  combustion, alimentée par la pression et la température réelles de chaque
  `GasCell` ; l'auto-inflammation libère effectivement l'énergie du reliquat.

### Injection

- débit dépendant de la pression rail/volume ;
- injection indirecte avec film liquide persistant et vaporisation thermique ;
- injection directe avec chaleur latente et refroidissement de charge ;
- paramètres complets dans le catalogue, JSON, YAML et validation ;
- distinction carburant mesuré, vaporisé, présent à l'étincelle et consommé.
- commande de masse carburant issue de l'oxygène et de la charge réellement
  piégés, avec déduction de la vapeur et du film déjà présents ;
- le modèle mean-value réutilise la masse d'air et la VE résolues dès qu'elles
  existent ; ses références de came ne servent plus qu'à l'estimation de
  démarrage et à la télémétrie, jamais au couple ni à la dose injectée ;
- profils par défaut propres au mode : 570–690° et 20 000 mg/s pour une DI
  haute pression, 250–620° et 5 000 mg/s pour une injection port. Pour
  l'essence, 400 cc/min correspond à environ 4 970 mg/s, pas 306 mg/s.

### Audio

- convolution partitionnée longue au lieu d'une FIR directe de 512 taps ;
- routage d'une IR mono/stéréo par chemin d'échappement ;
- lecture et ré-échantillonnage hors callback ;
- IR de secours dérivée de la géométrie pour chaque chemin ;
- mélange pression/dérivée, jitter, bruit d'air, leveler et anti-alias ;
- conservation du moteur de voix stéréo, des délais physiques et du FDN.
- une trame contenant la pression de chaque cylindre est publiée à chaque
  sous-pas thermodynamique par une file SPSC dédiée ; le renderer interpole,
  filtre passe-haut et dérive ce signal continu. Un raté produit donc le son de
  sa pression réelle effondrée et non uniquement une impulsion atténuée.

### Transmission

- le runtime existant utilisait déjà les rapports, l'embrayage et le véhicule :
  `TransmissionConfig` n'était donc pas une structure morte ;
- glissement d'embrayage avec capacité de couple, zone de verrouillage et
  synchronisation calculée avec l'inertie moteur et l'inertie réfléchie ;
- inertie totale des roues motrices ajoutée à la masse équivalente du véhicule ;
- passage manuel temporisé avec débrayage, changement de rapport puis
  réembrayage, et passage automatique optionnel avec seuils montée/descente ;
- réaction signée sur le vilebrequin : un embrayage engagé à l'arrêt peut
  caler le moteur ; glissement, couple et progression du changement sont
  exposés dans `EngineState`.

## Hors périmètre de cette passe

Conformément à la demande, les éléments suivants n'ont pas été implémentés :

- remplacement de la vue JUCE 2D par des meshes ou un rendu 3D ;
- scripting Piranha/Lua ou expressions conditionnelles dans les configs ;
- bielle maîtresse et bielles articulées pour les moteurs radiaux ;
- export/enregistrement WAV depuis l'UI ;
- import de profils de soupape depuis un fichier CSV externe ;
- diagnostics OBD-II et nouvelles normes de correction dyno.

## Limites restantes dans le périmètre physique

- le réseau gazeux reste un réseau de volumes 0D et non un solveur CFD/ondes 1D
  maillé ;
- la chimie reste une réaction globale essence/oxygène ;
- le spray et le film sont des modèles agrégés, sans gouttelettes 3D ;
- le délai d'auto-inflammation end-gas est une corrélation globale calibrée,
  pas une cinétique chimique multi-espèces ni un champ spatial 3D ;
- les transferts thermiques restent semi-empiriques ;
- huit chemins convolutifs maximum et 262 144 échantillons chargés par IR ;
- l'égalité perceptuelle absolue avec un exécutable ES2D n'est pas revendiquée
  sans scénario A/B exécuté sur les deux applications.

Ces limites sont explicites : elles ne sont ni masquées par un preset ni
contournées par une exception propre à un moteur.
