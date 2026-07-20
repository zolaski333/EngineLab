# Livraison des phases 0 à 3

> **Archive historique.** Les mesures et le renderer décrits ici précèdent la
> chaîne thermoacoustique physique. Pour l’état exécutable actuel, consulter
> [thermoacoustic-architecture.md](thermoacoustic-architecture.md) et
> [realtime-audio.md](realtime-audio.md).

Date de validation : 16 juillet 2026.

Ce document décrit les corrections réellement intégrées et mesurées. Il ne
prétend pas que la simulation est déjà étalonnée scientifiquement sur tous les
moteurs ni que la préférence audio face à ES2D est démontrée sans écoute A/B.

## Phase 0 — observabilité et critères de non-régression

- Le runtime compte chaque échéance manquée et conserve sa latence maximale.
- Le harnais audio refuse les pertes d'événements de combustion, de trames de
  pression, les événements rendus en retard et le vol de voix.
- Les niveaux RMS, crêtes, clipping, offset continu, équilibre spectral,
  différenciation entre moteurs et stabilité longue sont mesurés.
- Le harnais catalogue peut produire `catalog-dyno.csv` pour un moteur filtré.
  Chaque point est acquis à pleine charge sur un frein en N·m après une fenêtre
  de régime et d'accélération continûment stable.
- Un filtre catalogue qui ne correspond à aucun moteur est maintenant une
  erreur au lieu d'un succès silencieux.
- Les références de journal et de banque sont validées explicitement ; les
  anciennes configurations sont migrées, mais une contradiction auteur n'est
  plus réparée silencieusement.

Validation Release finale : 12 tests CTest sur 12 réussis, dont le catalogue
physique complet, le runtime temps réel et un rendu audio long.

## Phase 1 — interaction, ralenti, banc et données moteur

Les touches de gaz sont momentanées. Q/W/E/R appliquent leur ouverture tant que
la touche est maintenue ; le relâchement ou la perte de focus remet immédiatement
la commande conducteur à zéro. Le ralenti n'est plus obtenu en maintenant
secrètement le papillon ouvert : un actionneur d'air de ralenti PI séparé pilote
l'aire de bypass configurée.

Le banc applique maintenant un couple de frein explicite en N·m. Sa régulation
utilise le couple moteur mesuré en anticipation, une correction PI et un
amortissement sur l'accélération moyennée à l'échelle du cycle. Les courbes
enregistrent le couple de réaction réellement appliqué, pas une consigne 0–1
reconvertie après coup. La capacité de l'absorbeur est distincte de l'échelle
de réglage du contrôleur afin de pouvoir retenir un moteur au-dessous de son pic
de couple.

Le schéma moteur en mémoire est passé en version 3. Il décrit maintenant :

- le nombre et le diamètre des soupapes d'admission et d'échappement ;
- le nombre de corps de papillon identiques et leur diamètre individuel ;
- la topologie mécanique explicite sans références implicites invalides.

Ces champs sont disponibles en JSON, YAML, catalogue et DSL. L'utilisateur peut
donc saisir « 4 papillons de 46 mm » ou « 2 soupapes de 33 mm » sans calculer
une aire équivalente et sans écrire de C++.

## Phase 2 — gaz, thermodynamique et frottements

- Les deux cellules adjacentes à un débit accélèrent dans le sens du jet ; la
  réaction manquante appartient aux parois, pas à un recul fictif du gaz amont.
- La pression dynamique des deux cellules est projetée sur la même normale
  d'interface. L'ancien changement de signe créait un gradient auto-entretenu
  même à pression et vitesse égales.
- Les deux débits d'un croisement de soupapes sont évalués sur le même état
  initial, bornés ensemble puis engagés avec conservation masse/espèces/énergie
  et correction du terme cinétique croisé.
- Le changement de volume adiabatique conserve exactement `P*V^gamma` pour un
  pas fini.
- Une cellule sans énergie ne fabrique plus une température et une pression
  minimales dérivées.
- La limite artificielle de vitesse à Mach 1 a été retirée. Les pertes utilisent
  Darcy–Weisbach, Sutherland et Haaland, plus les coefficients de pertes locales ;
  l'énergie cinétique perdue devient de la chaleur.
- Le rendement de combustion représente la fraction chimiquement réagie : le
  carburant non brûlé reste présent et chaque mole réellement brûlée libère son
  PCI complet.
- La dilution résiduelle réduit effectivement la vitesse de flamme au lieu
  d'être annulée par une mauvaise borne.

La pression d'échappement publiée est la pression totale du collecteur dans la
direction de sortie. Sur le test intégré, le réseau ouvert donne environ
103 kPa et le réseau fortement restreint 125 kPa, avec baisse du débit d'air et
du couple. Un DAG explicite utilise son volume, sa longueur et son diamètre
hydraulique dérivés ; les anciens champs géométriques ne le modifient plus.

## Phase 3 — échappement et audio

Un événement d'échappement conserve jusqu'à quatre composantes route/mode avec
leur délai fractionnaire, gain, résonance et chemin. Les composantes les plus
énergétiques sont retenues, triées causalement et renormalisées en énergie. Le
renderer ne réduit donc plus toutes les branches d'un DAG à un seul délai moyen.

Les guides de runners et les réflexions d'échappement utilisent des délais
fractionnaires interpolés. Chaque sortie conserve son guide aller/retour, son
état non linéaire, son FDN et sa convolution. Le transfert entre pression dans
le tube et auditeur inclut le rayonnement de sortie ; la puissance acoustique
est aussi liée à la cylindrée unitaire au lieu de rendre un gros radial
artificiellement silencieux.

Dernière mesure manuelle du harnais conservatif :

| Moteur | RMS | Crête | Pertes/retards/voix volées |
|---|---:|---:|---:|
| I4 | 0,084 | 0,549 | 0 / 0 / 0 |
| V8 | 0,081 | 0,371 | 0 / 0 / 0 |
| I2 | 0,127 | 0,414 | 0 / 0 / 0 |
| Radial 5 | 0,050 | 0,300 | 0 / 0 / 0 |

Le rendu long I4 mesure environ 0,109 RMS pour 0,694 en crête, sans clipping ni
plateau. La similarité spectrale maximale entre les quatre moteurs testés est
0,805 ; elle était nettement plus proche de 1 auparavant.

## Cas Hayabusa

Le pic trop précoce avait deux causes cumulées :

1. le solveur inversait la normale de pression dynamique de la cellule aval,
   ce qui pouvait créer du débit et de la pression sans gradient physique ;
2. le fichier décrivait un seul papillon et des soupapes sans géométrie
   multi-soupapes explicite.

Le preset décrit maintenant quatre papillons et deux soupapes d'admission/deux
soupapes d'échappement par cylindre. La courbe stabilisée pleine charge place le
pic calculé autour de 9 000 tr/min, et non plus 6–7 000 tr/min. La puissance
calculée est proche de 99 kW : l'allure est beaucoup plus cohérente, mais la
valeur reste sous la référence d'un Hayabusa réel. Le preset doit donc encore
être étalonné avec une courbe de banc, des profils de came et des données de
flowbench ; aucun multiplicateur caché de couple n'a été ajouté.

Après un démarrage volontairement très accéléré, la mesure gaz fermés revient à
environ 1 330 tr/min pour une consigne de 1 250 tr/min. Le bypass de ralenti est
désormais la seule source d'air commandée quand le conducteur relâche les gaz.

## Banc : EngineLab ou ES2D ?

Le banc ES2D inspecté impose la vitesse par contrainte et propose une montée
continue d'environ 500 tr/min/s. C'est immédiat, lisible et agréable pour voir
une courbe se dessiner, mais une rampe mélange l'inertie, le contrôleur et le
couple stationnaire.

Le banc EngineLab stabilisé par absorption est préférable pour une mesure
physique répétable, maintenant qu'il travaille en N·m et exige une fenêtre
stable. ES2D reste plus plaisant pour l'expérience visuelle de sweep.

Le bon produit n'est donc pas de choisir exclusivement l'un des deux :

- mode **Palier stabilisé** pour étalonnage, comparaison et export ;
- mode **Sweep inertiel** avec inertie de rouleau, accélération et pertes
  explicitement configurées pour l'expérience rapide ;
- même graphe avec courbe brute, corrigée et intervalle d'incertitude.

## Solution proposée pour créer un moteur sans C++

La prochaine couche doit être un assistant guidé au-dessus du schéma déclaratif,
pas un générateur de code :

1. choix d'une famille et d'un usage (route, moto, course, aviation) ;
2. architecture, alésage/course, compression et géométrie de bielle ;
3. assistant culasse affichant directement nombre/diamètre de soupapes et
   profils de came issus d'une bibliothèque versionnée ;
4. admission avec nombre × diamètre de papillons, runners et plénum ;
5. concepteur visuel d'échappement déjà amorcé par le DAG ;
6. prévalidation automatique : géométrie, débit, vitesse piston, injecteurs,
   ralenti, rupteur, thermique et capacité du banc ;
7. essai virtuel court, puis banc stabilisé avec explication des limites ;
8. sauvegarde d'un paquet moteur contenant configuration v3, ECU, échappement,
   IR, provenance des pièces et rapport de calibration.

Les paramètres thermodynamiques avancés restent disponibles dans un mode
expert, mais l'assistant doit proposer des valeurs physiques sourcées et
expliquer leur effet en langage courant. Si une cible de couple est fournie, un
optimiseur peut suggérer des dimensions ou profils dans des bornes réalistes ;
il ne doit jamais appliquer un multiplicateur invisible pour faire coïncider la
courbe.

## Limites honnêtes avant un livrable final

- Le Hayabusa et les autres presets n'ont pas encore de dossiers de calibration
  traçables avec données réelles et tolérances.
- Le DAG d'échappement alimente une physique 0D agrégée ; un solveur acoustique
  1D par segment avec impédances et pertes fréquentielles reste nécessaire pour
  viser une reproduction de systèmes réels.
- La supériorité audio perceptuelle face à ES2D doit être établie par des écoutes
  aveugles à sonie égale, sur un corpus de références multi-microphones.
- L'assistant moteur décrit ci-dessus n'est pas encore implémenté en UI.
