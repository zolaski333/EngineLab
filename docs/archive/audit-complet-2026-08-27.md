# Audit complet EngineLab — 27 août 2026

## Verdict exécutif

La branche `codex/engine-corrections-roadmap`, basée sur `75f64fa`, est dans un
état **techniquement livrable pour recette utilisateur** : la solution Release
compile sans avertissement, les 44 tests CTest passent, les scénarios AFR et banc
signalés sont reproduits par des harness dédiés puis corrigés, et le ZIP extrait
démarre réellement.

Ce verdict n'est pas une affirmation impossible du type « il ne reste aucun bug ».
Il signifie plus précisément :

- les cinq problèmes signalés ont une cause identifiée et une correction intégrée ;
- les régressions automatisables connues passent sur le code final ;
- aucun défaut bloquant n'est encore confirmé dans le périmètre mesuré ;
- l'appréciation subjective du son d'admission et le parcours GUI complet à la
  souris restent à faire par un humain sur le matériel audio cible ;
- les modifications sont encore dans le worktree et ne sont pas commitées.

Le niveau de décision recommandé est donc **GO ingénierie / recette produit
manuelle requise avant diffusion publique**.

## Périmètre et méthode

L'audit a couvert les modules moteur, ECU, carburant, combustion, dynamique des
gaz, échappement, audio temps réel, transmission, banc, diagnostics, interface,
catalogue, tests et packaging Windows. Le changement technique touche 34 fichiers
et représente environ 2 049 insertions et 255 suppressions, hors ce rapport et
artefact ZIP.

La méthode a combiné :

1. reproduction des scénarios signalés sur le catalogue réel ;
2. télémétrie par cylindre et compteurs monotones de combustion/ratés ;
3. correction des causes physiques ou métrologiques, sans compresseur, sample,
   normalisation par moteur, enrichissement AFR arbitraire ni lissage cosmétique ;
4. tests ciblés après chaque changement ;
5. build Release complet `/W4 /WX` ;
6. suite CTest intégrale ;
7. mesure temps réel moteur + renderer audio de production ;
8. création, extraction et smoke test du paquet distribuable.

La recherche statique de premier niveau ne trouve ni `TODO`/`FIXME`/`HACK` dans
le code de premier niveau, ni appel évident à `strcpy`, `sprintf` ou `system`.
`git diff --check` ne trouve pas d'erreur d'espace ; les messages CRLF de Git sont
des avertissements de conversion de fin de ligne, pas des erreurs de contenu.
Ce contrôle ne remplace toutefois pas un analyseur statique, un fuzzer ou un audit
de vulnérabilités des dépendances.

## Résultat des problèmes signalés

| Problème | Diagnostic confirmé | Correction | Preuve finale | Statut |
|---|---|---|---|---|
| AFR turbo extrêmement pauvre à haut régime | Capacité injecteur trop faible, métrique de livraison ambiguë et demande de charge contaminée par les crêtes acoustiques pendant la longue fenêtre d'injection | Injecteurs catalogue portés à des capacités plausibles, télémétrie headroom/duty réelle, charge échantillonnée au début du pulse à haut régime et adaptation séparée ralenti/charge | 2JZ 10,0 %, EJ25 6,6 %, Audi I5 4,1 % d'erreur AFR haut régime ; zéro raté avant limiteur ; headroom minimum 45,1 %, 34,1 %, 40,6 % | Clos automatiquement |
| Son d'admission désagréable | Paramètres de milieu, délai, pertes et ouverture papillon changeaient par blocs ; la reconstruction pouvait créer des clics et une modulation à la cadence du callback | Interpolation par échantillon à dérivée continue, délais/pertes/milieu rampés, échantillonnage mécanique découplé du temps mur, suppression des sauts d'horloge audio | `AudioRender`, `AudioRuntimeDiscontinuity` et `AudioTransients` passent ; zéro drop/retard/limiteur/clamp sur le catalogue temps réel | Cause technique close ; jugement subjectif à confirmer |
| Point de couple/puissance extrêmement bas ou haut au banc | Le frein comparait une vitesse rotor filtrée à une consigne instantanée, freinait l'accélération demandée et pouvait archiver une observation rejetée ; puissance et couple pouvaient être interpolés indépendamment | Consigne filtrée avec même délai de groupe, retour sur erreur d'accélération, contact minimal 95 %, gel/réacquisition du bin rejeté, fenêtres de cycles complets, `P=Tω` imposé au bin | CP2 : 155 points de 1 800 à 9 500 tr/min, zéro récupération ; LS3 : 104 points de 1 120 à 6 270 tr/min, zéro récupération ; tests banc complets verts | Clos automatiquement |
| Courbes banc illisibles et impossibles à gérer | Historique sans identité visuelle stable ni gestion utilisateur | Nom éditable, palette stable, masquer/afficher, suppression, légende couple/puissance, info-bulle au survol, sélection enrichie, export CSV v3 avec validité AFR | Contrôles visibles dans l'arbre d'accessibilité de la Release ; tests archive/schéma/qualité verts | Fonctionnel ; clics manuels et persistance à recetter |
| AFR après coupure complète puis reprise plein gaz | AFR fictif pendant la coupure, film/volume de port mal comptabilisé, reprise partielle non synchronisée et apprentissage pollué par des cycles non observables | Bit de validité AFR, nouvelle époque d'observation, purge d'un cycle, pulse complet synchronisé, bilan conservatif du film et de l'empreinte injecteur, trim de reprise borné, séparation du transitoire de densité et du temps de stabilisation du film | AFR valide en 67–83 ms sur les moteurs essence ; erreur maximale 8,2–17,3 % ; zéro raté post-DFCO sur toutes les configurations essence exécutables | Clos automatiquement |

## AFR et carburant : détails

### Causes racines

Le défaut turbo ne venait pas d'une simple carte AFR. Trois problèmes se
superposaient :

- les injecteurs `sequential_port` et `high_flow_port` étaient respectivement à
  8 et 12 g/s, trop faibles pour certains débits du catalogue ; ils passent à
  16 et 18 g/s avec une fenêtre angulaire réaliste conservée ;
- l'ancien ratio de livraison mélangeait commande, film et inventaire présent,
  ce qui pouvait faire croire à une saturation injecteur inexistante ;
- le maximum d'une estimation de charge recalculée pendant toute une fenêtre de
  350 degrés transformait une crête de pression d'admission en carburant pour le
  cycle entier. L'EJ25 produisait ainsi un cycle riche isolé à AFR 9,51 malgré
  55 % de marge injecteur.

La correction conserve l'oxygène réellement résolu comme borne physique, mais
échantillonne l'estimation acoustique au début du pulse à haut régime. À bas
régime, la densité reste instantanée car le changement pendant la longue fenêtre
correspond réellement à un mouvement de papillon. La transition 3 600–4 400
tr/min évite une marche de commande.

### Résultats haut régime turbo

| Moteur | Erreur AFR max haut régime | Headroom injecteur min | Ratés physiques avant limiteur | Ruptures de couple |
|---|---:|---:|---:|---:|
| 2JZ-GTE-like | 10,0 % | 45,1 % | 0 | 0 |
| EJ25-like | 6,6 % | 34,1 % | 0 | 0 |
| Audi I5-like | 4,1 % | 40,6 % | 0 | 0 |

La limite du harness est 12 % au haut régime avec erreur moyenne inférieure à
8 %. Les trois moteurs passent sans coupure carburant/allumage avant limiteur et
sans trame AFR invalide.

### Reprise DFCO

La reprise ne fabrique plus une valeur numérique de type 100:1 pendant une phase
où le mélange n'est pas observable. Chaque cylindre doit fournir une nouvelle
combustion alimentée avant que l'AFR moteur redevienne valide.

Le premier correctif a révélé deux défauts secondaires utiles :

- sur le Merlin, reprendre à 1,25 fois le ralenti ne laissait pas le temps à la
  purge et au pulse complet ; le moteur passait sous 300 tr/min. La reprise est
  avancée à 1,50 fois le ralenti, encore sous l'entrée DFCO à 1,65 ;
- sur le CP4, l'estimation instantanée restait active pendant les deux secondes
  de stabilisation du film. À 6 040 tr/min, une crête tardive a livré 26,48 mg
  pour environ 269 mg d'air et créé un raté riche à AFR 9,90. Le transitoire de
  densité est maintenant borné à 0,35 s, tandis que le film continue à se
  stabiliser indépendamment.

Résultat final essence : AFR valide en 67–83 ms, erreur maximale de 8,2 à 17,3 %
selon le moteur, et zéro événement de raté post-reprise. Le diesel est évalué
séparément par sa limite de fumée/AFR pauvre, car il ne doit pas suivre une cible
stœchiométrique essence.

## Banc et lecture des données

### Métrologie

Le banc n'archive plus les valeurs instantanées d'une impulsion de combustion.
Il consomme le travail et le temps de cycles frein complets, exige la continuité
de séquence, le contact du frein et l'absence de limiteur/capacité saturée. Une
observation rejetée fige la prochaine vitesse non mesurée, réinitialise la
fenêtre et réacquiert le point ; elle ne devient jamais un zéro ou un pic dans
la courbe.

Pour chaque bin fixe, la puissance est recalculée depuis le couple et la vitesse
exacte (`P=Tω`). Le banc sait aussi approcher progressivement un moteur déjà à
haut régime, récupérer d'un calage éventuel et plafonner la rampe à une vraie
borne de régime.

### Ergonomie

L'écran banc propose désormais :

- un nom stable par run, modifiable ;
- une couleur stable et cyclable ;
- masquer/afficher une courbe ;
- supprimer une courbe ;
- une légende nommée, couple plein et puissance plus claire/pointillée ;
- une info-bulle du point le plus proche au survol ;
- le statut, le nombre de points et la puissance corrigée dans le sélecteur ;
- un export CSV incluant `actual_afr_valid` et les données de qualité utiles.

Limite connue : nom, couleur et visibilité sont actuellement une présentation de
session. Elles ne sont pas sérialisées dans l'archive au redémarrage, et la
suppression ne propose pas d'annulation. Ce n'est plus bloquant pour lire et gérer
une session, mais c'est une dette UX réelle.

## Audio et admission

Le problème d'admission n'a pas été « corrigé » en baissant simplement son volume.
Les sources physiques restent actives. Les changements portent sur la continuité
du réseau acoustique :

- milieu gazeux, ouverture papillon, délai de propagation et pertes de paroi sont
  interpolés à la fréquence audio avec deux pôles appariés ;
- la première frontière réelle initialise directement le réseau au lieu de
  balayer depuis l'air ambiant ;
- la fréquence mécanique de production est publiée par le simulateur ; le
  renderer ne la déduit plus des intervalles de temps mur soumis au scheduler ;
- une interruption de callback ne saute plus 40–120 ms de physique en conservant
  un état récursif ancien ; l'horloge revient progressivement et expose le drift ;
- des diagnostics distinguent pression source soupape, onde d'échappement,
  admission et bruit de jet.

Les tests automatisés ne trouvent aucune discontinuité interdite. Ils ne peuvent
pas décider si le timbre plaît à l'utilisateur. Une écoute A/B humaine reste
obligatoire, sans quoi il serait malhonnête de déclarer le grief subjectif clos.

## Autres défauts découverts et corrigés

- **Calage Merlin après coup de gaz** : la DFCO relançait le carburant trop tard ;
  tout le catalogue ralenti/blip passe désormais, Merlin inclus.
- **Trou de couple au changement de rapport** : le couple moteur revenait trop
  vite alors que l'embrayage dissipait encore l'écart de vitesse. La restitution
  suit maintenant une double courbe douce jusqu'à la synchronisation ; les tests
  audio de changement de rapport NA et turbo passent.
- **Coupures classées comme ratés** : les coupures intentionnelles ECU/limiteur
  sont séparées des ratés physiques, avec compteurs monotones par événement.
- **AFR non observable présenté comme une mesure** : les états moteur, banc et
  export transportent maintenant une validité explicite.
- **Pertes mécaniques haut régime sous-estimées** : un terme commun de pertes
  vitesse/accessoires a été ajouté au-dessus de 4 000 tr/min sans toucher les
  ralenti/blip ; les références catalogue et la régression de friction passent.
- **Échantillonnage audio couplé au scheduler** : la cadence mécanique est
  désormais indépendante du temps mur, ce qui élimine une source de variation
  de hauteur/bande passante lors d'un retard du thread.

## Validation finale

### Build et tests

- Build : Visual Studio 2022, Release x64, `/m:1 /nr:false`.
- Résultat : **0 avertissement, 0 erreur**, 41,97 s en incrémental final.
- CTest : **44/44**, 0 échec, 902,00 s.
- Sont notamment verts : Core, PhysicsRegression, CatalogPhysics,
  CatalogReference, IntakeTuning, PartLoadFuelling, TurboDownstreamAuthority,
  IdleStabilityRegression, AudioRender, AudioRuntimeDiscontinuity,
  AudioTransients, deux changements de rapport, accélérations chargées,
  catalogue DFCO, trois scénarios banc utilisateur et DieselControl.

### Budget temps réel frais

Une passe catalogue a exécuté le moteur 240 Hz et le renderer de production à
48 kHz / 256 échantillons, à 75 % du limiteur, en free-run :

- 16/16 moteurs respectent le contrat ;
- pire capacité physique : LS3, **1,151× temps réel** ;
- pire P99 audio : Merlin, **88 % d'un bloc** ;
- Merlin mesuré plus longuement : 1,380×, moyenne audio 58,2 %, P99 83 % ;
- zéro overrun moteur, render over-budget, drop, retard, voix volée, troncature de
  délai, frontière invalide, AGC, saturation, soft limit, hard clamp ou valeur
  non finie.

La marge LS3/Merlin est suffisante sur cette machine, mais pas énorme. Le
free-run inclut le renderer de production, pas une vraie pile WASAPI/ASIO avec
la GUI et les autres logiciels de l'utilisateur. Il faut donc conserver ce
point comme risque de performance surveillé.

### Paquet Windows

- Artefact : `EngineLab-0.1.0-win64.zip`.
- Taille : 6 616 117 octets.
- SHA-256 : `B87BE7B0E2260786153F59765B60BC9DAE514E8DE267AF7F6D0841AA81334CD6`.
- Extraction : 114 fichiers, dont 16 catalogues `.engine.yaml`.
- Smoke test : l'exécutable extrait reste vivant après 6 secondes ; il est
  ensuite arrêté proprement par le test.

### Contrôle utilisateur GUI

La Release locale a été ouverte et son arbre d'accessibilité expose bien
`Courbes archivées`, `COULEUR`, `MASQUER` et `SUPPRIMER COURBE`. L'automatisation
de capture/clic échoue toutefois sur cette fenêtre JUCE avec une erreur COM
`SetIsBorderRequired` (`0x80004002`) et ne fournit pas de géométrie exploitable.
Le démarrage du paquet est confirmé, mais pas un parcours intégral à la souris.

## Limites et risques résiduels

1. **Écoute subjective non réalisée** : aucune métrique ne remplace une écoute
   humaine de l'admission sur casque et enceintes de référence.
2. **Recette GUI partielle** : visibilité/accessibilité et démarrage sont
   confirmés ; création, renommage, couleur, masquage, suppression, survol et CSV
   doivent être cliqués par un humain dans le paquet.
3. **Marge temps réel machine-dépendante** : LS3 à 1,151× et Merlin à 88 % P99
   justifient un test avec le vrai périphérique audio et la GUI ouverte.
4. **Merlin et Radial sans rapports routiers** : les harness d'accélération
   chargée les déclarent non applicables. Ils sont couverts en ralenti, physique,
   audio et banc, mais pas en conduite fixe.
5. **Présentation banc non persistante** : noms/couleurs/visibilité disparaissent
   au redémarrage ; suppression sans corbeille/annulation.
6. **Pas de campagne longue durée** : absence de soak d'une heure, fuzz des
   imports JSON/YAML/CSV et tests répétés de branchement/débranchement audio.
7. **Sanitizers et dépendances** : ASan/UBSan sont prévus par CMake mais n'ont pas
   été exécutés dans cette validation Windows Release ; aucune campagne CVE
   formelle n'a été menée pour JUCE 8.0.13, nlohmann/json 3.12.0 et yaml-cpp 0.8.0.
8. **Pas de comparaison live ES2D** : aucune conclusion de supériorité sonore ou
   physique sur un autre simulateur ne peut être tirée de cet audit.

## Plan d'action

### P0 — avant diffusion publique

1. Faire une recette humaine du ZIP : démarrage, cinq écrans, sélection moteur,
   ralenti, blip, accélération, passage banc, renommage/couleur/masquage/suppression,
   survol et export CSV.
2. Faire une écoute A/B aveugle admission activée/désactivée puis avant/après sur
   au moins K20, 2JZ, LS3, Merlin et CP2, casque puis enceintes.
3. Mesurer LS3 et Merlin avec le périphérique WASAPI/ASIO cible, GUI ouverte et
   charge utilisateur réaliste ; conserver comme gates `sim/wall > 1,10`, P99
   audio < 90 % et zéro render over-budget/drop/clamp.

### P1 — durcissement prochain lot

4. Sérialiser nom, couleur et visibilité des courbes ; ajouter confirmation ou
   corbeille/annulation à la suppression.
5. Ajouter un soak automatique de 30–60 minutes avec changements de moteur,
   rapports, DFCO, banc et reconfiguration audio.
6. Exécuter ASan/UBSan sur la configuration supportée et fuzz les entrées
   JSON/YAML/CSV ainsi que les limites de catalogue.
7. Ajouter au CI le build Release, les 44 tests, CPack ZIP, extraction et smoke
   test de l'exécutable empaqueté.

### P2 — suivi et qualité continue

8. Conserver les métriques AFR haut régime, DFCO, P99 audio, capacité physique et
   qualité banc comme tendances historiques, pas seulement comme seuils binaires.
9. Ajouter un harness de parcours UI ou des tests de composants JUCE indépendants
   de la capture COM Windows.
10. Auditer périodiquement les versions et avis de sécurité des trois dépendances
    externes épinglées.

## Conclusion

Les problèmes signalés étaient réels et plusieurs partageaient des causes plus
profondes que leur symptôme visible : observabilité AFR, inventaire de carburant,
temps de cycle complet, cohérence filtre/consigne et continuité acoustique. Ils
ont été corrigés au niveau de ces causes. L'état automatisé final est solide et
reproductible ; la seule façon honnête de terminer la qualification produit est
maintenant une recette humaine du ZIP, surtout pour le timbre d'admission et le
parcours banc à la souris.
