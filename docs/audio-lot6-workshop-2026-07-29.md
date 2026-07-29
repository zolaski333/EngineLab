# Lot 6 — atelier audio et contrôles honnêtes (2026-07-29)

## Résultat livré

Le bouton **AUDIO HQ** ouvre un atelier séparé sans arrêter le moteur. Cette
fenêtre réunit :

- les faders du mix temps réel ;
- `MUTE` et `SOLO` pour les quatre familles de sources ;
- le choix 48/96/192 kHz et PCM 24 bits/float 32 bits ;
- master seul ou master plus six stems ;
- le profil `showcase` intégré ou un scénario JSON choisi par l'utilisateur ;
- un rendu hors ligne sur thread séparé ;
- progression, annulation, ouverture du dossier produit et rapport final ;
- l'identité du moteur et l'état du graphe physique.

Le rendu reste disponible pendant une simulation ou un passage au banc : il
travaille sur une copie cohérente de la configuration et n'emprunte pas le
thread audio. Fermer la fenêtre pendant un rendu ne détruit pas la tâche.
Quitter l'application demande l'annulation puis attend proprement le thread.

Chaque lancement crée un sous-dossier horodaté. L'application ne réutilise donc
pas silencieusement un ancien `master.wav`.

## Contrôles honnêtes

L'ancien écran `MIXER / AUDIO` affichait plusieurs paramètres qui ne pouvaient
plus agir après compilation des graphes physiques. Ce n'était pas un défaut du
DSP : les voix procédurales correspondantes sont volontairement retirées. Le
défaut était l'interface, qui continuait à présenter ces paramètres comme
actifs.

L'atelier et les raccourcis appliquent maintenant la table suivante :

| Contrôle | Graphe physique | Comportement |
|---|---:|---|
| Master | tous | actif |
| Retour IR mesuré | seulement si une IR a été chargée | désactivé et forcé à zéro sans IR |
| Aigus master | tous | actif |
| Bruit admission legacy | seulement sans graphe d'admission | désactivé avec le graphe quasi-1D |
| Bruit échappement legacy | seulement sans graphe d'échappement | désactivé avec le graphe thermoacoustique |
| Combustion directe | seulement sans graphe d'échappement | désactivée : la pression cylindre excite déjà l'échappement |
| Échappement | tous | actif, sec et retour IR |
| Admission + suralimentation | tous | actif |
| Structure / mécanique | tous | actif |

Les touches historiques `X`, `V`, `B` et `J` ne modifient plus une valeur
inaudible dans les cas `N/A`. L'écran principal l'indique également au lieu
d'afficher une jauge trompeuse.

Les faders visibles constituent le **mix de base**. Mute/solo produit un **mix
effectif** distinct envoyé au thread audio et au rendu HQ. Ainsi, un solo ne
détruit pas la valeur d'un autre fader ; la désactivation du solo restitue le
mix précédent.

## Sémantique dry/IR

Le contrôle IR est nommé **retour IR mesuré** et non « convolution » :

- le dry n'est pas atténué ;
- la valeur 0..1 ajoute jusqu'à 50 % de retour mesuré, conformément au routage
  existant de `RealtimeEngineAudio` ;
- sans fichier IR valide, le contrôle est désactivé et le mix effectif vaut
  zéro ;
- l'export HQ permet d'inspecter séparément `stem_exhaust_dry.wav` et
  `stem_exhaust_ir.wav`.

Il n'y a donc ni preset de pièce caché, ni faux wet à 100 %.

## Preuves automatiques

Le test `EngineLab.AudioWorkshop` construit la vraie fenêtre JUCE et vérifie :

- trois tailles, de la taille minimale 980×620 à 1600×900 ;
- chaque composant visible possède des dimensions non nulles ;
- aucun composant ne sort de la fenêtre ;
- neuf faders sont présents ;
- les quatre faders sans effet du cas physique sans IR sont désactivés ;
- quatre paires MUTE/SOLO existent et la combustion directe est indisponible ;
- l'action d'export HQ est visible ;
- le routage de solo conserve l'échappement et son retour IR, puis coupe
  admission et structure ;
- `MUTE` reste prioritaire sur `SOLO` ;
- le retour au mode de compatibilité réactive les contrôles réellement
  disponibles.

Commande :

```powershell
cmake --build out\build\windows-vs2022 --config Release `
  --target EngineLabApp EngineLabAudioWorkshopTests `
  --parallel 1 -- /nr:false /m:1 /v:minimal

ctest --test-dir out\build\windows-vs2022 -C Release `
  -R "^EngineLab\.AudioWorkshop$" --output-on-failure
```

Résultat :

```text
Audio workshop: layout, truthful availability, mute/solo routing
and HQ export controls PASS
```

Le `EngineLab.exe` Release a aussi été lancé depuis son chemin de build exact et
a créé une fenêtre native `EngineLab`. La capture automatisée de cette fenêtre
JUCE n'était pas disponible sur cette session Windows
(`SetIsBorderRequired`, `0x80004002`) ; aucune coordonnée n'a donc été cliquée à
l'aveugle. Le test de géométrie ci-dessus remplace explicitement cette preuve
visuelle manquante.

## Limite volontaire

Le bus suralimentation possède un stem hors ligne distinct, mais partage encore
le fader temps réel de l'admission. Le séparer dans le runtime changerait le
contrat historique des harnais qui utilisent « mute intake » pour isoler
l'échappement. Cette séparation pourra être faite dans une évolution dédiée,
avec A/B et mise à jour de tous les isolateurs ; elle n'a pas été glissée dans
ce lot d'interface.
