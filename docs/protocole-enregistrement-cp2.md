# Protocole d'enregistrement — MT-07 (CP2), échappement Arrow

But : obtenir la **cible sonore** du jalon 1 de `VISION.md`. Sans régime connu,
un enregistrement ne sert qu'à l'oreille ; avec régime connu, il sert à mesurer
l'écart simulation/réel bande par bande. Tout ce qui suit vise ce second usage.

## Avant de commencer : noter

- Modèle exact de l'Arrow : slip-on ou ligne complète, référence, catalyseur
  conservé ou non, **db-killer monté ou retiré**. Le timbre en dépend fortement.
- Année de la moto, kilométrage, filtre à air d'origine ou non, cartographie
  modifiée ou non.
- Température extérieure approximative, vent.

## Matériel

- **Idéal :** un enregistreur portable (type Zoom H1n/H4n, 80–150 €), en WAV
  48 kHz / 24 bits, gain manuel.
- **Acceptable :** un téléphone, **à condition de couper tous les traitements** :
  contrôle automatique de gain, réduction de bruit, « mode concert ». Utiliser
  une application d'enregistrement qui permet un gain fixe et le format WAV.
  Les traitements automatiques détruisent précisément le timbre qu'on veut
  mesurer.
- Un **second téléphone qui filme le tableau de bord** pendant toute la prise,
  pour lire le régime. Un clap devant les deux au début de chaque prise permet de
  les synchroniser.

## Réglage du niveau

Faire un essai à haut régime avant la vraie prise et vérifier qu'**aucun pic ne
sature** (rien au plafond sur la forme d'onde). Le niveau choisi ne change plus
ensuite, sinon les prises ne sont plus comparables entre elles.

## Positions de micro

Toujours les mêmes, mesurées au mètre ruban, micro à hauteur de la sortie
d'échappement :

- **A — proche :** 50 cm de la sortie, à 45° de l'axe du jet (jamais dans le
  jet : le souffle sature le micro).
- **B — scène :** 4 m sur le côté de la moto, côté échappement. C'est la
  distance d'écoute par défaut de la simulation, donc la plus directement
  comparable.

Enregistrer toute la séquence en A, puis toute la séquence en B. En extérieur,
dans un espace dégagé, loin des murs et des voitures garées (les réflexions
changent le timbre), par temps sans vent.

## Séquence (moteur chaud, à l'arrêt)

Moteur **chaud** (après au moins 10 minutes de roulage) : la température des gaz
change la vitesse du son dans l'échappement, donc ses résonances.

1. Clap, puis **ralenti** 20 s.
2. **Paliers stabilisés**, en tenant la poignée aussi constante que possible :
   2 000, 3 000, 4 000, 5 000 tr/min, environ 5 s chacun. Laisser redescendre
   au ralenti quelques secondes entre deux paliers.
3. **Montée lente** : du ralenti à 6 000 tr/min en environ 8 s, puis relâcher.
4. **Coups de gaz** : trois coups secs jusqu'à ~6 000 tr/min, relâchés
   franchement. C'est la prise qui capte les éventuelles pétarades de
   décélération.
5. Clap final.

Ménager le moteur : pas de palier long au-dessus de 5 000 tr/min à l'arrêt,
laisser tourner le ventilateur entre les séries, et respecter le voisinage.

## Optionnel, plus tard

- Prise en roulant (passage devant le micro à rapport et régime notés) : c'est
  la seule prise **en charge**, mais le régime est plus difficile à connaître.
- Même séquence avec le db-killer inversé (monté/retiré) : un changement d'un
  seul facteur, idéal pour vérifier que la simulation réagit comme la réalité.

## Livraison

Déposer les fichiers dans `references/cp2-mt07-arrow/` avec un court
`notes.md` : matériel, position (A/B), niveau de gain, conditions, et pour chaque
fichier l'instant (en secondes) de chaque palier et de chaque coup de gaz, lu
sur la vidéo du tableau de bord.
