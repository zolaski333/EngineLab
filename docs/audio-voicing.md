# Voicing audio declarative

La voicing est la couche de monitorage non physique d'EngineLab. Elle ne
modifie ni la combustion, ni les pressions cylindre, ni la dynamique des gaz,
ni le couple. Son objectif est de permettre un travail d'ecoute rapide sans
recompiler et sans cacher une correction physique dans un gain arbitraire.

## Resolution par couches

Le catalogue fusionne, dans cet ordre :

1. `voicing/default.yaml` ;
2. `voicing/families/<famille_normalisee>.yaml`, si present ;
3. `voicing/engines/<cle_moteur_normalisee>.yaml`, si present.

Une couche ne remplace que les champs qu'elle declare. La famille vient du
champ `family` du fichier moteur. La cle moteur vient du nom de fichier sans
la derniere extension, puis est normalisee en minuscules avec des `_` (par
exemple `11_yamaha_cp2_mt07_like.engine.yaml` devient
`11_yamaha_cp2_mt07_like_engine.yaml`).

Le schema est strict. Une cle inconnue, une valeur non finie, hors plage ou un
placement de saturation inconnu refuse le nouvel instantane complet. Dans
l'application, la derniere voicing valide reste alors active et l'erreur est
affichee. Les fichiers sont sondes une fois par seconde sur le thread UI ; le
callback audio ne fait aucune E/S et ne lit que des atomiques lock-free.

## Champs schema 1

```yaml
schema_version: 1
voicing:
  volume: 1.0                  # 0..2
  convolution: 0.45           # 0..1
  high_frequency_gain: 1.0    # 0.2..2.5
  low_frequency_gain: 1.0     # 0.2..2.5
  low_frequency_noise: 0.35   # 0..1.5
  high_frequency_noise: 0.35  # 0..1.5
  combustion_gain: 1.0        # 0..2
  exhaust_gain: 1.0           # 0..2
  intake_gain: 0.85           # 0..2
  mechanical_gain: 0.70       # 0..2
  stereo_width: 1.0           # 0 mono, 1 neutre, 2 large
  outlet_jet_gain: 1.0        # 0..2, bruit de jet physique seulement
  saturation_drive: 0.0       # 0 bypass exact, puis 0..4
  saturation_placement: post_shelf # pre_shelf ou post_shelf
```

La saturation a un gain petit-signal unitaire et son bypass `0.0` est exact.
La largeur stereo utilise un traitement mid/side uniquement lorsqu'elle est
differente de `1.0`. Le gain de jet agit sur la composante de pression de jet
separee publiee par le reseau acoustique d'echappement.

## Profils catalogue et A/B instantane

Les seize moteurs livres possedent maintenant un override sous
`voicing/engines/`. Ces profils sont des presentations `estimatedFamily` : ils
mettent en avant le caractere produit par la pression, la topologie et la
geometrie, mais ne sont pas annonces comme des egalisations micro mesurees. Les
prises CC0 dont le regime, la charge ou la geometrie micro sont inconnus ne
servent qu'a encadrer le caractere attendu.

Dans **AUDIO HQ**, **VOICING CATALOGUE** rappelle l'override complet du moteur et
**NEUTRE** restaure toutes les valeurs du schema, pas seulement les neuf faders
visibles. Le changement est instantane et ne redemarre pas le moteur. Deplacer
un fader conserve desormais `low_frequency_gain`, `stereo_width`,
`outlet_jet_gain`, `saturation_drive` et `saturation_placement`; ils etaient
auparavant remis silencieusement au defaut par la reconstruction partielle du
mix.

Un changement de moteur pendant que l'atelier est ouvert recharge egalement son
profil catalogue et ses disponibilites physiques. L'A/B n'est donc plus expose
a un mix appartenant au moteur precedent.

## Non-regression du defaut

Le fichier livre reprend exactement les anciennes constantes compilees. Sur le
scenario K20A `showcase`, 48 kHz float32, master seul, le rendu avant et apres
l'introduction du catalogue donne le meme SHA-256 :

`851B708DC02385D7A141746484002EED63B140425324FF778E10FF93648AE221`

Cette egalite bit a bit est la condition de base : toute future voicing audible
doit etre un override explicite et faire l'objet d'une ecoute A/B aveugle.
