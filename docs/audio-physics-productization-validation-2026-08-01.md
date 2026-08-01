# Productisation des physiques audio — validation du 1er août 2026

## Verdict

Les trois fonctions qui existaient sans chemin produit évident sont maintenant
essayables dans EngineLab :

- AUDIO HQ expose variation cycle-à-cycle, afterfire, rupteur spark-cut humide,
  démo/bypass et télémétrie live ;
- ÉCHAP PRO expose les trois paramètres de garnissage avec démo/bypass ;
- `Audio Physics Lab 689 Twin` active toute la chaîne sans modifier les quinze
  configurations historiques ;
- l'export HQ publie l'activité réellement mesurée dans un manifeste schéma 3.

Le comportement historique reste le défaut exact : COV nul, afterfire coupé,
rupteur fuel+spark cut et garnissage nul. La nouvelle variante est explicitement
un laboratoire d'écoute avec valeurs estimées, pas une calibration Yamaha.

## Preuve A/B de bout en bout

`EngineLabOfflineAudioExportTests` rend à 48 kHz float32 le même scénario court
avec le même moteur, d'abord en bypass exact puis avec la démo complète. Le
second rendu doit avoir une variation non neutre, brûler réellement du carburant
dans l'échappement, contenir un silencieux poreux et produire un WAV différent.

Résultat Release :

| Mesure | Bypass | Démo physique |
|---|---:|---:|
| COV auteur | 0 | 0,06 |
| Multiplicateur cycle mesuré | 1,000..1,000 | 0,858323..1,15301 |
| Afterfire | off | 99,5317 kW peak |
| Carburant brûlé dans l'échappement | 0 mg | 6 602,49 mg |
| Silencieux poreux | 0 | 1 |
| Delta WAV RMS | — | 0,0640149 |
| Delta WAV peak | — | 0,836792 |

Ces métriques prouvent une différence de signal et une excitation physique non
vacante. Elles ne prétendent pas prouver que le timbre est bon : ce verdict
reste une écoute humaine A/B.

## Chemins d'utilisation

1. Sélectionner **Audio Physics Lab 689 Twin** dans le catalogue.
2. Ouvrir **AUDIO HQ**. La ligne `LIVE` doit montrer une plage de cycles non
   unitaire. Monter au rupteur pour fournir du carburant imbrûlé ; les kW/mg/s
   d'afterfire restent honnêtement à zéro tant que les conditions chimiques ne
   sont pas réunies.
3. Utiliser **DEMO AUDIBLE** puis **BYPASS**. Chaque application redémarre le
   moteur parce qu'elle remplace sa configuration physique.
4. Ouvrir **ÉCHAP PRO**, générer le graphe depuis la géométrie si nécessaire,
   sélectionner le muffler et comparer **GARNISSAGE DEMO** à **BYPASS
   GARNISSAGE**. Valider/appliquer entre les écoutes.
5. Un export HQ enregistre les valeurs auteur et mesurées sous
   `audio_physics` dans `render-manifest.json`.

## Mesure temps réel finale disponible

Commande exécutée après le build Release complet :

```powershell
out/build/windows-vs2022/tools/Release/EngineLabRealtimeBudgetHarness.exe `
  --free-run --rpm 7000 --seconds 6
```

Cette passe est une preuve fonctionnelle, **pas une référence propre de marge** :
un échantillonnage juste après le run a mesuré `RobloxPlayerBeta` à environ
171 % d'un cœur logique, Chrome à environ 100 % cumulé et `audiodg` actif. Il
faut fermer cette charge externe et reprendre la table avant toute décision CPU.

| Moteur | Cyl. | Workers | Facteur | Overruns | P éch. moyenne | BP |
|---|---:|---:|---:|---:|---:|:---:|
| K20A-like 2.0 I4 VTEC | 4 | 2 | 1,507 | 0 | 100,8 kPa | non |
| 2JZ-GTE-like 3.0 I6 Turbo | 6 | 2 | 1,527 | 0 | 283,0 kPa | oui |
| LS3-like 6.2 Crossplane V8 | 8 | 2 | 1,133 | 0 | 110,6 kPa | non |
| EJ25-like 2.5 Flat-4 Turbo | 4 | 2 | 1,890 | 0 | 253,0 kPa | oui |
| Audi I5-like 2.5 Turbo | 5 | 2 | 1,696 | 0 | 305,5 kPa | oui |
| Hayabusa-like 1.3 I4 | 4 | 2 | 1,755 | 0 | 100,4 kPa | non |
| Big Twin-like 1.9 V2 | 2 | 0 | 3,333 | 0 | 102,6 kPa | non |
| Merlin-like 19.8 V12 Scaled | 12 | 3 | **1,050** | 0 | 89,3 kPa | non |
| Aircooled-like 3.6 Flat-6 | 6 | 2 | 1,310 | 0 | 107,4 kPa | non |
| Radial-like 6.5 R5 | 5 | 2 | 2,390 | 0 | 100,6 kPa | non |
| Yamaha CP2 MT-07-like 689 Twin | 2 | 0 | 2,999 | 0 | 95,4 kPa | non |
| Yamaha CP3 MT-09-like 890 Triple | 3 | 2 | 2,206 | 0 | 87,3 kPa | non |
| Yamaha CP4 MT-10-like 998 Crossplane I4 | 4 | 2 | 1,770 | 0 | 94,7 kPa | non |
| VW EA288-like 2.0 TDI I4 | 4 | 2 | 2,280 | 0 | 256,6 kPa | non |
| MT-07-like 689 Twin Full System | 2 | 0 | 2,948 | 0 | 95,9 kPa | non |
| Audio Physics Lab 689 Twin | 2 | 0 | 2,847 | 0 | 98,0 kPa | non |

La passe produit 16/16 moteurs au-dessus du temps réel et zéro overrun. Le pire
facteur contaminé 1,050 représente seulement 4,8 % de marge avant échéance ; il
ne satisfait pas la preuve demandée de 10–15 %. Aucune optimisation CPU n'est
justifiée depuis cette table contaminée.

## Validation logicielle

- Build MSVC Release complet : réussi.
- Suite complète : 31/31 tests réussis en 663,35 s avec `ctest -C Release -j 1`.
- Tests ciblés : `Core`, `Exhaust`, `AudioWorkshop`, `OfflineAudioExport`,
  `Scripting` et `VehicleDynamics` réussis pendant l'intégration.
- L'application Release lie avec les nouveaux contrôles AUDIO HQ et ÉCHAP PRO.
- Le test natif `AudioWorkshop` vérifie à 1 100×800, 1 280×860 et
  1 600×1 000 que chaque contrôle visible possède des dimensions non nulles et
  reste dans la fenêtre. La capture Windows automatique de JUCE reste bloquée
  par `SetIsBorderRequired: 0x80004002`; aucune inspection visuelle automatisée
  n'est donc revendiquée.
- CPack ZIP : 84 entrées. L'extrait liste les 16 moteurs et `EngineLab.exe`
  reste vivant après un smoke test caché de six secondes.
- L'exporteur **extrait du ZIP** rend le showcase du moteur laboratoire :
  734 400 frames / 15,3 s à 48 kHz float32, RMS 0,024855, zéro troncature,
  frontière invalide ou télémétrie perdue. Il mesure 0,858323..1,153011 sur les
  cycles, 71,970520 kW d'afterfire, 2 703,423250 mg brûlés et un silencieux
  poreux.

Artefacts finaux :

| Fichier | Taille | SHA-256 |
|---|---:|---|
| `EngineLab-0.1.0-win64-audio-physics-20260801.zip` | 6 263 567 | `F7CFC6082D433282DF846DF8DFF3899419764868103C68D95B42986D9F1AE9E8` |
| `EngineLab.exe` extrait | 8 690 176 | `AB456789271D5B421FAA2ED267673E8ECE4CDC345745EE2D89A9DBBB6AD7E1FA` |
| `master.wav` de preuve package | 5 875 244 | `71E99969453273E44D9C3CA314220C753AC320E9DAFE1B35A15C55971CDA889B` |

## Limites honnêtes

- Les valeurs démo sont choisies pour rendre le chemin observable, pas pour
  représenter un moteur ou un silencieux précis.
- La coupure DFCO reste prioritaire : l'afterfire n'est pas garanti à chaque
  lever de pied.
- L'application redémarre le moteur lors d'une modification physique ; il n'y a
  pas encore de morphing instantané sans reset.
- La qualité sonore finale doit être jugée à l'oreille. Les métriques A/B ne
  remplacent ni un casque ni une référence micro appariée.
