# Third-party notices

EngineLab itself is released under the [MIT license](LICENSE.md). It is built on,
and ships with, the following third-party work.

## Libraries

| Component | Version | License | Source |
|---|---|---|---|
| JUCE | 8.0.13 | AGPLv3 or the commercial JUCE 8 licence | <https://github.com/juce-framework/JUCE> |
| nlohmann/json | 3.12.0 | MIT | <https://github.com/nlohmann/json> |
| yaml-cpp | 0.8.0 | MIT | <https://github.com/jbeder/yaml-cpp> |

The libraries are fetched by CMake at the pinned revisions above and linked
statically into the executable. Their full license texts are in their
repositories.

JUCE is dual-licensed: its modules are available under the AGPLv3 or under the
[JUCE 8 End User Licence Agreement](https://juce.com/legal/juce-8-licence/).
The MIT license of EngineLab covers EngineLab's own source code only. Anyone
redistributing a binary that links JUCE must do so under one of JUCE's own
licences.

## Exhaust impulse responses

The impulse responses in `assets/ir/` come from
[Engine Sim 2D](https://github.com/ange-yaghi/engine-sim) by Ange Yaghi
(AngeTheGreat), copyright © 2022 Ange Yaghi, under the MIT license. Per-file
provenance is in [assets/ir/README.md](assets/ir/README.md) and the license text
in [assets/ir/LICENSE-es2d.txt](assets/ir/LICENSE-es2d.txt).

## Microsoft Visual C++ runtime

Release archives include the Microsoft Visual C++ runtime DLLs
(`msvcp140.dll`, `vcruntime140.dll`, `vcruntime140_1.dll` and related), which
Microsoft permits to be redistributed alongside an application under the
Visual Studio license terms.

## Reference recordings

The real engine recordings used for A/B listening are not stored in this
repository. They are CC0 1.0 field recordings, downloaded and checksum-verified
on demand; authors and sources are listed in
[references/real-engine-audio/manifest.json](references/real-engine-audio/manifest.json).
