# Third-party notices

The Unity project is not uniformly Apache-2.0. The following components retain
their upstream licenses and terms:

| Component | Location | License / notice |
| --- | --- | --- |
| MARUS platform sources | `Packages/com.iwin-fins.fins-sim/Runtime/Platform` | Apache-2.0 files with LABUST copyright notices; preserved at `Docs/ThirdParty/MARUS-LICENSE` and in source headers. |
| MARUS gRPC/protobuf/plugin dependencies | `Packages/com.iwin-fins.fins-sim/Runtime/Plugins` | Each nested component retains its own notice and redistribution terms; it is not covered by the FinsSim dual license. |
| Flexible Color Picker | `Assets/Plugins/FlexibleColorPicker` | Retains its upstream notice and redistribution terms; it is not covered by the FinsSim dual license. |
| Crest (Built-in reference edition) | `Assets/Crest/Crest` | MIT; copyright Wave Harmonic and contributors. This upstream GitHub edition targets Unity's Built-in Render Pipeline, not HDRP; see `Assets/Crest/README.md`. |
| UnityHDRPSimpleWater | `Assets/FinsSimUnity/Content/PlatformEnvironment/UnityHDRPSimpleWater` | MIT; copyright Çağlayan Karagözler. |
| OpenSourceCloth components | `Assets/ThirdParty/OpenSourceCloth` | Apache-2.0 or the license in each nested component directory. |
| Unity ML-Agents | `Packages/com.unity.ml-agents` | Apache-2.0; copyright Unity Technologies. |
| NWH Common / Dynamic Water Physics | `Packages/com.nwh.common`, `Packages/com.nwh.dynamicwaterphysics` | Unity Asset Store Terms of Service; headers identify NWH Coding d.o.o. as copyright owner. |

The project also contains models, shaders, plugins, samples, and package
dependencies whose terms may be supplied in adjacent files or by their
provider. Those materials require a release-specific audit. Do not remove or
alter upstream notices, and do not publish the NWH packages, Asset Store
assets, or any other third-party material as FinsSim-owned Apache/commercial
code without written permission.
