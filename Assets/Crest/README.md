# Crest reference material

This directory retains the upstream open-source **Crest Water** source as
reference material outside the `com.iwin-fins.fins-sim` UPM package.

- Upstream: [wave-harmonic/crest](https://github.com/wave-harmonic/crest)
- License for the Crest core in this directory: [MIT](Crest/LICENSE), copyright
  Wave Harmonic and contributors.
- Pipeline: the upstream GitHub edition targets Unity's **Built-in Render
  Pipeline**. It is not an HDRP implementation.

Do not use this directory as a drop-in replacement for Crest Water HDRP. A
Crest-based HDRP project requires a separately purchased and installed
[Crest Water HDRP](https://assetstore.unity.com/packages/tools/particles-effects/crest-water-4-hdrp-ocean-rivers-lakes-164158)
asset. Likewise, the optional DWP2 integration requires a separately licensed
installation of Dynamic Water Physics 2. Neither commercial Asset Store
package is redistributed by FinsSim public releases.

FinsSim's open-source Fossen hydrodynamics and water-provider backends are a
separate alternative when a DWP2/Crest-HDRP workflow is not required. See
[`Docs/HydroDynamics/`](../../Docs/HydroDynamics/) for the maintained design
and usage documentation.

`Crest-Examples` may contain assets with terms separate from the Crest core;
preserve their accompanying notices when using or redistributing them.
