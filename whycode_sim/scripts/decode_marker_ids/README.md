# Measuring the WhyCode texture -> id mapping

A WhyCode id is a necklace code, not the texture's file index: `whycode_3.png` does not
decode to 3. Getting it wrong gives ground truth that is right in position and wrong in
identity, so the mapping is measured rather than assumed.

## How

`necklace_decode.cpp` links `whycode_vision`'s own `CNecklace` and exposes its `decode()`
on the command line, so the answer is the id the detector will report.

`decode_marker_ids.py` samples the encoding ring out of each texture, converts it to the
same 2*id_bits sample string the detector builds, and feeds it through that binary.

Two checks on the result:

- **Phase sweep.** A necklace code is rotation-invariant, so a correct read is stable
  across all 60 sampling phases within a half-bit segment. Measured: 59 of 60 agree for
  every texture, the dissenter being the phase landing on a black/white transition.
- **Radius sweep.** Re-run at 0.19 to 0.27 of image width; the answer does not move.

The nine textures decode to an exact permutation of 0..8. A sampling error would collide
or produce -1 instead.

## Running

```bash
g++ -std=c++17 -O2 -I<whycode_vision>/include -o decode_main necklace_decode.cpp \
    <whycode_vision>/src/core/CNecklace.cpp
python3 decode_marker_ids.py
```

Needs numpy and Pillow. Run it from this directory so it finds `./decode_main`.

## Result, 2026-08-27, id_bits 6, hamming 1

| texture | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|
| decoded id | 8 | 6 | 7 | 3 | 4 | 5 | 1 | 2 | 0 |

This is what `marker.families.whycode.ids` in `config/scene.yaml` holds. Re-run it if the
textures are ever regenerated, or if `id_bits`/`hamming_distance` change in
`whycon_config_sim.yaml` -- the necklace table depends on both.

AprilTag ids need no measurement: they are the number in the filename.
