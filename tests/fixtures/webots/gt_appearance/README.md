# Webots appearance ground truth (tests/webots_pbr.test.ts)

`*.json.gz` hold, for every `Shape` of a MURKA world, its world-space characteristic points (box corners, cylinder /
cone cap centres, sphere centre, plane corners, a mesh's frame; mm), size signature, mesh file name and the full
appearance **as Webots R2025a reports it** (`PBRAppearance`: baseColor, transparency, roughness, metalness,
emissiveColor, emissiveIntensity; `Appearance`: its Material). They were extracted from the supervisor dumps of the
2026-10-25 verification (Webots R2025a, batch mode, copies of the worlds with gravity 0 and the controllers off, dump
before the first step) by `tests/_probe/g1_pbr_fixture.test.ts`.

| fixture | MURKA world | shapes |
|---|---|---|
| retail_store_v2 | robot-retailer-murka-v2/ros2_ws/src/murom_webots/worlds/retail_store.wbt | 190 |
| murka_sim_retail_store | simulation_ws/src/MURKA_Simulation/ros2_ws/src/murka_simulation/worlds/retail_store.wbt | 187 |
| murca_garden | simulation_ws/src/agromarca/murca_garden_ws/src/murca_webots/worlds/murca_garden.wbt | 14 |
| murca_apple_garden | simulation_ws/src/agromarca/murca_garden_ws/src/murca_webots/worlds/murca_apple_garden.wbt | 191 |
