# Agricultural robotics workflow

1. **Field**: Agriculture › Create field / orchard. Pick a crop preset (apple spindle, pear 2D trellis, cherry V,
   citrus hedgerow, grape, strawberry/tomato/cucumber gutters, blueberry, olive, almond, kiwi pergola, row crop),
   dimensions, row spacing/heading, headland, ripe fraction. Or import a GeoJSON boundary (File › Open) and
   regenerate rows on it. The generator creates `CropRow` items with plants and fruit positions, a navigation map
   (rows rasterised as obstacles, inflation) and headland zones.
2. **Fleet**: Mobile & Fleet › Create fleet: number/type of platforms (tracked harvest platform, orchard sprayer,
   Ackermann tractor, AMR, scout rover), allocation strategy, charging zone. Platforms can carry arms (add a robot
   from the library and drag it onto a mobile robot in the tree, Shift-drop keeps the relative pose).
3. **Mission**: Agriculture › Create mission (harvest, spray, mow, prune, scout, weed, pollinate, transport, thin,
   irrigate), select rows (`1-5,8`), work speed, both sides. Planning produces one fleet task per row side with a
   traffic segment; harvesting tasks are sized by the number of ripe fruit on that side.
4. **Run**: start the world clock (toolbar 🌍 ▶ or Simulation tab). The Fleet tab shows robots, batteries, tasks,
   KPIs (tasks/h, utilisation, distance, energy, waiting). Robots return to the charging zone when below the
   battery threshold.
5. **Arm programs**: select an arm mounted next to a row, Robot › Fruit picking program: reachable ripe fruit
   become approach/pick/retreat targets with gripper events; export with any post or the ROS 2 node.
6. **Export**: missions/tasks are available through the API (`RDK.App('fleetManager', fleet)`), programs via
   post processors, the whole scene as `.vbstation`, field boundaries as GeoJSON.

Models are parametric and deterministic (seeded), so scenarios are reproducible for what-if studies
(fleet size vs. throughput, row spacing vs. cycle time, battery capacity vs. charging downtime).
