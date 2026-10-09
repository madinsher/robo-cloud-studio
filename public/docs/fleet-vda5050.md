# VDA 5050 — AGV / AMR fleet interface

[VDA 5050](https://github.com/VDA5050/VDA5050) is the standard MQTT/JSON interface between a fleet's *master
control* and its vehicles. It is what KUKA Fleet (KMP / KMR platforms), MiR, SEW, Jungheinrich, Linde, STILL,
Omron, ANT-driven vehicles and Open-RMF adapters speak. The studio implements **both sides** of it (v2.0 messages,
v2.1-compatible field names accepted):

| Role | What the studio does | Use case |
|---|---|---|
| **Master** | dispatches fleet tasks as `order` messages (nodes/edges/actions built from the planned path), sends `instantActions` (cancelOrder, startPause/stopPause, initPosition, startCharging…), mirrors every AGV's `state`/`connection`/`visualization`/`factsheet` onto the station's mobile robots | run the orchard/greenhouse fleet plan on real AMRs; monitor a mixed fleet in 3D |
| **AGV bridge** | every mobile robot of the station becomes a VDA 5050 vehicle: answers orders by driving the simulated robot, publishes `state` (1 Hz), `visualization` (5 Hz), retained `connection` (with last-will `CONNECTIONBROKEN`) and `factsheet` | test KUKA Fleet / your own dispatcher against digital twins before the hardware exists |

## Where it runs

The MQTT connection lives in the **studio server** (`npm run server`, `server/vda5050.ts`, mqtt.js). The browser
talks to it through `/vda5050/*` HTTP endpoints; the pure protocol logic is in `src/fleet/vda5050.ts` and is
shared (and unit-tested) by both.

- UI: `Connect › VDA 5050 fleet interface…` — broker URL (`mqtt://`, `mqtts://`, `ws://`, `wss://`), topic prefix
  (`uagv` by default), role, manufacturer, map id, credentials, shadow / auto-dispatch switches.
- Env: `STUDIO_MQTT_URL`, `STUDIO_VDA_PREFIX`, `STUDIO_VDA_ROLE` (`master` | `bridge` | `both`),
  `STUDIO_VDA_MANUFACTURER` connect at server start (e.g. a headless twin farm in CI).
- HTTP: `GET /vda5050/status`, `GET /vda5050/states`, `POST /vda5050/connect|disconnect|order|instantAction|link`.

## Conventions

- Topics: `<prefix>/v2/<manufacturer>/<serialNumber>/<order|instantActions|state|connection|visualization|factsheet>`.
- Units: metres / radians on the wire, mm / degrees in the station; `mapId` defaults to `station` (the station frame).
- Pairing: an AGV is linked to the mobile robot whose sanitized name equals its serial number (`Harvester 2` ↔
  `Harvester_2`), or set the robot param `vdaSerial`.
- Orders: nodes every 2 m along the planned path (`nodeSpacing`), `released` for the base, horizon beyond; the task
  type becomes the last node's action — `startCharging` for charging, `pick`/`drop` for transport, custom
  `harvest`/`spray`/`mow`/`prune`/`scout`/`pollinate`/`weed` actions (declared in the factsheet) for field work.
  Re-sending to a vehicle with a running order produces an *order update* (same `orderId`, `orderUpdateId + 1`).
- State mirroring: position, velocity, battery, driving/paused/charging/error status and the remaining node count
  land on the robot (`robot.params.vda5050`), so fleet KPIs and the 3D view follow the real vehicles.
- Twin behaviour: order validation per the standard (newer `orderUpdateId`, released nodes, no conflicting running
  order), node arrival by `allowedDeviationXY`, node actions marked `FINISHED` (`startCharging` stays `RUNNING`),
  instant actions `cancelOrder`, `startPause`, `stopPause`, `initPosition`, `startCharging`, `stopCharging`,
  `stateRequest`, `factsheetRequest`; unknown actions report `FAILED`.

## Quick start with a local broker

```bash
# any MQTT broker, e.g. mosquitto
mosquitto -p 1883
cd studio && STUDIO_MQTT_URL=mqtt://localhost:1883 STUDIO_VDA_ROLE=both npm run server
# open the studio with ?server=ws://localhost:20500, create a fleet with mobile robots, plan a mission:
# every assignment becomes an order; the twins drive and report back like real vehicles.
```

Tests: `tests/vda5050.test.ts` (messages, master bookkeeping, twin execution, end-to-end over an in-memory broker).
