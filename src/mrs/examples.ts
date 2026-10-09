/**
 * Course documents of the group-control module: one or more ready models per practicum (ПР1–ПР6), the homework
 * warehouse fleet, and the worked examples of the theory chapters. Loadable from *Group › Course examples…*,
 * used by the demo scenarios and the documentation.
 */
import { MrsKind } from './model';

export interface MrsExample { id: string; kind: MrsKind; name: string; /** ПР1 … ПР6, ДЗ or chapter */ part: string; chapter: string; source: string; summary: string }

export const MRS_EXAMPLES: MrsExample[] = [
  // ---------------------------------------------------------------- ПР1
  { id: 'pr1-chain', kind: 'consensus', part: 'ПР1', chapter: '4', name: 'ПР1 · Chain P₄: discrete consensus iterations (§4.3.4)', summary: 'x(0) = (0, 4, 8, 12), ε = 0.25: the three iterations of the text, ρ = 0.854, ≈ 30 iterations to 1 %; topology table 4.1.', source: `consensus Chain of four robots (§4.3.4)
graph path n=4
x0 0 4 8 12
eps 0.25
steps 30
compare path ring star complete
continuous true` },
  { id: 'pr1-formation', kind: 'consensus', part: 'ПР1', chapter: '4', name: 'ПР1 · Formation of six robots on a ring (step 3)', summary: 'Random start, ring graph, circular offsets r = 0.3: the shape error falls below 1 mm and the centre of ξ is invariant; one robot fails at 10 s.', source: `consensus Formation of six robots
robots 6 seed=3 box=1
graph ring n=6
formation circle r=0.3 gain=1 steps=600 dt=0.05
fail 4 at=10` },
  { id: 'pr1-rendezvous', kind: 'consensus', part: 'ПР1', chapter: '4', name: 'ПР1 · Rendezvous on the edge of range (step 4)', summary: 'A chain at 0.95 R: plain consensus breaks the links, the Ji–Egerstedt weights keep every initial link.', source: `consensus Rendezvous of a chain at the edge of the radio range
robot r1 at=0,0
robot r2 at=0.95,0
robot r3 at=1.9,0
robot r4 at=2.85,0
robot r5 at=2.85,0.95
graph disk radius=1
rendezvous radius=1 gain=0.5 dt=0.002 steps=5000 vmax=0.5
steps 20` },
  { id: 'pr1-wmsr', kind: 'consensus', part: 'ПР1', chapter: '4, 16', name: 'ПР1 · W-MSR against a malicious agent on K₇ (step 5, §16.8.3)', summary: 'Agent 7 alternates ±100; with F = 1 the normal agents converge inside [0, 0.9] while plain consensus is hijacked; K₇ is 3-robust.', source: `consensus W-MSR on a complete graph of seven
graph complete n=7
x0 0.1 0.4 0.2 0.9 0.5 0.3 0
eps 0.1
steps 40
wmsr F=1 eps=0.1 steps=300 malicious=7 value=100 pattern=alternate` },
  { id: 'pr1-sparse', kind: 'consensus', part: 'ПР1', chapter: '4, 16', name: 'ПР1 · W-MSR fails on a sparse chain (1-robust)', summary: 'The same attack on a path graph: a chain is defenceless against a single liar because it is only 1-robust.', source: `consensus W-MSR on a path of seven
graph path n=7
x0 0.1 0.4 0.2 0.9 0.5 0.3 0
eps 0.3
steps 40
wmsr F=1 eps=0.3 steps=300 malicious=7 value=100 pattern=constant` },
  { id: 'pr1-event-transport', kind: 'consensus', part: 'ПР1', chapter: '4, 13', name: 'ПР1 · Event-triggered consensus and cooperative transport (§13.5.2, §4.5.2)', summary: 'Broadcast only on a 15 % change (a fraction of the periodic traffic); three robots carry an object with a PD law: critical damping vs oscillation.', source: `consensus Ring of eight with event-triggered updates
robots 8 seed=2 box=2
graph ring n=8
eps 0.2
steps 300
event sigma=0.15 abs=0.01
transport target=5,2 m=1 kp=4 kd=4` },
  // ---------------------------------------------------------------- ПР2
  { id: 'pr2-boids', kind: 'swarm', part: 'ПР2', chapter: '5', name: 'ПР2 · Reynolds flock of twenty robots (step 1)', summary: 'Separation / alignment / cohesion; polarization rises above 0.9 without collisions.', source: `swarm Flock of twenty robots
model boids n=20 steps=800 dt=0.05 vmin=0.2 vmax=0.3 r_sep=0.15 r_view=0.6 w_sep=0.05 w_ali=5 w_coh=0.1 seed=4` },
  { id: 'pr2-vicsek', kind: 'swarm', part: 'ПР2', chapter: '5, 1', name: 'ПР2 · Vicsek phase transition (step 2)', summary: '100 particles on a 5×5 torus: order > 0.8 at η = 0.3, < 0.3 at η = 6 — order–disorder transition of a complex system.', source: `swarm Vicsek model on a torus
model vicsek n=100 box=5 radius=1 speed=0.03 steps=300 eta=0.3,1,2,3,4,5,6 seed=5` },
  { id: 'pr2-pso', kind: 'swarm', part: 'ПР2', chapter: '5', name: 'ПР2 · PSO with a ring topology finds the source (step 3)', summary: 'Twelve particles, lbest ring: the maximum of a Gaussian field is found to 0.02.', source: `swarm PSO lbest on a Gaussian field
model pso source=0.7,-0.4 sigma=0.5 n=12 steps=150 seed=8 range=-2,2` },
  { id: 'pr2-robots', kind: 'swarm', part: 'ПР2', chapter: '5', name: 'ПР2 · Robot swarm source seeking with noise and a distractor (step 4)', summary: 'Eight robots with a 3 cm step, repulsion and noisy measurements reach the source within 0.3 m despite a false local maximum.', source: `swarm Robots seek a gas source
model pso robots=true source=2,1.5 sigma=0.8 n=8 steps=400 seed=9 vmax=0.03 d_min=0.1 noise=0.005 range=0,0.5 forget=0.002
distractor at=0.8,1.6 sigma=0.3 amp=0.3` },
  { id: 'pr2-aco', kind: 'swarm', part: 'ПР2', chapter: '5', name: 'Chapter 5 · Ant colony optimisation of a patrol route (§5.5)', summary: 'Ten waypoints, ten ants: the pheromone-guided tour is at least as short as the nearest-neighbour heuristic.', source: `swarm Patrol route by ants
model aco n=10 ants=10 iterations=60 alpha=1 beta=3 rho=0.3 seed=1` },
  // ---------------------------------------------------------------- ПР3
  { id: 'pr3-ssi-cbba', kind: 'allocation', part: 'ПР3', chapter: '10, 9', name: 'ПР3 · Nine tasks for three robots: greedy, Hungarian, SSI, CBBA, Vickrey', summary: 'Random instance (seed 2): greedy vs optimal, the SSI rounds and bids, CBBA convergence and conflict-freeness, second-price payments.', source: `allocation Nine tasks for three robots
random robots=3 tasks=9 area=10 seed=2
methods greedy hungarian ssi cbba vickrey
cbba capacity=3 lambda=0.95 graph=complete` },
  { id: 'pr3-line', kind: 'allocation', part: 'ПР3', chapter: '10', name: 'ПР3 · CBBA over a line graph of four robots (step 4)', summary: 'Ten tasks, communication only along a chain: consensus takes more iterations but still ends conflict-free.', source: `allocation CBBA over a chain
random robots=4 tasks=10 area=10 seed=13
methods ssi cbba
cbba capacity=3 lambda=0.95 graph=path` },
  { id: 'pr3-course-matrix', kind: 'allocation', part: 'ПР3', chapter: '10', name: 'ПР3 · Greedy loses to the Hungarian assignment (step 1)', summary: 'Two robots, two tasks placed so that the cheapest first pair forces an expensive second one.', source: `allocation Greedy pays for its first choice
robot r1 at=0.05,0
robot r2 at=2,0
task t1 at=1,0 reward=10
task t2 at=-3,0 reward=10
methods greedy hungarian ssi` },
  // ---------------------------------------------------------------- ПР4
  { id: 'pr4-warehouse', kind: 'gridmapf', part: 'ПР4', chapter: '11', name: 'ПР4 · Four robots cross the warehouse grid (steps 1–5)', summary: 'Prioritized planning vs CBS on the 7×10 shelf grid, then ADG execution with 30 % random delays: no collisions.', source: `gridmapf Warehouse crossing
map
  ..........
  .##.##.##.
  ..........
  .##.##.##.
  ..........
  .##.##.##.
  ..........
end
agent a start=0,0 goal=6,9
agent b start=6,9 goal=0,0
agent c start=0,9 goal=6,0
agent d start=6,0 goal=0,9
methods prioritized cbs
execute delay=0.3 runs=5 seed=12` },
  { id: 'pr4-incomplete', kind: 'gridmapf', part: 'ПР4', chapter: '11', name: 'ПР4 · Prioritized planning is incomplete, CBS is not (steps 2, 4)', summary: 'Agent a parks on its goal in the corridor and walls in agent b; CBS finds the plan where a waits in the pocket.', source: `gridmapf Corridor with a pocket
map
  ....
  .#..
end
agent a start=0,1 goal=0,2
agent b start=0,0 goal=0,3
methods prioritized cbs
execute delay=0.2 runs=3 seed=1` },
  { id: 'pr4-random', kind: 'gridmapf', part: 'ПР4', chapter: '11', name: 'ПР4 · Six random agents with delays (step 5)', summary: 'Random starts and goals (seed 12): CBS plan executed through the action dependency graph, compared with clock-driven execution.', source: `gridmapf Six random agents
map
  ..........
  .##.##.##.
  ..........
  .##.##.##.
  ..........
  .##.##.##.
  ..........
end
random agents=6 seed=12
methods cbs
execute delay=0.3 runs=5 seed=12` },
  // ---------------------------------------------------------------- ПР5
  { id: 'pr5-lloyd', kind: 'coverage', part: 'ПР5', chapter: '11', name: 'ПР5 · Lloyd coverage of a hot spot (steps 1–2)', summary: 'Six robots start in a corner; the coverage functional falls monotonically by > 70 % and the robots gather around the Gaussian hot spot.', source: `coverage Hot spot at (3, 1)
area 0 4 0 4 res=0.1
density gaussian center=3,1 sigma=0.8 base=0.05
robots 6 seed=1 spawn=0,0.8
iterations 60` },
  { id: 'pr5-limited', kind: 'coverage', part: 'ПР5', chapter: '11', name: 'ПР5 · Limited-range Lloyd spreads the robots (step 3)', summary: 'Eight robots with r_sense = 0.8 on a uniform density: the limited cost decreases and the robots spread (pairwise > 0.5 m).', source: `coverage Uniform area, limited sensing
area 0 5 0 5 res=0.1
density uniform
robots 8 seed=2 spawn=2,3
iterations 80
range 0.8` },
  { id: 'pr5-estimation', kind: 'estimation', part: 'ПР5', chapter: '4, 11', name: 'ПР5 · Information consensus and covariance intersection (steps 4–5)', summary: 'Eight robots, unequal sensors: every estimate converges to the centralised one; plain averaging is worse; CI with ω = 0.5.', source: `estimation Target position by eight robots
target 2,-1
robots 8 seed=3 radius=0.55
noise 0.01 1.0
iterations 300
ci a=0,0 A=1,4 b=1,1 B=4,1` },
  // ---------------------------------------------------------------- ПР6
  { id: 'pr6-crossing', kind: 'safety', part: 'ПР6', chapter: '16, 13', name: 'ПР6 · Two robots cross safely (steps 1–3)', summary: 'Nominal go-to-goal collides; the barrier filter keeps 0.2 m and both reach their goals.', source: `safety Two robots cross
scenario crossing
params d_safe=0.2 gamma=2 vmax=0.3 dt=0.02 steps=1500 sense=1` },
  { id: 'pr6-antipodal', kind: 'safety', part: 'ПР6', chapter: '16, 13', name: 'ПР6 · Antipodal swap: deadlock and the keep-right rule (step 4)', summary: 'Eight robots swap places across a circle: the plain filter deadlocks, the hysteresis rule breaks the symmetry and everybody arrives.', source: `safety Antipodal swap of eight robots
scenario antipodal n=8 radius=1
params d_safe=0.2 gamma=2 vmax=0.3 dt=0.02 steps=2500 sense=1
unstuck angle=-45 hold=50` },
  { id: 'pr6-stale', kind: 'safety', part: 'ПР6', chapter: '16', name: 'ПР6 · Stale data about an uncooperative neighbour (step 5)', summary: 'Robot 2 lost the link and drives straight at robot 1 (which wants to stay put); robot 1 knows robot 2\'s position with a 0.5 s delay: the enlarged margin d + vmax·age and the full-responsibility constraint keep 0.3 m.', source: `safety Uncooperative neighbour known with delay
robot r1 at=0,0 goal=0,0
robot r2 at=1.2,0 goal=-3,0 speed=0.1
params d_safe=0.3 gamma=2 vmax=0.3 dt=0.02 steps=2500 sense=2 compare=false
stale lag=25 vmax_j=0.1
uncooperative 2` },
  // ---------------------------------------------------------------- ДЗ
  { id: 'hw-baseline', kind: 'warehouse', part: 'ДЗ', chapter: '10, 11', name: 'ДЗ · Warehouse fleet, baseline (stages 1–3)', summary: 'Four robots, six orders at start + Poisson arrivals, CBBA with commitment, executor FSM, cell reservation: throughput, latency, no path conflicts.', source: `warehouse Homework fleet: four robots, six stations
robots r1,r2,r3,r4 homes=H1,H2,H3,H4 speed=0.3 handle=3 radio=4 drop=0 latency=0.05
orders rate=0.05 reward=10 first=6 pickups=P1,P2,P3 drops=D1,D2,D3
traffic settle=0.3 stale=2 wait=6 lookahead=2 protective=true
cbba capacity=3 discount=0.98 period=0.5 stale=3 commit=2 lost=6
duration 600 dt=0.1 seed=1` },
  { id: 'hw-radio', kind: 'warehouse', part: 'ДЗ', chapter: '10, 16', name: 'ДЗ · Short radio range and packet loss (stage 2 experiments)', summary: 'Range 2.5 m and 30 % loss partition the fleet: conflicting claims (double commits) appear and latency grows.', source: `warehouse Fleet with a poor radio
robots r1,r2,r3,r4 homes=H1,H2,H3,H4 speed=0.3 handle=3 radio=2.5 drop=0.3 latency=0.05
orders rate=0.05 reward=10 first=6 pickups=P1,P2,P3 drops=D1,D2,D3
traffic settle=0.3 stale=2 wait=6 lookahead=2
cbba capacity=3 discount=0.98 period=0.5 stale=3 commit=2 lost=6
duration 600 dt=0.1 seed=1` },
  { id: 'hw-faults', kind: 'warehouse', part: 'ДЗ', chapter: '16', name: 'ДЗ · Robot failure and recovery (stage 4)', summary: 'r2 stops at 120 s and comes back at 400 s: its orders are reclaimed after lost_after, the protective layer keeps the others from driving into it.', source: `warehouse Fleet with a failing robot
robots r1,r2,r3,r4 homes=H1,H2,H3,H4 speed=0.3 handle=3 radio=4 drop=0 latency=0.05
orders rate=0.05 reward=10 first=6 pickups=P1,P2,P3 drops=D1,D2,D3
traffic settle=0.3 stale=2 wait=6 lookahead=2 protective=true
cbba capacity=3 discount=0.98 period=0.5 stale=3 commit=2 lost=6
fault r2 at=120 kind=stop
fault r2 at=400 kind=recover
duration 600 dt=0.1 seed=1` },
  // ---------------------------------------------------------------- architectures (chapter 3) over configured robots
  { id: 'arch-compare', kind: 'mission', part: 'architectures', chapter: '3, 4, 10, 16', name: 'Architectures · Form, move, allocate, gather, home — centralised vs decentralised vs hybrid', summary: 'The same five-phase mission with a robot failure at 40 s and a coordinator outage 60–110 s: the centralised group stalls during the outage, the decentralised one never depends on it, the hybrid one falls back and resynchronises.', source: `mission Form, move, allocate, return — three architectures
robots r1 r2 r3 r4 r5 r6 box=3 seed=1
architecture compare
comm radius=4 drop=0.05 period=0.5 lost=3 settle=1
coordinator at=0,0 range=12 fail=60 recover=110
phase form circle r=1.2
phase goto at=6,0 speed=0.3
phase allocate targets=8,2;9,-1;7,-3;10,1;8,-2;9,3
phase gather
phase home
safety d_safe=0.4 gamma=2 sense=1.5
obstacle at=3,0.5 r=0.6
fail r4 at=40
duration 300 dt=0.1 seed=1 vmax=0.5` },
  { id: 'arch-partition', kind: 'mission', part: 'architectures', chapter: '3, 4', name: 'Architectures · Short radio: robots beyond the coordinator', summary: 'Radio 3.5 m with 20 % loss, coordinator range 3 m on a wide start: the robots the coordinator cannot reach never move in the centralised variant; the decentralised group forms and gathers on its own; the hybrid group forms with the far robots in fallback.', source: `mission Robots beyond the coordinator's range
robots r1 r2 r3 r4 r5 r6 box=4 seed=2
architecture compare
comm radius=3.5 drop=0.2 period=0.5 lost=3 settle=1
coordinator at=0,0 range=3
phase form circle r=1.5
phase gather
phase home
safety d_safe=0.4 gamma=2 sense=1.5
duration 240 dt=0.1 seed=2 vmax=0.5` },
  { id: 'arch-station', kind: 'mission', part: 'architectures', chapter: '3, 10', name: 'Architectures · Station mission over configured robots (zones, no-go, supervisor)', summary: 'For Run on fleet: phases refer to station zones (goto zone=Dock, allocate zones=…), no-go zones become obstacles, a des supervisor named "Fleet supervisor" gates the phases and a hybrid model "Fleet modes" limits the speed near a human.', source: `mission Station mission (hybrid)
robots r1 r2 r3 r4
architecture hybrid
comm radius=6 drop=0 period=0.5 lost=3 settle=1
coordinator at=0,0 range=30
phase form wedge r=1.0
phase goto zone=Dock at=6,0 speed=0.3            # zone on the station; at= for the analysis
phase allocate zones="Shelf A,Shelf B,Shelf C,Shelf D" targets=9,2;9,-2;11,2;11,-2
phase gather
phase home
safety d_safe=0.5 gamma=2 sense=2
supervisor Fleet supervisor
modes Fleet modes
duration 400 dt=0.1 seed=2 vmax=0.6` },
  // ---------------------------------------------------------------- chapters
  { id: 'ch9-game', kind: 'game', part: 'ch. 9', chapter: '9', name: 'Chapter 9 · Task allocation game and a cooperative game (§9.3.4, §9.5.3)', summary: 'Pure equilibria (Z1, Z2), (Z2, Z1), mixed q = 0.8 with payoff 5.2; Shapley value (26.67, 41.67, 51.67) in the core.', source: `game Task allocation between two robots (§9.3.4)
players R1 R2
strategies Z1 Z2 ; Z1 Z2
payoff Z1 Z1 4 4
payoff Z1 Z2 10 6
payoff Z2 Z1 6 10
payoff Z2 Z2 2 2
start Z1 Z1
rounds 500
coalition A=10 B=20 C=30 AB=60 AC=70 BC=90 ABC=120` },
  { id: 'ch9-dilemma', kind: 'game', part: 'ch. 9', chapter: '9', name: 'Chapter 9 · Prisoner\'s dilemma of two robots (§9.4.1)', summary: 'Defection dominates, the unique equilibrium is Pareto-dominated by mutual cooperation; replicator dynamics drive cooperation out.', source: `game Prisoner's dilemma: share the charger or hog it
players R1 R2
strategies C D ; C D
payoff C C 3 3
payoff C D 0 5
payoff D C 5 0
payoff D D 1 1
replicator 0.9 0.1` },
  { id: 'ch7-qlearning', kind: 'marl', part: 'ch. 7', chapter: '7', name: 'Chapter 7 · Q-learning in the corridor and two independent learners (§7.2.4, §7.4.2)', summary: 'Table 7.2 reproduced (Q(s₃) = 5, 7.5, 8.75; optimum 6.2 / 8 / 10); independent Q-learning of two robots reduces collisions.', source: `marl Corridor Q-learning (§7.2.4)
map
  S..G
end
params alpha=0.5 gamma=0.9 epsilon=0.1 episodes=300 step=-1 goal=10 seed=1 path=right
agents 2 starts=0,0;0,3 goals=0,3;0,0 penalty=-5 episodes=600` },
  { id: 'ch7-grid', kind: 'marl', part: 'ch. 7', chapter: '7', name: 'Chapter 7 · Q-learning on a 4×5 grid with walls', summary: 'ε-greedy Q-learning learns a shortest path around the wall; independent learners on the same grid.', source: `marl Grid world with a wall
map
  S....
  .##..
  .#G..
  .....
end
params alpha=0.3 gamma=0.95 epsilon=0.15 episodes=500 step=-1 goal=20 seed=3
agents 2 starts=0,0;3,4 goals=3,4;0,0 penalty=-5 episodes=800` },
  { id: 'ch12-evo', kind: 'evo', part: 'ch. 12', chapter: '12', name: 'Chapter 12 · GA for x² (§12.2.6), DE on Rastrigin, ES tuning the consensus step', summary: 'The roulette table and the first generation of the text; DE reaches the Rastrigin optimum; the ES finds ε* = 2/(λ₂ + λₙ).', source: `evo Genetic algorithm for x² and continuous tuning
method ga bits=5 pop=4 generations=30 pc=0.9 pm=0.05 elitism=true seed=4 initial=01101,11000,01000,10011 fitness="x*x"
method de benchmark=rastrigin dim=2 pop=20 generations=150 seed=5
method es objective=consensus graph=path n=6 generations=40` },
  { id: 'ch4-ca', kind: 'ca', part: 'ch. 4', chapter: '4 §4.6, 5', name: 'Chapter 4 · Rule 90, rule 30, a glider and a pheromone trail (§4.6, §5.5)', summary: 'Sierpiński triangle vs chaos, the glider period 4 / shift (1, 1), stigmergy field with evaporation and diffusion.', source: `ca Rule 90, a glider and a pheromone trail
wolfram rule=90 width=31 steps=15
life steps=12 size=10
  .#.
  ..#
  ###
end
pheromone size=12 steps=40 rho=0.05 kappa=0.1` },
  { id: 'ch4-rule30', kind: 'ca', part: 'ch. 4', chapter: '4 §4.6', name: 'Chapter 4 · Rule 30: aperiodic pattern', summary: 'The same single-cell start under rule 30 gives an asymmetric, chaotic pattern.', source: `ca Rule 30
wolfram rule=30 width=41 steps=20` },
  { id: 'ch13-fuzzy', kind: 'fuzzy', part: 'ch. 13', chapter: '13 §13.4', name: 'Chapter 13 · Fuzzy obstacle avoidance (§13.4.2)', summary: 'd_L = 2.0, d_F = 0.7, d_R = 1.6: rules 5 and 2 fire at 0.4 and 0.3, the centroid gives a slow-ish left turn; closed loop passes three obstacles.', source: `fuzzy Obstacle avoidance (§13.4.2)
input dL=2.0 dF=0.7 dR=1.6
obstacles 3,0 3,0.4 3,-0.4 goal=6,0` },
  { id: 'ch16-resilience', kind: 'resilience', part: 'ch. 16', chapter: '16, 13 §13.5', name: 'Chapter 16 · Four agents with one traitor, trust, degradation, switching (§16.8.2, §16.5.3, §13.5.1)', summary: 'Honest agents agree despite D; reputation filters an alternating liar; a star loses connectivity when its hub fails; two stable modes destabilise under fast switching.', source: `resilience Byzantine agents, trust, degradation, switching
agent A continue
agent B continue
agent C continue
agent D continue lies A=continue B=evacuate C=evacuate
trust graph=complete n=6 x0=0.1,0.4,0.2,0.9,0.5,0 liar=6 eps=0.1 steps=200
degrade graph=star n=5 order=1,2
switched A1=-0.1,1,-10,-0.1 A2=-0.1,10,-1,-0.1 tau=0.05,3` },
  { id: 'ch16-three', kind: 'resilience', part: 'ch. 16', chapter: '16', name: 'Chapter 16 · Three agents cannot tolerate one traitor (n < 3f + 1)', summary: 'With n = 3 the evidence of the honest agents is a tie: agreement is not guaranteed.', source: `resilience Three agents, one traitor
agent A continue
agent B evacuate
agent C continue lies A=continue B=evacuate` },
];
export const MRS_PARTS = ['ПР1', 'ПР2', 'ПР3', 'ПР4', 'ПР5', 'ПР6', 'ДЗ', 'architectures', 'ch. 4', 'ch. 7', 'ch. 9', 'ch. 12', 'ch. 13', 'ch. 16'];
