# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Simulation of decentralized swarm-robot lattice formation: robots self-organize into
repeating 2D tilings (square, hexagon, octagon-square, snub square, etc.) by passing
messages and building "cycles" that correspond to the faces of the target lattice, with
no central coordinator and no global knowledge — each robot only uses local
observations and messages from neighbors.

## Build system

Multi-module Gradle project (Kotlin DSL), Java 21 toolchain. Modules, declared in
`settings.gradle.kts`:

- **Graphs** — a generic, unrelated `Graph<T>`/`Node`/`Edge`/`Tree` toy library. Not used
  by the other two modules (do not confuse its `org.graphs` package with
  `Lattice-Robots`' own, unrelated `org.graphs` package).
- **Transformations** — a small standalone 2D transform/robot-panel scratch module.
  Declared as a dependency of `Lattice-Robots` in its `build.gradle.kts`, but nothing in
  `Lattice-Robots` currently imports `org.transformations` — it's dead weight, not a
  shared library in active use.
- **Lattice-Robots** — the actual simulation. Everything of substance lives here.

Common commands (run from repo root):

```
./gradlew.bat build                          # build everything
./gradlew.bat :Lattice-Robots:run             # launch the async simulation panel (Swing GUI)
./gradlew.bat :Lattice-Robots:compileJava     # compile only
```

### Tests — read this before assuming `gradle test` covers anything

JUnit tests in `Lattice-Robots` (`VoltageGraphBuilderTest`, `VoltageGraphsTest`,
`HungarianAlgoTest`) live under `src/main/java`, not `src/test/java`, and the module's
`build.gradle.kts` declares `junit-jupiter` as `implementation`, not
`testImplementation`. This is intentional-but-unusual: it makes the tests
IDE-discoverable (VS Code's Java Test Runner scans the whole classpath) without setting
up a `src/test` tree for this module. The practical consequence: `./gradlew.bat
:Lattice-Robots:test` reports `NO-SOURCE` and runs nothing — it is not a signal that
tests pass. To actually run these tests, use the IDE's test runner/explorer, or invoke
the JUnit Console Launcher directly against the compiled `main` classes. The `Graphs`
module has an (empty) `src/test` tree and also runs nothing via `gradle test`.

`HungarianAlgo` additionally has a hand-rolled `HungarianAlgoTestRunner` with a
`main()` that runs the same test methods manually via reflection, for use without any
test framework wiring at all.

## Architecture (Lattice-Robots)

### The core idea: role graphs, not robot graphs

The target formation is never represented as a graph of *robots* — it's a small,
finite graph of **roles** (`Role`), each labeled with a local pose, connected by
directed edges (`HalfEdge`) labeled with the rigid-body transform to a neighboring
role. Every robot in the swarm, once assigned, adopts one of these finitely many roles.
Because the transform between two roles is the same everywhere in the lattice, this
finite "quotient" graph is enough to describe an unboundedly large repeating tiling —
this is a **voltage graph** (topological graph theory term); the infinite physical
lattice is its "derived/covering graph." See
`Lattice-Robots/src/main/Documents/Graphs.md` and
`Lattice-Robots/src/main/Documents/DCEL-Implementation-Plan.md` for the full
mathematical motivation (the second doc's pseudocode is what `org.graphs.voltage` is
the realized version of — package name differs from the doc's proposed `org.graphs`,
everything else matches).

### `org.graphs.voltage` — the graph representation

- `Role` — a role vertex: `(id, pose)`.
- `HalfEdge` — a directed edge from one role to another, carrying its `voltage`
  (a `RigidBodyTransformation`) plus DCEL-style `twin`/`next`/`prev`/`face` pointers,
  all resolved **once**, at construction time — never inferred at lookup time.
- `Face` — a `next`-orbit of half-edges; its `cycleLength` is the number of hops
  around it (4 for a square face, 8 for an octagon face, etc).
- `VoltageGraph` — owns the finite sets of roles/half-edges/faces for one lattice type,
  plus `validateCycle(walk)`: a candidate robot chain is a real graph cycle iff its
  accumulated voltage (product of transforms) is within epsilon of identity **and**
  its length matches some face's `cycleLength` (length alone isn't sufficient — a
  role can have fewer outgoing edges than the faces it borders are long, e.g.
  octagon-square, so a walk can revisit half-edges without closing).
- `VoltageGraphBuilder` — declares roles and half-edge pairs (`addHalfEdgePair` — a
  single call registers both a half-edge and its twin), optionally overrides a role's
  rotation order (`setRotationOrder` — needed whenever a role is both endpoints of one
  of its own edges, e.g. a single-role square lattice; not needed when insertion order
  already matches, e.g. `HexagonVoltageGraph`'s two roles), then `build()` resolves
  `next`/`prev` via Edmonds' rotation-system rule and discovers faces by walking
  `next`-orbits, throwing if a face's holonomy doesn't converge to identity within
  `MAX_LAPS` (a malformed lattice declaration fails fast here, not silently).
- Concrete lattices (`HexagonVoltageGraph`, `SquareVoltageGraph`,
  `OctagonSquareVoltageGraph`, `SnubSquareVoltageGraph`, `SnubHexagonVoltageGraph`,
  `HexagonTriangleVoltageGraph`, `HexagonSquareTriangleVoltageGraph`,
  `DodecagonTriangleVoltageGraph`, `DodecagonHexagonSquareVoltageGraph`,
  `ElongatedTriangularVoltageGraph`) are each a small static `build()` factory using the
  builder. Every edge is inserted in clockwise order so every face traces clockwise —
  keep new lattices consistent with this convention.
- `org.graphs.lattice` (`LatticeGraph`, `HexagonLattice`, `SquareLattice`, `Vertex`,
  `LatticeEdge`) is the **legacy** pre-voltage-graph representation described as
  "Current Architecture" in `Graphs.md`. It's superseded by `org.graphs.voltage` and
  not used by `GeometricCycleLatticeRobot`/`CyclebuilderComms` — don't wire new code to
  it; it's kept around for reference/history, not as an active abstraction.

### `org.communicationModels.CyclebuilderComms` — the decentralized algorithm

One `CyclebuilderComms` per robot; drives a state machine over `CycleRole`
(`unassigned` → `cycleBuilder` → eventually `root`/`stable`), driven entirely by
message passing (`org.communicationModels.Messages`: `PositioningMessage`,
`PromotionMessage`, `StatusMessage`, `RejectAssignmentMessage`, `ChainMemberList`).
Robots reason only about their own assigned `HalfEdge` and observations of physically
nearby neighbors (`Observation`, populated from `GeometricCycleLatticeRobot.getNeighbors()`)
— never global state. Key methods to read together when touching this class:
`processMessages`/`broadcastMessage` (the per-tick state transition + action),
`inferNextEdge` (just `graph.getNext(assignedEdge)` — the rotation-order math is baked
in once at graph-build time, not recomputed here), and `findBestNeighborForEdge`
(nearest-neighbor matching of a target `HalfEdge`'s voltage against live observations).
`completedCycles` tracks, per outgoing edge id from the root, whether that face's cycle
has closed.

### `org.robots` / `org.motionModels` / `org.drawingModels`

`GeometricCycleLatticeRobot` (extends `Robot`) is the simulated agent: owns a
`CyclebuilderComms`, a `LatticeMotionModel` (diff-drive-style movement toward an
assigned pose), and a `TriangularModel` for rendering. `GRAPH` is a `static final
VoltageGraph` shared by all robots of a run — swap which lattice's `build()` is
assigned here to simulate a different tessellation. `executeTimeStep(dt, tick)` is the
per-activation entry point and returns a full `TickRecord` (before/after
`CommsSnapshot`, pose delta, messages sent) for logging — role dispatch (root/stable
process-then-broadcast vs. cycleBuilder/unassigned's process-then-move-then-broadcast
ordering) intentionally lives here rather than in the simulation panel.

### `org.simulation.AsynchSim.AsyncRobotPanel` — the runnable simulation

Swing app; each robot activates on its own `ScheduledThreadPoolExecutor` task at a
shared period but staggered initial delay (`i/n * period`) for reproducible asynchrony
without randomness. A single `ReentrantReadWriteLock` guards all shared robot state:
the periodic proximity-check task and UI drag/load/save hold the **write** lock; robot
tick tasks (running concurrently with each other) and the render loop hold the **read**
lock. Motion (`robot.move(dt)`) runs on its own 30fps task decoupled from the
logic-tick rate. Persistence is JSON via `org.utils.RobotDataIO`
(`output/robot_data/robot_data.json`) — note neighbor/edge lists are visualization
state only and are *not* restored on import; `runProximityCheck()` must be called after
loading to rebuild live neighbor relationships from imported positions. Keyboard: Space
play/pause, → step, J save, K load, T stats overlay, D proximity overlay, S screenshot
(written to `output/robot_panel_images`).

### Geometry primitives

`org.graphs.util.OrientedPoint` (x, y, orientation) and
`org.graphs.util.RigidBodyTransformation` (2D homogeneous transform backed by a Jama
`Matrix`) are the shared geometry types used throughout — transforms compose via
`compose()` (matrix multiply, order matters: `a.compose(b)` applies `b` in `a`'s target
frame), and `isApproximatelyIdentity(epsilon)` underlies both cycle-closure detection
(`CyclebuilderComms`) and face-discovery (`VoltageGraphBuilder`).

### `com.darcarms.htmllog`

Self-contained HTML debug-log library (headings, grouped sections, Java2D canvas
snapshots, exception traces) — general-purpose, not swarm-specific. Used by
`org.utils.logging.SimulationLogger` to produce a per-run, timestamped HTML tick log
(path printed to stdout on `AsyncRobotPanel` startup).
