# DCEL Implementation Plan

*Follow-up to `Graphs.md`, grounded in the current `org.graphs` / `org.communicationModels` code.*

## 0. Where this plugs into the existing system

`Graphs.md` motivates the DCEL in the abstract. This document maps it onto the
concrete classes that will have to change:

| Current (Lattice Graph)                         | Role                                                              | Replaced by                        |
|---------------------------------------------------|--------------------------------------------------------------------|-------------------------------------|
| `org.graphs.Vertex`                                | role vertex, immutable `(id, pose)`                                 | `Role`                              |
| `org.graphs.LatticeEdge`                           | directed transform between roles, per-vertex-local id               | `HalfEdge`                          |
| `org.graphs.LatticeGraph` (+ `HexagonLattice`, `SquareLattice`) | `HashMap<Vertex, ArrayList<LatticeEdge>>`, `getPrimaryVertex()` = `keySet().toArray()[0]` | `LatticeDCEL`                       |
| `CyclebuilderComms.inferNextEdge(...)`             | hard-coded "same id if leaving role 1, else id+1 mod 3" rule for hex lattices only | `dcel.next(halfEdge)` — O(1) field lookup |
| `CyclebuilderComms.getCurrentVertex()`             | `assignedEdge.getTo()`                                               | `assignedHalfEdge.target()` (unchanged in spirit) |
| No face concept                                    | —                                                                     | `Face`, with explicit `cycleLength` |
| Cycle closure = "next best neighbor id == chain root id" | implicit, geometric                                            | closure = **holonomy check** (see §2.3) |

The rest of `CyclebuilderComms` (message passing, `ChainMemberList`, role
promotion, conflict resolution) does **not** need to change — it only needs a
different object to ask "what's my current edge" and "what's next."

---

## 1. Why a plain DCEL isn't quite enough here

A textbook DCEL describes a **finite, fully-instantiated** planar subdivision:
every vertex, half-edge, and face is a distinct stored object. Our lattice is
**infinite and repeating** — exactly like the current Lattice Graph, we can
only afford to store one instance per *role*, not per physical position.

The right formalization for "a small finite graph whose edges are labeled
with group elements, representing an infinite periodic structure" already
exists in topological graph theory: a **voltage graph**, whose **derived
graph** (or *covering graph*) is the infinite lattice we actually build in
the field. This is the same machinery used to construct genus embeddings and
periodic tilings/meshes (cf. CGAL's periodic triangulations). Framing the
role graph this way gives us, for free, precise definitions for every
Design Requirement in `Graphs.md`:

- **Requirement 2 (deterministic traversal)** falls out of the DCEL's
  `next`/`twin` pointers, computed once at construction time from a
  **rotation system**, not inferred per-lattice at runtime.
- **Requirement 3 (cycle validation)** falls out of **voltage / holonomy**:
  a closed walk in the role graph lifts to a closed cycle in the real lattice
  **iff** the product of its edge transforms is the identity.
- **Requirement 1/4 (asymmetric lattices, explicit faces)** fall out of
  allowing multiple `Role`s and multiple `Face` types with different
  `cycleLength`, instead of one hard-coded id-rotation rule.
- **Requirement 5 (decentralization)** is preserved because the role graph
  stays small and finite — a robot only ever touches its one assigned
  half-edge and that half-edge's `O(1)` `twin`/`next` pointers, never the
  (unbounded) physical lattice.

---

## 2. Mathematical Framework

### 2.1 DCEL primitives

A half-edge structure is a triple of finite sets `(V, H, F)` with maps

```
origin      : H -> V
twin        : H -> H         (involution, no fixed points: twin(twin(h)) = h)
next, prev  : H -> H         (mutually inverse permutations of H)
incidentFace: H -> F
```

such that `next`'s orbits partition `H` into faces, and `twin` reverses
orientation (`origin(twin(h))` is the "head" of `h`).

### 2.2 Rotation systems (how `next` is actually computed)

By Edmonds' theorem, a combinatorial embedding is equivalent to a
**rotation system**: a cyclic ordering `σ_v` of the half-edges leaving each
vertex `v` (i.e., the CCW order neighbors appear in, physically). Given that,
face traversal has a closed-form definition:

```
next(h) = σ_{origin(twin(h))}( twin(h) )
```

In words: *to continue around a face after traversing `h`, cross to the twin
`twin(h)`, then take the next outgoing edge in rotation order at the vertex
you just arrived at.*

This is the formal version of the ad hoc rule currently hard-coded in
`CyclebuilderComms.inferNextEdge`:

```java
int nextId = (incomingFromType.getId() == 1)
        ? incomingId
        : (incomingId % candidateEdges.size()) + 1;
```

That method is literally trying to compute `σ_v(twin(h))` for the
2-role hex lattice by guessing an id arithmetic rule. It works only because
the hex lattice happens to have one face type and a uniform rotation. The
DCEL replaces this "guess an id formula" approach by **precomputing `next`
once, per role, at graph-construction time**, so it generalizes to any
rotation system (any number of roles, any mix of face sizes) with zero
runtime inference.

### 2.3 The lattice as a voltage graph (handles the "infinite" part)

Let `Γ` be the group of rigid-body motions that map the lattice onto itself
(its symmetry/wallpaper group — translations, and rotations for multi-role
lattices like hex). The **role graph** is the quotient of the infinite
lattice by `Γ`: finitely many role vertices, finitely many half-edges.

Label each half-edge `h` with a **voltage** `ψ(h) ∈ Γ`: the rigid-body
transform (already exactly what `LatticeEdge`/`RigidBodyTransformation`
store) that carries the physical instance of `origin(h)` to the physical
instance of the vertex at the far end of `h`. `ψ(twin(h)) = ψ(h)⁻¹`.

This is precisely a **voltage graph**; the physical infinite lattice is its
**derived graph** (the covering space obtained by "unrolling" every role
into all its physical copies, connecting copy `p` of `u` to copy `ψ(h)·p` of
`v` for each edge `u→v`).

**Holonomy / cycle validation (Design Requirement 3).** A closed walk
`h_1, h_2, ..., h_k` in the role graph (i.e. `target(h_i) = origin(h_{i+1})`,
`target(h_k) = origin(h_1)`) lifts to an actual closed physical cycle **iff**
its **holonomy**

```
Hol(walk) = ψ(h_1) ∘ ψ(h_2) ∘ ... ∘ ψ(h_k)
```

is the identity transform. This is a standard fact about voltage graphs and
gives us a cheap, exact test: a `GeometricCycleLatticeRobot` chain is a valid
graph cycle iff (a) it has the length of some `Face.cycleLength`, and (b) the
composition of the `RigidBodyTransformation`s it traversed is within
tolerance of identity — replacing the current "does the next-best-neighbor
id equal my chain's root id" heuristic in
`CyclebuilderComms.findBestNeighborForEdge`/`forwardSuccessUpstream` with an
exact geometric check.

`Face`s of the role-DCEL are the `next`-orbits of `H`; a face's
`cycleLength` is the orbit size, and by construction the holonomy around any
face orbit is the identity (that's what makes it a valid face rather than an
open spiral) — so faces double as the canonical "valid cycle" templates that
Design Requirement 3 asks robots to check against.

---

## 3. Data Model (pseudocode)

This mirrors the existing `org.graphs` package structure and naming so the
migration is mechanical. **Pseudocode — meant to be argued with.**

```java
package org.graphs;

// Replaces Vertex. Same immutable id+pose value object; unchanged in spirit.
public final class Role {
    private final int id;
    private final OrientedPoint poseInGraph;

    public int id();
    public OrientedPoint pose();
}

// Replaces LatticeEdge. A directed half-edge from one Role instance to another.
public final class HalfEdge {
    private final int id;                    // globally unique within the DCEL (fixes the
                                              // old per-vertex-local id collision issue)
    private final Role origin;
    private final RigidBodyTransformation voltage; // psi(h): origin-frame -> target-frame

    // Structural pointers, all resolved ONCE at construction time (§2.2):
    private HalfEdge twin;   // psi(twin) == voltage.inverse()
    private HalfEdge next;   // sigma_{target}(twin(this))  -- next edge around this half-edge's face
    private HalfEdge prev;   // inverse of next
    private Face face;       // the face this half-edge bounds

    public int id();
    public Role origin();
    public Role target();               // = twin.origin()
    public RigidBodyTransformation voltage();
    public HalfEdge twin();
    public HalfEdge next();
    public HalfEdge prev();
    public Face face();
}

// New: explicit face representation (Design Requirement 4).
public final class Face {
    private final int id;
    private final HalfEdge representative;  // any one half-edge on the boundary
    private final int cycleLength;          // size of the next-orbit; e.g. 4 for a square
                                              // face, 8 for an octagon face

    public List<HalfEdge> boundary();        // walk next() cycleLength times from representative
    public RigidBodyTransformation holonomy(); // should be ~identity; assert this at construction
                                                // as a self-check (see Task 9)
}

// Replaces LatticeGraph. Owns the finite role/half-edge/face sets.
public final class LatticeDCEL {
    private final List<Role> roles;
    private final Map<Integer, HalfEdge> halfEdgesById;
    private final Map<Role, List<HalfEdge>> outgoing;   // same shape as old LatticeGraph.edges
    private final List<Face> faces;
    private final Role primaryRole;   // EXPLICIT now -- fixes the old
                                       // "edges.keySet().toArray()[0]" arbitrary-order bug

    public Role primaryRole();
    public List<HalfEdge> outgoingHalfEdges(Role r);
    public HalfEdge halfEdgeById(int id);

    public HalfEdge twin(HalfEdge h)  { return h.twin(); }
    public HalfEdge next(HalfEdge h)  { return h.next(); }   // O(1) -- replaces inferNextEdge()

    // Design Requirement 3: verify a physically-realized robot chain corresponds
    // to a real graph cycle.
    public boolean validateCycle(List<HalfEdge> walk) {
        if (walk.isEmpty()) return false;
        RigidBodyTransformation hol = RigidBodyTransformation.identity();
        for (HalfEdge h : walk) hol = hol.compose(h.voltage());
        return hol.isApproximatelyIdentity(EPSILON)
            && faces.stream().anyMatch(f -> f.cycleLength() == walk.size());
    }
}

// Builder: separates "declare roles + template half-edges" (design time)
// from "resolve twin/next/face pointers" (one-time preprocessing), so a new
// lattice is defined declaratively instead of hand-writing rotation logic
// per lattice type like HexagonLattice does today.
public final class LatticeDCELBuilder {
    public LatticeDCELBuilder addRole(int id, OrientedPoint pose);
    public LatticeDCELBuilder addHalfEdgePair(
        int roleFromId, int roleToId, OrientedPoint toPoseRelativeToFrom);
        // registers h and its twin in one call; voltage(twin) is derived automatically

    public LatticeDCELBuilder setPrimaryRole(int id);

    // Resolves next/prev/face for every half-edge from the rotation order in
    // which addHalfEdgePair calls were made per role, then walks next-orbits
    // to materialize Face objects and asserts each face's holonomy ~= identity.
    public LatticeDCEL build();
}
```

---

## 4. Integration with `GeometricCycleLatticeRobot` / `CyclebuilderComms`

Only the graph-facing lines of `CyclebuilderComms` change; message flow,
`ChainMemberList`, role promotion, and conflict resolution are untouched.

```java
// BEFORE
private static final LatticeGraph graph = new HexagonLattice();

private LatticeEdge inferNextEdge(LatticeEdge assignedEdge) {
    if (assignedEdge.isNull()) return null;
    Vertex currentVertex = getCurrentVertex();
    Vertex incomingFromType = assignedEdge.getFrom();
    ArrayList<LatticeEdge> candidateEdges = graph.getOutgoingEdges(currentVertex);
    if (candidateEdges.isEmpty()) return null;
    int incomingId = assignedEdge.getId();
    int nextId = (incomingFromType.getId() == 1)
            ? incomingId
            : (incomingId % candidateEdges.size()) + 1;
    for (LatticeEdge edge : candidateEdges) if (edge.getId() == nextId) return edge;
    return null;
}

private Vertex getCurrentVertex() {
    LatticeEdge assignedEdge = getAssignedEdge();
    return assignedEdge.isNull() ? graph.getPrimaryVertex() : assignedEdge.getTo();
}
```

```java
// AFTER
private static final LatticeDCEL graph = HexagonDCEL.build(); // or OctagonSquareDCEL.build()

private HalfEdge inferNextEdge(HalfEdge assignedEdge) {
    if (assignedEdge == null) return null;
    // h.next() already IS sigma_{target}(twin(h)) -- the rotation-system rule
    // from §2.2 is baked in once, at construction time, not recomputed here.
    // (Calling graph.twin() again on top of it would apply the rule twice.)
    return graph.next(assignedEdge);
}

private Role getCurrentRole() {
    HalfEdge assignedEdge = getAssignedHalfEdge();
    return assignedEdge == null ? graph.primaryRole() : assignedEdge.target();
}
```

Cycle-closure detection (`forwardSuccessUpstream` / the "is the next best
neighbor actually my root" check in `findBestNeighborForEdge`) gets replaced
by an explicit call once a chain candidate closes geometrically:

```java
// AFTER, in the closure-detection path
List<HalfEdge> traversedWalk = chainMemberList.toHalfEdgeWalk();  // new helper, one entry
                                                                     // per hop already recorded
if (graph.validateCycle(traversedWalk)) {
    forwardSuccessUpstream();
} else {
    // reject / retry -- replaces the current root-ID-equality heuristic
}
```

`completedCycles` (currently `HashMap<Integer /*edge id*/, Boolean>`,
keyed off the primary vertex's outgoing edges) becomes keyed off
`Face.id()` instead of a raw edge id — it now tracks "which faces incident
to the primary role have a completed perimeter," which generalizes cleanly
once a lattice (like octagon-square) has more than one face type meeting at
a role.

The `isNull()` sentinel pattern (`new LatticeEdge()` as a null-object) can be
dropped in favor of `HalfEdge` / `Role` being ordinary nullable references —
worth a scrutiny point, see §6.

---

## 5. Worked example: octagon-square lattice (proves Requirement 1)

This is the case `Graphs.md` calls out as the one the current representation
can't handle (two face types, cycle lengths 4 and 8, meeting at every
vertex). Sketch of the declarative construction:

```java
public final class OctagonSquareDCEL {
    public static LatticeDCEL build() {
        LatticeDCELBuilder b = new LatticeDCELBuilder();
        b.addRole(0, ORIGIN);                 // single role: the lattice is vertex-transitive
        b.setPrimaryRole(0);

        // Each vertex has 3 outgoing edges (degree 3): two toward octagon
        // faces, one toward the square face. Rotation order = insertion order.
        b.addHalfEdgePair(0, 0, EDGE_A);       // -> next octagon corner
        b.addHalfEdgePair(0, 0, EDGE_B);       // -> square corner
        b.addHalfEdgePair(0, 0, EDGE_C);       // -> next octagon corner (other side)

        return b.build();
        // build() walks next-orbits and should discover:
        //   - orbits of length 8 (octagon faces), holonomy ~= identity
        //   - orbits of length 4 (square faces), holonomy ~= identity
        // If it instead discovers a single non-closing orbit, the declared
        // EDGE_A/B/C poses are geometrically inconsistent -- build() should
        // throw, which is a real correctness check the current
        // LatticeGraph has no way to express at all.
    }
}
```

The key point to scrutinize: with the old representation this lattice is
*inexpressible* (id-rotation arithmetic can't distinguish an 8-cycle from a
4-cycle sharing a vertex). With the DCEL it's just "declare the local
rotation order once"; correctness of both face types is verified
automatically by `build()`'s holonomy check rather than trusted by
inspection.

---

## 6. Implementation Task List

1. **Core primitives** — add `Role`, `HalfEdge`, `Face`, `LatticeDCEL`,
   `LatticeDCELBuilder` to `org.graphs`, per §3. `RigidBodyTransformation`
   needs `identity()`, `compose(RigidBodyTransformation)`, and
   `isApproximatelyIdentity(double eps)` added (only `inverse()`/`apply()`/
   `isInverse()` exist today).
2. **Builder + rotation/face resolution** — implement
   `LatticeDCELBuilder.build()`: wire `twin` pairs, derive `next`/`prev` via
   §2.2's `next(h) = σ_target(twin(h))` rule from per-role insertion order,
   walk `next`-orbits into `Face` objects, and assert every face's holonomy
   is within tolerance of identity (fail fast on a malformed declaration —
   see §5).
3. **Migrate `SquareLattice` and `HexagonLattice`** to
   `LatticeDCELBuilder`-based declarations. Add a golden/characterization
   test asserting the new `graph.next(h)` sequence matches what
   `CyclebuilderComms.inferNextEdge` currently produces for every edge, for
   both lattices — this is the regression safety net for the swap.
4. **Add `OctagonSquareDCEL`** (§5) as new-capability coverage: this is the
   concrete proof that Design Requirement 1 is met, since it cannot be
   expressed in `LatticeGraph` today.
5. **Update `CyclebuilderComms`**: swap `LatticeGraph`/`Vertex`/`LatticeEdge`
   fields for `LatticeDCEL`/`Role`/`HalfEdge`; replace `inferNextEdge` and
   `getCurrentVertex` per §4; keep `assignedVertexID`/`assignedOutgoingEdgeID`
   or collapse them to a single `assignedHalfEdgeId` now that ids are
   globally unique (design choice — see §7).
6. **Cycle validation** — add `ChainMemberList.toHalfEdgeWalk()` (or
   equivalent) and route chain-closure decisions through
   `graph.validateCycle(...)` instead of the current root-ID-equality
   check, per §4.
7. **`completedCycles` rework** — rekey from `outgoing-edge-id -> boolean`
   to `face-id -> boolean` so `initializeEdgeMap()` /
   `promoteAdjacentVerticesToRoots()` generalize to roles bordering more
   than one face type.
8. **Update `GeometricCycleLatticeRobot`** call sites (`getRole()`-adjacent
   plumbing is untouched; only the two-method surface it delegates through
   changes, per §4).
9. **Tests** — the module currently has zero tests for this subsystem
   (`HungarianAlgoTest` is the only JUnit test under `src/main`, and it's
   unrelated). At minimum add:
   - DCEL invariants: `twin(twin(h)) == h`, `next` cycles back to start
     after exactly `face.cycleLength()` steps, every face's holonomy
     `isApproximatelyIdentity`.
   - Behavioral equivalence tests from Task 3.
   - `OctagonSquareDCEL` face-discovery test (finds one 4-cycle orbit type
     and one 8-cycle orbit type).
   - A `CyclebuilderComms` integration test exercising a full cycle build
     on `SquareDCEL` end-to-end (currently impossible to regression-test at
     all, since there's no test scaffolding for this class).
10. **Deprecate/remove** `LatticeGraph`, `Vertex`, `LatticeEdge`,
    `HexagonLattice`, `SquareLattice` once Tasks 3–9 are green and nothing
    else in the codebase references them (`grep` for `LatticeGraph`/
    `LatticeEdge`/`Vertex` usage outside `org.graphs` first — the exploration
    found none besides `CyclebuilderComms`, but re-verify before deleting).

---

## 7. Open questions to scrutinize

- **Edge id scheme**: should `HalfEdge.id()` be globally unique across the
  whole DCEL (as proposed), or keep the old `(roleId, localId)` compound key
  that `CyclebuilderComms` currently threads through messages
  (`assignedVertexID`/`assignedOutgoingEdgeID`)? Global ids simplify
  `LatticeDCEL.halfEdgeById`, but touch the message-serialization format
  (`PositioningMessage` et al.) — worth confirming that's acceptable churn.
- **Null-object vs. `null`**: `LatticeEdge`'s `isNull()` sentinel is used
  pervasively in `CyclebuilderComms`. The proposed pseudocode switches to
  plain `null` checks for brevity; the existing convention may be
  intentional (avoids NPEs in a large switch-heavy class) and worth keeping
  instead.
- **Where does `RigidBodyTransformation.compose` live**: needs matrix
  multiplication order to match the existing `apply()` convention exactly,
  or holonomy checks will be silently wrong (composed in the wrong order
  can still numerically look "close to identity" for symmetric lattices by
  coincidence — the octagon-square test in Task 9 is the one likely to
  catch an ordering bug, since 4- and 8-cycles are less forgiving of
  ordering mistakes than the hex lattice is).
- **`COMM_RANGE`/proximity-based neighbor discovery** (`AsyncRobotPanel.
  runProximityCheck`) is completely independent of the graph representation
  and doesn't need to change — noting this explicitly so it isn't
  accidentally pulled into the migration scope.
