# DCEL Graph Representation for Decentralized Swarm Formation

## Background

Graph representation is a fundamental component of the decentralized swarm formation algorithm. It defines the desired formation and provides each robot with the local information required to determine where neighboring robots should be positioned.

A formation is represented as a directed graph

\[
G = (V, E)
\]

where **V** is the set of vertices and **E** is the set of directed edges. Rather than representing individual robots, each vertex represents a **role** within the formation. During execution, every robot assumes one of these roles.

Each role contains a collection of outgoing edges that describe the desired rigid-body transformations from that role to neighboring roles. Once assigned a role, a robot uses these outgoing edges to determine how neighboring robots should be assigned within the formation. Conceptually, this process can be viewed as a function

\[
f : N \rightarrow E_o
\]

where **N** is the set of neighboring robots and **E<sub>o</sub>** is the set of outgoing edges associated with the robot's assigned role.

The graph representation therefore determines what information a robot can infer locally, how cycles are traversed during formation, and what geometric properties of the formation can be identified. Consequently, the choice of graph representation has a direct impact on the capabilities of the decentralized formation algorithm.

The current implementation uses the **Lattice Graph** representation. While this representation is compact and effective for highly symmetric lattice structures, it relies heavily on symmetry to infer graph connectivity. As the algorithm evolves to support more complex lattice structures, these assumptions no longer hold.

This document proposes replacing the Lattice Graph with a **Doubly Connected Edge List (DCEL)** representation that preserves decentralized execution while providing richer topological information.

## Current Architecture: Lattice Graph Representation

The Lattice Graph is a strongly connected directed multigraph

\[
G = (V, E).
\]

Each directed edge represents a desired rigid-body transformation from a source role to a neighboring target role. Every edge has a corresponding inverse edge that represents the reverse rigid-body transformation, allowing traversal back to the source role.

Instead of explicitly storing every vertex within an infinite repeating lattice, the Lattice Graph exploits symmetry. Only the minimum number of unique role vertices required to describe the repeating pattern are stored. The outgoing edges of these role vertices reference one another, allowing the lattice to repeat indefinitely.

Because identical role vertices appear throughout the lattice, one role is designated as the **primary vertex**, providing a consistent reference for ordering and identifying the remaining role vertices.

### Example: Square Lattice

A square lattice requires only a single role vertex, **V₀**.

Every outgoing edge from **V₀** references another copy of **V₀**, producing the repeating square lattice. Each edge stores the desired rigid-body transformation from the current robot, treated as the local origin, to one of its neighboring robots.

### Example: Hexagonal Lattice

A hexagonal lattice requires two role vertices, **V₀** and **V₁**.

Each role contains three outgoing edges, all of which connect to the opposite role. Robots therefore infer which role they occupy during assignment based on the existing structure of the partially constructed formation.

### Symmetry-Based Cycle Traversal

The decentralized formation algorithm exploits this symmetry to construct cycles between robots.

Outgoing edges are ordered so that, given the edge currently being traversed, the algorithm can infer the next outgoing edge that continues around the desired geometric cycle. For regular lattices such as squares and hexagons, this allows robots to traverse a cycle using only local information.

This inference mechanism is possible because every instance of a role vertex shares the same local neighborhood structure.

## Benefits and Limitations

### Benefits

The primary advantage of the Lattice Graph representation is its simplicity.

By reducing a repeating lattice to a minimal collection of role vertices and directed edges, symmetric formations can be represented compactly while still providing every robot with sufficient local information for decentralized assignment.

This representation performs well for highly regular lattice structures such as square and hexagonal formations.

### Limitations

Although effective for symmetric lattices, the Lattice Graph relies on symmetry to infer graph topology.

Consider the octagon-square lattice. Unlike square or hexagonal lattices, this structure contains multiple face types with different cycle lengths. Traversing an octagonal face requires eight edges before returning to the starting point, while traversing a square requires only four.

The existing edge inference mechanism assumes that every cycle follows the same repeating symmetric structure. Once multiple cycle lengths exist within the same lattice, this assumption breaks down. The ordering of outgoing edges is no longer sufficient to uniquely determine the next edge in a traversal, preventing reliable reconstruction of graph cycles.

A second limitation is the absence of an explicit representation of graph faces. While vertex adjacency is encoded, the regions enclosed by edges are not. As a result, operations involving faces must be reconstructed indirectly from the graph structure, increasing algorithmic complexity.

These limitations restrict the current algorithm from supporting more general lattice structures and motivate the need for a richer graph representation.

## Design Requirements

A replacement for the Lattice Graph should satisfy the following requirements:

1. Support both symmetric and asymmetric lattice structures.
2. Allow deterministic traversal of a graph cycle beginning from any directed edge.
3. Provide a mechanism for verifying that a proposed robot cycle corresponds to a valid graph cycle.
4. Explicitly represent graph faces.
5. Preserve the decentralized nature of the formation algorithm by allowing robots to operate using only local information.

## Proposed Architecture: Doubly Connected Edge List (DCEL)

To address the limitations of the Lattice Graph, we propose replacing it with a **Doubly Connected Edge List (DCEL)** representation.

Unlike the Lattice Graph, which stores only role vertices and adjacency relationships, a DCEL explicitly represents the topological relationships between vertices, directed edges, and faces. Rather than relying on symmetry to infer graph connectivity, the connectivity is encoded directly within the data structure.

Every directed edge stores sufficient information to navigate both around its incident vertex and around the face to which it belongs. Consequently, operations that previously required inference become simple graph traversals.

Although the underlying representation changes, the decentralized execution model remains unchanged.

Each robot continues to:

- assume a role within the formation,
- treat itself as the origin of its local coordinate frame,
- assign neighboring robots using only locally available information.

The DCEL therefore changes only how the formation is represented internally. It does not require robots to possess global knowledge of the swarm or alter the decentralized assignment process.

## Advantages of the DCEL

Replacing the Lattice Graph with a DCEL provides several advantages.

### Generalized Lattice Support

Because connectivity is represented explicitly rather than inferred from symmetry, arbitrary repeating lattice structures can be represented naturally. This includes asymmetric lattices such as the octagon-square tessellation.

### Deterministic Cycle Traversal

Given any directed edge, the next edge in a cycle is obtained directly from the graph representation rather than inferred through edge ordering. This makes cycle traversal independent of lattice symmetry.

### Cycle Validation

Since graph cycles correspond directly to face boundaries, robots can verify that a proposed robot cycle matches a valid cycle within the desired formation.

### Explicit Face Representation

Faces become first-class entities within the graph representation, enabling algorithms that reason about enclosed regions without additional preprocessing.

### Future Extensibility

The richer topological information provided by the DCEL establishes a stronger architectural foundation for future decentralized algorithms involving topology, navigation, validation, or geometric reasoning.

## Summary

The Lattice Graph representation has proven effective for highly symmetric lattice formations because it provides a compact representation and enables decentralized robot assignment using local information.

However, its reliance on symmetry limits its ability to represent more general lattice structures and complicates operations involving cycle traversal, cycle validation, and face identification.

Replacing the Lattice Graph with a Doubly Connected Edge List removes these limitations by explicitly representing the topology of the formation. The DCEL preserves the decentralized execution model while enabling support for asymmetric lattice structures and providing a more expressive foundation for future swarm formation algorithms.