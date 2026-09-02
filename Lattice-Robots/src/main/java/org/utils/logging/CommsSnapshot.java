package org.utils.logging;

import org.communicationModels.Observation;
import org.communicationModels.TrustLevel;
import org.communicationModels.cycleBuildingComms.CycleRole;
import org.communicationModels.cycleBuildingComms.CycleStatus;
import org.communicationModels.cycleBuildingComms.CyclebuilderComms;
import org.communicationModels.cycleBuildingComms.FaceObligation;
import org.communicationModels.cycleBuildingComms.Messages.AbstractMessage;
import org.graphs.voltage.HalfEdge;

import java.util.List;
import java.util.Map;

/**
 * Read-only, defensively-copied view of a {@link CyclebuilderComms} instance
 * at one moment in time.
 *
 * <p>Produced by {@link CyclebuilderComms#snapshot()}. All collection fields
 * are immutable copies, so a {@code CommsSnapshot} can be held onto across
 * ticks (e.g. as the "before" half of a diff) without risk of the live comms
 * state mutating out from under it.</p>
 *
 * <p>This type intentionally has no dependency on the logging package —
 * {@code CyclebuilderComms} produces it as a plain read model, and the
 * logging subsystem (see {@code org.logging}) consumes it. That keeps the
 * algorithm code free of any knowledge that logging exists.</p>
 */
public record CommsSnapshot(
        CycleRole role,
        TrustLevel trust,
        boolean hasFailed,
        int stableID,
        HalfEdge assignedEdge,
        HalfEdge originEdge,
        Map<Integer, CycleStatus> completedCycles,
        List<AbstractMessage> queueInOrder,
        Map<Integer, Observation> observations,
        /**
         * The communication tuples this robot holds -- who assigned it each face, which
         * edge it owes onward, who it has offered that edge to, and who has declined.
         *
         * <p>Replaces the flat {@code unableToDoAssignmentIDs} list that used to sit here.
         * Those exclusions were robot-scoped and so said nothing about <em>which</em> face
         * a robot had been ruled out of; they are now per-obligation, readable via
         * {@link FaceObligation#getBans()}.
         *
         * <p>It also replaces the two fields that used to sit above it, both of which
         * survived only while a robot served one face at a time. {@code pendingChildID} was
         * "the child I am waiting on", which stops being a single value the moment a robot
         * has several; {@code certificate} was "the walk I am carrying", which stops being
         * one at the same moment -- the walks now live in the inbox, one per queued
         * assignment. Neither could be kept as a derived convenience without picking an
         * arbitrary one of several and reporting it as the answer.
         */
        List<FaceObligation> obligations,

        /**
         * The lattice this robot reasons about, so a reader can resolve the half-edge ids in
         * {@link #completedCycles()} and {@link #obligations()} back into faces.
         *
         * <p>A reference, not a copy: a {@code VoltageGraph} is built once and immutable
         * thereafter, so there is nothing here to defensively copy and nothing that can
         * mutate out from under a held snapshot. It is the one field that is the same object
         * for every robot in a run.
         *
         * <p>Carried rather than reached for statically because the graph is injectable --
         * {@code LatticeHarness} stands up scenarios on eleven different lattices, and a
         * frame view resolving ids through {@code GeometricCycleLatticeRobot.GRAPH} would
         * silently render every one of them as whichever lattice the simulation happens to
         * be configured for.
         */
        org.graphs.voltage.VoltageGraph graph
) {

    /**
     * Whether {@link #completedCycles()} means anything for this robot.
     *
     * <p>The map is populated by {@code CyclebuilderComms.initializeEdgeMap()},
     * which runs once per lattice site a robot occupies — when it accepts an
     * assignment and becomes a {@code cycleBuilder}, or when it is promoted
     * straight from {@code unassigned} — one entry per outgoing half-edge of
     * the role it occupies. So builders track corners too: a status wrapping a
     * closed face is recorded by every participant, not only by the roots.</p>
     *
     * <p>Empty only for {@code unassigned} robots, and it survives promotion to
     * {@code root} and then to {@code stable}. Keyed off the map rather than off
     * {@link #role()} because no single role answers this — one role has no map
     * and the other three do.</p>
     */
    public boolean tracksCycles() {
        return !completedCycles.isEmpty();
    }

    /** Total faces this robot is responsible for closing as a root, i.e. its outgoing-edge count. */
    public int totalCycles() {
        return completedCycles.size();
    }

    /** How many of those faces have closed. */
    public int completedCycleCount() {
        return countWithStatus(CycleStatus.complete);
    }

    /**
     * How many faces this root still has to close: everything not yet
     * {@link CycleStatus#complete}.
     *
     * <p>{@link CycleStatus#failed} counts as remaining, not as finished — a
     * failed cycle is re-armed to {@code unattempted} by
     * {@code reattemptFailedCycles()} whenever a neighbour reaches stable, so
     * it is outstanding work rather than a settled outcome. Use
     * {@link #countWithStatus(CycleStatus)} to break the remainder down.</p>
     */
    public int remainingCycles() {
        return totalCycles() - completedCycleCount();
    }

    /**
     * The face a tracked corner belongs to, as {@code "<faceId> (<cycleLength>-cycle)"}.
     *
     * <p>Task 7 of {@code DCEL-Implementation-Plan.md}, corrected. The doc asks for
     * {@code completedCycles} to be <em>re-keyed</em> from half-edge to face, which would be
     * a regression: {@code Face} is a face <em>type</em>, not an instance, so several of a
     * role's outgoing edges share one face id -- all four on a square lattice, all six on
     * triangle. Re-keying collapses four corners into one and six into two, a root promotes
     * itself to stable after closing a single face, and nothing fails to compile and no test
     * goes red.
     *
     * <p>So the key stays the edge and the face travels beside it -- and travels for free,
     * because a half-edge already knows its incident face. There is nothing to store and
     * nothing to keep in step; this is a rendering of {@code completedCycles}, not a second
     * copy of it.
     */
    public String describeFaceOf(int outgoingEdgeID) {
        HalfEdge edge = graph == null ? null : graph.getHalfEdgeById(outgoingEdgeID);
        if (edge == null || edge.getFace() == null) {
            return "face ?";
        }
        return "face " + edge.getFace().getId()
                + ", " + edge.getFace().getCycleLength() + "-cycle";
    }

    /** How many of this root's cycles currently sit in the given status. */
    public int countWithStatus(CycleStatus status) {
        int count = 0;
        for (CycleStatus value : completedCycles.values()) {
            if (value == status) {
                count++;
            }
        }
        return count;
    }
}
