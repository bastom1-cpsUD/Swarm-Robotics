package org.utils.logging;

import org.communicationModels.Observation;
import org.communicationModels.TrustLevel;
import org.communicationModels.cycleBuildingComms.CycleRole;
import org.communicationModels.cycleBuildingComms.CycleStatus;
import org.communicationModels.cycleBuildingComms.CyclebuilderComms;
import org.communicationModels.cycleBuildingComms.Messages.AbstractMessage;
import org.communicationModels.cycleBuildingComms.Messages.VoltageCertificate;
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
        int pendingChildID,
        int stableID,
        VoltageCertificate certificate,
        HalfEdge assignedEdge,
        HalfEdge originEdge,
        Map<Integer, CycleStatus> completedCycles,
        List<AbstractMessage> queueInOrder,
        Map<Integer, Observation> observations,
        List<Integer> unableToDoAssignmentIDs
) {

    /**
     * Whether {@link #completedCycles()} means anything for this robot.
     *
     * <p>The map is populated by {@code CyclebuilderComms.initializeEdgeMap()},
     * which only runs on promotion to {@code root} — one entry per outgoing
     * half-edge of the role this robot occupies. It is empty for
     * {@code unassigned}/{@code cycleBuilder} robots, and survives the
     * subsequent promotion to {@code stable} (fully complete at that point),
     * so this is keyed off the map rather than off {@link #role()}.</p>
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
