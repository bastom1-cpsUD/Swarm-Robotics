package org.utils.logging;

import org.communicationModels.CycleRole;
import org.communicationModels.CyclebuilderComms;
import org.communicationModels.Observation;
import org.communicationModels.TrustLevel;
import org.communicationModels.Messages.AbstractMessage;
import org.communicationModels.Messages.ChainMemberList;
import org.graphs.LatticeEdge;

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
        ChainMemberList chainMemberList,
        LatticeEdge assignedEdge,
        LatticeEdge originEdge,
        Map<Integer, Boolean> completedCycles,
        List<AbstractMessage> queueInOrder,
        Map<Integer, Observation> observations,
        List<Integer> unableToDoAssignmentIDs
) {
}
