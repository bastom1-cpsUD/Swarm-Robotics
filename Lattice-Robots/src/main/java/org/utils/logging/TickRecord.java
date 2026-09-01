package org.utils.logging;

import org.graphs.util.OrientedPoint;
import org.graphs.voltage.HalfEdge;
import org.utils.MathUtils;

import java.util.List;

/**
 * Everything that happened during one robot's activation: its comms state
 * and pose immediately before the tick, which message was processed and
 * what action resulted, which outgoing messages were sent, and its comms
 * state and pose immediately after.
 *
 * <p>Produced by {@code GeometricCycleLatticeRobot.executeTimeStep(dt, tick)}.
 * Consumed by {@link SimulationLogger}, which decides whether it's worth
 * writing to the HTML log via {@link #changed()}.</p>
 */
public record TickRecord(
        int tick,
        int robotId,
        CommsSnapshot before,
        OrientedPoint poseBefore,
        String processedDescription,
        String actionDescription,
        List<OutgoingMessageRecord> sent,
        CommsSnapshot after,
        OrientedPoint poseAfter
) {

    /**
     * Whether anything worth logging actually happened this tick.
     *
     * <p>Deliberately avoids relying on {@code equals()} for domain types
     * whose equality semantics aren't guaranteed (e.g. {@link HalfEdge}
     * instances are re-looked-up from the graph on every call and may not
     * override {@code equals}/{@code hashCode}); compares by the specific
     * identifying fields instead.</p>
     */
    public boolean changed() {
        return before.role() != after.role()
                || before.stableID() != after.stableID()
                || before.hasFailed() != after.hasFailed()
                || before.queueInOrder().size() != after.queueInOrder().size()
                || !before.completedCycles().equals(after.completedCycles())
                || !sameEdge(before.assignedEdge(), after.assignedEdge())
                || !sameEdge(before.originEdge(), after.originEdge())
                // The obligations, which took over from the pendingChildID and certificate
                // comparisons that used to sit here. Both of those were single-valued and
                // stopped meaning anything once a robot serves several faces; between them
                // they were also trying to say what this one comparison says outright, which
                // is "did any face this robot is working on move". A relay leaves role, edges
                // and pose untouched, so without this the tick whose only effect was carrying
                // a walk one hop goes unlogged -- exactly the tick worth seeing.
                || !before.obligations().equals(after.obligations())
                || !samePose(poseBefore, poseAfter)
                || !sent.isEmpty();
    }

    private static boolean sameEdge(HalfEdge a, HalfEdge b) {
        if (a == null || b == null) {
            return a == b;
        }
        // HalfEdge ids are globally unique, unlike the old per-vertex-local
        // LatticeEdge ids, so id equality alone is sufficient here.
        return a.getId() == b.getId();
    }

    private static boolean samePose(OrientedPoint a, OrientedPoint b) {
        return a.distance(b) < MathUtils.EPSILON
                && MathUtils.anglesEqual(a.orientation, b.orientation);
    }
}
