package org.simulation.AsynchSim;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;

import org.communicationModels.cycleBuildingComms.CycleStatus;
import org.communicationModels.cycleBuildingComms.FaceObligation;
import org.graphs.util.OrientedPoint;
import org.robots.GeometricCycleLatticeRobot;
import org.utils.logging.CommsSnapshot;

/**
 * The whole swarm's state as one string, so two runs can be compared for exact equality.
 *
 * <p>This is the instrument the reproducibility work is measured with. "The formations look the
 * same" is not a claim that can fail a build; "the digests are equal" is, and it fails on the first
 * robot whose pose differs in the last bit rather than after a divergence has grown large enough to
 * see. {@code DeterminismTest} runs the same schedule twice and compares these.
 *
 * <p><strong>Everything is read in a sorted order, never a map's own.</strong> Robots by ascending
 * id, cycle statuses by ascending edge id, bans ascending. {@code completedCycles} is a
 * {@code HashMap}, and a digest that inherited its iteration order would be reporting on the map's
 * internals as much as on the simulation -- worse, it would look stable, because {@code Integer}
 * hashing is not seeded and so repeats within a machine. Sorting removes the question.
 *
 * <p><strong>Poses go in as raw bits.</strong> {@link Double#doubleToLongBits} rather than a
 * rounded decimal: the property under test is that two runs compute the <em>same</em> numbers, and
 * rounding is exactly how a real divergence hides. A digest that tolerates a millimetre would pass
 * a run that had already picked a different neighbour.
 */
public final class SwarmDigest {

    private SwarmDigest() {}

    /**
     * A stable, human-readable rendering of every robot's pose and protocol state.
     *
     * <p>Returned as text rather than a hash so that a failing comparison can be diffed and read.
     * Callers wanting a short value can hash it themselves; nothing here needs to be compact.
     */
    public static String of(Collection<GeometricCycleLatticeRobot> robots) {
        List<GeometricCycleLatticeRobot> ordered = new ArrayList<>(robots);
        ordered.sort((a, b) -> Integer.compare(a.getRobotId(), b.getRobotId()));

        StringBuilder out = new StringBuilder();
        for (GeometricCycleLatticeRobot robot : ordered) {
            appendRobot(out, robot);
        }
        return out.toString();
    }

    private static void appendRobot(StringBuilder out, GeometricCycleLatticeRobot robot) {
        OrientedPoint pose = robot.getPosition();
        CommsSnapshot state = robot.snapshot();

        out.append("robot ").append(robot.getRobotId())
           .append(" pose=").append(Double.doubleToLongBits(pose.x))
           .append(',').append(Double.doubleToLongBits(pose.y))
           .append(',').append(Double.doubleToLongBits(pose.getOrientation()))
           .append(" role=").append(state.role())
           .append(" trust=").append(state.trust())
           .append(" failed=").append(state.hasFailed())
           .append(" assigned=").append(state.assignedEdge() == null ? -1 : state.assignedEdge().getId());

        appendCycleStatuses(out, state.completedCycles());
        appendObligations(out, state.obligations());

        // Messages still in flight. A divergence in ordering shows up here a tick before it shows
        // up in any position, which is the whole reason the inbox is included.
        out.append(" queue=[");
        for (int i = 0; i < state.queueInOrder().size(); i++) {
            if (i > 0) {
                out.append(' ');
            }
            var message = state.queueInOrder().get(i);
            out.append(message.getMessageType()).append(':').append(message.getSenderId());
        }
        out.append("]\n");
    }

    private static void appendCycleStatuses(StringBuilder out, Map<Integer, CycleStatus> statuses) {
        List<Integer> edges = new ArrayList<>(statuses.keySet());
        edges.sort(Integer::compare);
        out.append(" cycles={");
        for (int i = 0; i < edges.size(); i++) {
            if (i > 0) {
                out.append(' ');
            }
            out.append(edges.get(i)).append('=').append(statuses.get(edges.get(i)));
        }
        out.append('}');
    }

    /**
     * Obligations in the order the robot holds them, which is itself part of what is being tested --
     * {@code FaceObligationSet} is backed by an insertion-ordered list precisely so that face
     * selection cannot become order-dependent, and a digest that sorted them would hide a change in
     * that order.
     */
    private static void appendObligations(StringBuilder out, List<FaceObligation> obligations) {
        out.append(" links=[");
        for (int i = 0; i < obligations.size(); i++) {
            if (i > 0) {
                out.append(' ');
            }
            FaceObligation link = obligations.get(i);
            List<Integer> bans = new ArrayList<>(link.getBans());
            bans.sort(Integer::compare);
            out.append(link.getParentId()).append('>')
               .append(link.getEdgeId()).append('>')
               .append(link.getChildId() == null ? "none" : link.getChildId())
               .append(bans.isEmpty() ? "" : "!" + bans);
        }
        out.append(']');
    }
}
