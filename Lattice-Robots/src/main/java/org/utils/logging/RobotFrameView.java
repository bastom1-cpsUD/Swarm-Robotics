package org.utils.logging;

import com.darcarms.htmllog.HtmlLog;
import com.darcarms.htmllog.LogGraphics;
import com.darcarms.htmllog.Loggable;
import com.darcarms.htmllog.Rect;
import com.darcarms.htmllog.TransformTools;

import org.communicationModels.Observation;
import org.graphs.util.OrientedPoint;
import org.graphs.util.RigidBodyTransformation;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Path2D;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

/**
 * Renders a single robot's {@link TickRecord} into an {@link HtmlLog}: a
 * text summary of what happened, plus a robot-centric drawing.
 *
 * <p>The drawing is deliberately in the robot's own local frame rather than
 * world coordinates:</p>
 * <ul>
 *   <li>The robot itself sits at the origin, facing +y — no transform needed,
 *       since that's already the frame {@code makeFirstPhaseObservations()} computes.</li>
 *   <li>Neighbor dots are plotted directly from {@code Observation.getLocalPosition()}
 *       captured in the pre-tick snapshot — exactly the data the algorithm
 *       itself used to make its decision this tick. Only the position is drawn;
 *       {@code Observation.getLocalOrientation()} now also carries the neighbor's
 *       heading relative to this robot, which could be rendered as a facing arrow.</li>
 *   <li>The trajectory (a list of world-frame poses accumulated by
 *       {@link SimulationLogger}) is re-projected into <em>this tick's</em>
 *       local frame at render time, via {@code RigidBodyTransformation}'s
 *       inverse of the robot's current pose. Since that frame rotates with
 *       the robot each tick, the trail visually sweeps out behind it.</li>
 * </ul>
 */
public record RobotFrameView(TickRecord rec, List<OrientedPoint> trajectory) implements Loggable {

    private static final Rect VIEW = new Rect(-90, -90, 90, 90); // a bit past COMM_RANGE
    private static final int IMG_SIZE = 420;

    @Override
    public void show(HtmlLog log) {
        log.heading("Robot " + rec.robotId() + " \u2014 tick " + rec.tick());

        log.pre("""
                role:            %s -> %s
                queue size:      %d -> %d
                pendingChildID:  %d -> %d
                processed:       %s
                action:          %s
                sent:            %s
                """.formatted(
                rec.before().role(), rec.after().role(),
                rec.before().queueInOrder().size(), rec.after().queueInOrder().size(),
                rec.before().pendingChildID(), rec.after().pendingChildID(),
                rec.processedDescription(),
                rec.actionDescription(),
                rec.sent().isEmpty() ? "(none)" : rec.sent().toString())
                + cycleProgress());

        try (LogGraphics canvas = log.mappedGraphics("robot-" + rec.robotId() + "-", VIEW, IMG_SIZE, IMG_SIZE)) {
            Graphics2D g = canvas.graphics();
            drawTrajectory(g);
            drawNeighbors(g);
            drawSelf(g);
        }
    }

    /**
     * The root-only progress lines appended to the summary block: how many of
     * this robot's faces are still open, and where each one stands.
     *
     * <p>Empty for any robot with no cycle bookkeeping (see
     * {@link CommsSnapshot#tracksCycles()}), so unassigned robots and
     * cycleBuilders — for which the counts would be a meaningless
     * {@code 0 of 0} — get no line at all. The "before" side reads {@code n/a}
     * on the tick a robot is promoted to root, since its edge map did not exist
     * yet when that snapshot was taken.</p>
     */
    private String cycleProgress() {
        if (!rec.before().tracksCycles() && !rec.after().tracksCycles()) {
            return "";
        }

        CommsSnapshot after = rec.after();
        return """
                cycles left:     %s -> %s
                cycle statuses:  %s
                """.formatted(
                describeRemaining(rec.before()),
                describeRemaining(after),
                describeStatuses(after));
    }

    private static String describeRemaining(CommsSnapshot snapshot) {
        if (!snapshot.tracksCycles()) {
            return "n/a";
        }
        return "%d of %d".formatted(snapshot.remainingCycles(), snapshot.totalCycles());
    }

    /**
     * Per-edge breakdown of the remainder, sorted by half-edge id so the same
     * root reads the same way from one tick to the next — {@code completedCycles}
     * is a copy of a {@code HashMap}, whose iteration order carries no meaning.
     */
    private static String describeStatuses(CommsSnapshot snapshot) {
        if (!snapshot.tracksCycles()) {
            return "(none)";
        }
        return snapshot.completedCycles().entrySet().stream()
                .sorted(Map.Entry.comparingByKey())
                .map(entry -> "edge " + entry.getKey() + "=" + entry.getValue())
                .collect(Collectors.joining(", "));
    }

    private void drawTrajectory(Graphics2D g) {
        if (trajectory.size() < 2) {
            return;
        }

        RigidBodyTransformation globalToLocal = new RigidBodyTransformation(rec.poseAfter()).inverse();

        Path2D path = new Path2D.Double();
        OrientedPoint first = globalToLocal.apply(trajectory.get(0));
        path.moveTo(first.x, first.y);
        for (OrientedPoint global : trajectory) {
            OrientedPoint local = globalToLocal.apply(global);
            path.lineTo(local.x, local.y);
        }

        g.setColor(new Color(60, 120, 210));
        g.setStroke(new BasicStroke(1.2f));
        g.draw(path);
    }

    private void drawNeighbors(Graphics2D g) {
        g.setStroke(new BasicStroke(1f));
        for (Map.Entry<Integer, Observation> entry : rec.before().observations().entrySet()) {
            OrientedPoint local = entry.getValue().getLocalPosition();

            g.setColor(new Color(210, 90, 90));
            g.fill(new Ellipse2D.Double(local.x - 3, local.y - 3, 6, 6));
            g.setColor(Color.BLACK);
            g.draw(new Ellipse2D.Double(local.x - 3, local.y - 3, 6, 6));

            TransformTools.drawUprightString(g, String.valueOf(entry.getKey()), local.x + 5, local.y - 5);
        }
    }

    private void drawSelf(Graphics2D g) {
        g.setColor(new Color(120, 180, 255));
        TransformTools.drawArrowhead(g, 0, -16, 0, 16, 12, Math.PI / 5.0, true);
        g.setColor(Color.BLACK);
        g.fill(new Ellipse2D.Double(-2.5, -2.5, 5, 5));
    }
}
