package org.communicationModels.cycleBuildingComms;

import java.util.ArrayList;
import java.util.List;

import org.graphs.util.OrientedPoint;
import org.graphs.util.RigidBodyTransformation;
import org.graphs.voltage.Face;
import org.graphs.voltage.HalfEdge;
import org.graphs.voltage.VoltageGraph;
import org.robots.GeometricCycleLatticeRobot;
import org.utils.logging.CommsSnapshot;
import org.utils.logging.OutgoingMessageRecord;
import org.utils.logging.TickRecord;

/**
 * Stands up small, exactly-placed robot populations and drives them through real ticks,
 * so the cycle-building protocol can be tested end to end without a Swing panel.
 *
 * <p>Two decisions shape everything here.
 *
 * <p><strong>Robots are placed on their exact ideal sites.</strong> {@link #placeOnFace}
 * composes the lattice's own voltages to derive each pose, so no robot ever has to
 * <em>drive</em> anywhere. A test that begins with robots scattered spends most of its
 * ticks exercising {@code TimeStepDiffDrive} covering 50 to 70 units, and fails for
 * motion reasons when the protocol is fine -- or worse, passes because the protocol never
 * got far enough to be wrong. Placement makes these tests about message passing.
 *
 * <p><strong>Activation order is fixed and explicit.</strong> The simulation buys
 * reproducible asynchrony without randomness by staggering activation; a test wants the
 * same property with the stagger pinned down, so {@link #tick} activates in list order,
 * every tick. Where order is itself the thing under test, run the scenario twice with the
 * list reversed rather than introducing a shuffle.
 */
final class LatticeHarness {

    private LatticeHarness() {
    }

    /**
     * Places one robot on every site of the face reached by walking {@code getNext} from
     * {@code start}, with the first robot at {@code origin}.
     *
     * <p>Poses are derived the same way {@code CyclebuilderComms.getAssignedGlobalPosition}
     * derives a child's target -- compose the running transform with the half-edge's
     * voltage -- so every robot lands where the protocol would have sent it, to floating
     * point. Robot ids run {@code 0..cycleLength-1} in walk order.
     *
     * <p>The closing edge is deliberately not composed: doing so would place a duplicate
     * robot back on the origin. The returned list is exactly one robot per site.
     *
     * @param graph  the lattice these robots reason about
     * @param start  the half-edge whose face is to be populated
     * @param origin the pose of the first robot, which anchors the whole face
     * @return the robots, in walk order, already wired as mutual neighbours
     */
    static List<GeometricCycleLatticeRobot> placeOnFace(VoltageGraph graph, HalfEdge start,
                                                        OrientedPoint origin) {
        List<HalfEdge> boundary = boundaryFrom(start);
        List<GeometricCycleLatticeRobot> robots = new ArrayList<>(boundary.size());

        RigidBodyTransformation running = new RigidBodyTransformation(origin);
        robots.add(new GeometricCycleLatticeRobot(0, running.asPose(), graph));

        for (int i = 0; i < boundary.size() - 1; i++) {
            running = running.compose(boundary.get(i).getVoltage());
            robots.add(new GeometricCycleLatticeRobot(i + 1, running.asPose(), graph));
        }

        makeAllNeighbors(robots);
        return robots;
    }

    /**
     * The face boundary starting at a given half-edge, in {@code getNext} order.
     *
     * <p>Walks rather than calling {@code Face.getBoundary()}, because that starts from
     * the face's own representative half-edge and a test needs the walk to begin where it
     * says it does -- the robot at {@code origin} must be the one that owns {@code start}.
     */
    static List<HalfEdge> boundaryFrom(HalfEdge start) {
        Face face = start.getFace();
        List<HalfEdge> boundary = new ArrayList<>(face.getCycleLength());

        HalfEdge h = start;
        for (int i = 0; i < face.getCycleLength(); i++) {
            boundary.add(h);
            h = h.getNext();
        }
        return boundary;
    }

    /** Wires every robot as every other's neighbour, standing in for the proximity check. */
    static void makeAllNeighbors(List<GeometricCycleLatticeRobot> robots) {
        for (GeometricCycleLatticeRobot a : robots) {
            for (GeometricCycleLatticeRobot b : robots) {
                if (a.getRobotId() != b.getRobotId()) {
                    a.addNeighbor(b);
                }
            }
        }
    }

    /**
     * Runs {@code ticks} activations, each robot once per tick in list order.
     *
     * @return every {@link TickRecord} produced, in the order they happened. Tests read
     *         state from {@code record.after()}, which is already a {@link CommsSnapshot},
     *         so no new accessor on the robot is needed.
     */
    static List<TickRecord> tick(List<GeometricCycleLatticeRobot> robots, int ticks) {
        List<TickRecord> records = new ArrayList<>();
        for (int t = 1; t <= ticks; t++) {
            for (GeometricCycleLatticeRobot robot : robots) {
                records.add(robot.executeTimeStep(1.0, t));
            }
        }
        return records;
    }

    /** The most recent tick record for one robot, or null if it never activated. */
    static TickRecord lastRecordOf(List<TickRecord> records, int robotId) {
        TickRecord last = null;
        for (TickRecord record : records) {
            if (record.robotId() == robotId) {
                last = record;
            }
        }
        return last;
    }

    /**
     * Every message of the given type sent by anyone across the whole run.
     *
     * <p>Matched on {@code AbstractMessage.getMessageType()} -- "Assignment", "Status",
     * "Rejection", "Attempt Later", "Promotion" -- because {@link OutgoingMessageRecord}
     * keeps the type string and a summary rather than the message object itself.
     */
    static List<OutgoingMessageRecord> messagesOfType(List<TickRecord> records, String messageType) {
        List<OutgoingMessageRecord> found = new ArrayList<>();
        for (TickRecord record : records) {
            for (OutgoingMessageRecord sent : record.sent()) {
                if (sent.messageType().equals(messageType)) {
                    found.add(sent);
                }
            }
        }
        return found;
    }

    /**
     * Whether any face of {@code robotId} reached {@code complete} by the end of the run.
     *
     * <p>Reads the last snapshot rather than the robot, so a test asserts against what the
     * protocol recorded at the time rather than against state that may have been reset
     * since.
     */
    static boolean anyCycleComplete(List<TickRecord> records, int robotId) {
        TickRecord last = lastRecordOf(records, robotId);
        return last != null && last.after().completedCycles().containsValue(CycleStatus.complete);
    }
}
