package org.communicationModels.cycleBuildingComms;

import java.util.List;
import java.util.Set;

import org.communicationModels.cycleBuildingComms.Messages.PositioningMessage;
import org.communicationModels.cycleBuildingComms.Messages.RejectAssignmentMessage;
import org.communicationModels.cycleBuildingComms.Messages.StatusMessage;
import org.communicationModels.cycleBuildingComms.Messages.VoltageCertificate;
import org.graphs.util.OrientedPoint;
import org.graphs.util.RigidBodyTransformation;
import org.graphs.voltage.HalfEdge;
import org.graphs.voltage.SquareVoltageGraph;
import org.graphs.voltage.VoltageGraph;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.robots.GeometricCycleLatticeRobot;
import org.utils.logging.TickRecord;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Duplicate certificates racing one face, and the return path that stops them.
 *
 * <p>This is what replaced co-initiation arbitration. Two robots building the same face used to be
 * a conflict to resolve: whichever met the other's walk first compared initiator ids and one stood
 * down. Both ways of expressing that failed, and they failed for the same underlying reason --
 * <strong>a robot cannot usefully arbitrate over a walk that is not its own.</strong>
 *
 * <ul>
 *   <li><em>Refusing</em> the loser's walk sent a non-retryable rejection from a robot standing
 *       exactly on the site being offered. The offerer banned the one robot that could occupy that
 *       site, worked down candidates that were all equally not-there, and wrote the corner off. A
 *       temporary condition became a permanent verdict.</li>
 *   <li><em>Holding</em> it until the winner's own walk resolved deadlocked instead. Two walks
 *       tracing one face visit the same sites in the same direction, so they share every
 *       communication link along it; holding one holds everything queued behind it, including the
 *       holder's own returning certificate.</li>
 * </ul>
 *
 * <p>So neither walk is arbitrated. Both are carried, each laps the face and closes at its own
 * initiator, and a corner already complete makes the later one a no-op. What has to be true for
 * that to terminate is the subject of these tests: a returning message must name the robot it is
 * addressed to, or it circulates forever.
 */
class DuplicateCertificateRaceTest {

    private static final VoltageGraph SQUARE = SquareVoltageGraph.build();
    private static final int LONG_RUN = 240;

    /**
     * Runs one robot until it has a walk of its own out with a child recorded, and returns that
     * attempt.
     *
     * <p>Polls rather than reading a fixed tick: the attempt is opened on one activation and
     * offered on the next, and it is dropped again the moment its status comes home -- so any
     * single tick can miss it in either direction.
     */
    private static FaceObligation attemptInFlight(GeometricCycleLatticeRobot robot) {
        for (int tick = 1; tick <= 40; tick++) {
            for (FaceObligation o : robot.executeTimeStep(1.0, tick).after().obligations()) {
                if (o.getParentId() == FaceObligation.NO_PARENT && o.getChildId() != null) {
                    return o;
                }
            }
        }
        return null;
    }

    /**
     * Every root on a shared face closes its own walk. Not one of them -- all of them.
     *
     * <p>Four roots on one square face means four certificates in flight around the same four
     * sites at once, which is precisely the situation the old arbitration existed to prevent. The
     * claim is that it needed no preventing.
     */
    @Test
    @DisplayName("four certificates racing one face all close, at their own initiators")
    void everyRacingCertificateCloses() {
        List<GeometricCycleLatticeRobot> robots = LatticeHarness.placeOnFace(
                SQUARE, SQUARE.getOutgoingHalfEdges(SQUARE.getPrimaryRole()).get(0),
                new OrientedPoint(0, 0, 0));
        for (GeometricCycleLatticeRobot robot : robots) {
            robot.promoteToPrimaryRoot();
        }

        List<TickRecord> records = LatticeHarness.tick(robots, LONG_RUN);

        for (GeometricCycleLatticeRobot robot : robots) {
            assertTrue(LatticeHarness.anyCycleComplete(records, robot.getRobotId()),
                    "robot " + robot.getRobotId() + " closed no corner in " + LONG_RUN + " ticks. "
                            + "Its certificate either never came home or was refused on the way. "
                            + "Duplicate walks on one face are redundant, not conflicting -- "
                            + "nothing on the walk is entitled to judge a certificate it did not "
                            + "mint.");
        }
    }

    /**
     * A neighbourhood with every site around the seed occupied, so any corner the seed opens has a
     * candidate standing on it and any edge a walk arrives over has a real sender.
     *
     * <p>Only the seed is ticked in the staged tests below. That is deliberate isolation: nobody
     * else sends anything, so the seed's inbox holds exactly what the test puts there and a single
     * activation is enough to observe how one message was handled.
     */
    private static List<GeometricCycleLatticeRobot> neighbourhood() {
        List<GeometricCycleLatticeRobot> robots = LatticeHarness.placeAroundRole(
                SQUARE, SQUARE.getPrimaryRole(), new OrientedPoint(0, 0, 0));
        robots.get(0).promoteToPrimaryRoot();
        return robots;
    }

    /** The robot standing at the far end of {@code edge} from the seed, or null. */
    private static GeometricCycleLatticeRobot occupantOf(List<GeometricCycleLatticeRobot> robots,
                                                         HalfEdge edge) {
        OrientedPoint site = new RigidBodyTransformation(robots.get(0).getPosition())
                .compose(edge.getVoltage()).asPose();
        for (GeometricCycleLatticeRobot robot : robots) {
            if (robot.getRobotId() != 0 && robot.getPosition().distance(site) < 1e-6) {
                return robot;
            }
        }
        return null;
    }

    /** The incoming edge a walk must arrive over to owe {@code corner} next. */
    private static HalfEdge arrivesOwing(HalfEdge corner) {
        for (HalfEdge outgoing : SQUARE.getOutgoingHalfEdges(SQUARE.getPrimaryRole())) {
            if (SQUARE.getNext(outgoing.getTwin()).getId() == corner.getId()) {
                return outgoing.getTwin();
            }
        }
        return null;
    }

    /**
     * A status stops at the robot named by its {@code initiatorId}, and only there.
     *
     * <p>The stop condition cannot come from the link chain, and that is worth stating because the
     * chain looks like it would do. A root building corner {@code c} while also relaying a walk
     * that owes {@code c} offers both to the same neighbour, so two of its tuples name that child
     * -- and {@code findByChild} answers with the carried one, because it scans the carried list
     * first. Routed that way the root would forward its own verdict onward instead of settling it,
     * and the status would lap the ring forever.
     */
    @Test
    @DisplayName("a status is consumed by its initiator and relayed by everyone else")
    void statusStopsAtItsInitiator() {
        List<GeometricCycleLatticeRobot> robots = neighbourhood();
        GeometricCycleLatticeRobot seed = robots.get(0);

        FaceObligation attempt = attemptInFlight(seed);
        assertNotNull(attempt, "scenario error: the seed never got a walk of its own in flight");
        int ownChild = attempt.getChildId();
        int corner = attempt.getEdgeId();

        // Give the seed a link of somebody else's to relay a foreign status back along.
        HalfEdge incoming = arrivesOwing(SQUARE.getHalfEdgeById(corner));
        assertNotNull(incoming, "scenario error: no incoming edge owes corner " + corner);
        GeometricCycleLatticeRobot sender = occupantOf(robots, incoming.getTwin());
        assertNotNull(sender, "scenario error: nobody stands where that walk must come from");
        seed.enqueueMessage(new PositioningMessage(sender.getRobotId(), seed.getRobotId(),
                incoming.getOrigin().getId(), incoming.getId(),
                incoming.getOrigin().getId(), incoming.getId(),
                new VoltageCertificate(999)));
        FaceObligation link = null;
        for (int tick = 50; tick <= 70 && link == null; tick++) {
            for (FaceObligation o : seed.executeTimeStep(1.0, tick).after().obligations()) {
                if (o.getParentId() == sender.getRobotId() && o.getChildId() != null) {
                    link = o;
                }
            }
        }
        assertNotNull(link, "scenario error: the seed never carried robot 999's walk onward");

        // A status for SOMEBODY ELSE'S walk: relayed, and this robot's own attempt untouched.
        seed.enqueueMessage(new StatusMessage(link.getChildId(), seed.getRobotId(), true,
                incoming.getOrigin().getId(), incoming.getId(), 999, new VoltageCertificate(999)));
        TickRecord relayed = seed.executeTimeStep(1.0, 100);
        assertTrue(relayed.processedDescription().contains("RELAYED"),
                "a status for robot 999's walk was not relayed onward: "
                        + relayed.processedDescription() + ". Only the minter may act on a verdict; "
                        + "everyone else forwards it along the link it arrived over.");
        assertTrue(relayed.after().obligations().stream()
                        .anyMatch(o -> o.getParentId() == FaceObligation.NO_PARENT),
                "relaying somebody else's status cleared this robot's own attempt");

        // Now one for the seed's OWN walk: consumed, corner recorded, attempt dropped.
        seed.enqueueMessage(new StatusMessage(ownChild, seed.getRobotId(), true,
                SQUARE.getPrimaryRole().getId(), corner, seed.getRobotId(),
                new VoltageCertificate(seed.getRobotId())));
        TickRecord settled = seed.executeTimeStep(1.0, 101);
        assertFalse(settled.processedDescription().contains("RELAYED"),
                "the seed forwarded a status for its OWN walk: " + settled.processedDescription()
                        + ". A return message not stopped by its initiator laps the ring forever -- "
                        + "that is what initiatorId exists to prevent.");
        assertEquals(CycleStatus.complete, settled.after().completedCycles().get(corner),
                "the seed's own status came home and its corner was not recorded. Marking happens "
                        + "on the status, not when the certificate was judged -- see settleOwnWalk.");
        // The attempt on THAT corner is gone. There may well be a new attempt in the same snapshot,
        // on a different corner: settling frees the in-flight marker and sendMessage runs later in
        // the same activation, so the root starts its next face immediately. That is the behaviour
        // the marker exists to enable, not a leak.
        assertTrue(settled.after().obligations().stream()
                        .noneMatch(o -> o.getParentId() == FaceObligation.NO_PARENT
                                && o.getEdgeId() == corner),
                "the attempt on corner " + corner + " survived its own status. It is the in-flight "
                        + "marker, and a root that never drops it never builds that face again: "
                        + settled.after().obligations());
    }

    /**
     * A communication link outlives the walk that opened it, and carries no exclusions once that
     * walk has resolved.
     *
     * <p>Both halves matter and they pull in opposite directions. The link must survive, or the
     * next certificate over that edge re-shops a neighbourhood whose answer cannot have changed.
     * Its bans must not, or a robot that refused once -- from wherever it happened to be standing
     * at the time -- is excluded from that link for the rest of the run, and when it later settles
     * onto the very site the link points at the offerer skips the occupant and sends somebody else
     * to a spot that is already taken.
     */
    @Test
    @DisplayName("a link survives its walk and forgets that walk's exclusions")
    void linksSurviveTheirWalk() {
        List<GeometricCycleLatticeRobot> robots = neighbourhood();
        List<TickRecord> records = LatticeHarness.tick(robots, LONG_RUN);

        boolean sawBoundSurvivor = false;
        for (GeometricCycleLatticeRobot robot : robots) {
            for (FaceObligation link : LatticeHarness.lastRecordOf(records, robot.getRobotId())
                    .after().obligations()) {
                if (link.getParentId() == FaceObligation.NO_PARENT) {
                    continue;
                }
                sawBoundSurvivor |= link.getChildId() != null;
                assertTrue(link.getBans().isEmpty(),
                        "robot " + robot.getRobotId() + " kept bans " + link.getBans()
                                + " on the link for edge " + link.getEdgeId()
                                + " after its walk resolved. A ban is scoped to one walk; kept on a "
                                + "permanent link it eventually hides the robot standing on the "
                                + "target site.");
            }
        }
        assertTrue(sawBoundSurvivor,
                "no robot ended the run holding a bound communication link, though walks passed "
                        + "through them. A resolved status must leave the link and its binding in "
                        + "place -- that is what lets the next certificate over that edge go "
                        + "straight to the same child.");
    }

    /**
     * One drawn edge per neighbour, however many walks cross the link.
     *
     * <p>Three sites draw an edge back toward a parent -- accepting a first assignment, relaying,
     * and closing a face -- and each used to run about once per walk, when a tuple lived and died
     * with one. Permanent links made them run on every certificate that crosses, so the pile grew
     * without bound over a long run.
     *
     * <p>The guard against that was always present and always dead:
     * {@code GeometricCycleLatticeRobot.addEdge} reads {@code if(!edges.contains(e)) edges.add(e)},
     * a set-add spelled as a list-add -- but {@link org.simulation.Edge} had no {@code equals}, so
     * {@code contains} compared references, every caller handed it a fresh instance, and the check
     * never once fired. This asserts against {@code addEdge} rather than against any helper on top
     * of it, because that is where the property belongs.
     */
    @Test
    @DisplayName("a link carrying many walks draws exactly one edge to its neighbour")
    void manyWalksDrawOneEdge() {
        List<GeometricCycleLatticeRobot> robots = neighbourhood();
        LatticeHarness.tick(robots, LONG_RUN);

        for (GeometricCycleLatticeRobot robot : robots) {
            List<org.simulation.Edge> drawn = List.copyOf(robot.getEdges());
            // Compared as endpoint pairs, deliberately, and NOT as a HashSet<Edge>: the equality
            // under test is the thing that would make such a set collapse duplicates, so using it
            // here would make this assertion true by construction whether or not it holds.
            Set<String> pairs = new java.util.HashSet<>();
            for (org.simulation.Edge edge : drawn) {
                pairs.add(edge.getFromId() + "->" + edge.getToId());
            }
            assertEquals(pairs.size(), drawn.size(),
                    "robot " + robot.getRobotId() + " is holding " + drawn.size() + " drawn edges "
                            + "covering only " + pairs.size() + " neighbour pairs after " + LONG_RUN
                            + " ticks: " + drawn + ". Links are permanent and carry many walks, so "
                            + "every draw site now runs per certificate rather than per walk -- "
                            + "addEdge has to actually deduplicate, which needs Edge to carry value "
                            + "equality.");
        }
    }

    /**
     * A duplicate certificate arriving on the very corner this root is building is relayed, not
     * refused and not held. Exactly the case the old arbitration fired on.
     */
    @Test
    @DisplayName("a walk owing the corner this root is building is carried, not refused")
    void aWalkOnTheCornerIAmBuildingIsCarried() {
        List<GeometricCycleLatticeRobot> robots = neighbourhood();
        GeometricCycleLatticeRobot seed = robots.get(0);

        FaceObligation attempt = attemptInFlight(seed);
        assertNotNull(attempt, "scenario error: the seed never got a walk of its own in flight");

        HalfEdge corner = SQUARE.getHalfEdgeById(attempt.getEdgeId());
        HalfEdge incoming = arrivesOwing(corner);
        assertNotNull(incoming, "scenario error: no incoming edge owes corner " + corner.getId());
        GeometricCycleLatticeRobot sender = occupantOf(robots, incoming.getTwin());
        assertNotNull(sender, "scenario error: nobody is standing where that walk must come from");

        seed.enqueueMessage(new PositioningMessage(sender.getRobotId(), seed.getRobotId(),
                incoming.getOrigin().getId(), incoming.getId(),
                incoming.getOrigin().getId(), incoming.getId(),
                new VoltageCertificate(777)));

        TickRecord onArrival = seed.executeTimeStep(1.0, 200);
        assertFalse(onArrival.processedDescription().contains("REJECTED"),
                "the seed refused a walk from a robot standing exactly on the site it was offered: "
                        + onArrival.processedDescription() + ". A rejection is non-retryable, so "
                        + "the offerer bans the one robot that can serve that site and then works "
                        + "through candidates that are all guaranteed to refuse.");

        boolean carried = false;
        for (int tick = 201; tick <= 240 && !carried; tick++) {
            for (FaceObligation o : seed.executeTimeStep(1.0, tick).after().obligations()) {
                carried |= o.getParentId() == sender.getRobotId() && o.getChildId() != null;
            }
        }
        assertTrue(carried,
                "the seed accepted the walk and then never carried it onward. Sitting on it until "
                        + "its own attempt resolved was tried and deadlocks: duplicate walks share "
                        + "every link along the face, so holding one holds the other -- including "
                        + "this robot's own returning certificate.");
    }

    private static boolean hasEdgeTo(GeometricCycleLatticeRobot from, int toId) {
        return from.getEdges().stream()
                .anyMatch(e -> e.getFromId() == from.getRobotId() && e.getToId() == toId);
    }

    /**
     * Refusing a futile offer does not tear down a real connection.
     *
     * <p>A root opens a corner nobody occupies, so candidate selection falls through to nearest and
     * offers the site to a neighbour that is standing correctly somewhere else. The neighbour
     * refuses, as it must. What must <em>not</em> happen is the offerer deleting the edge between
     * them, because the two have a permanent communication link and have been relaying for each
     * other -- the refusal was about one site, not about whether they can talk.
     *
     * <p>The failure this pins was subtle and self-inflicted: giving {@code Edge} value equality
     * made {@code addEdge} deduplicate, so {@code offer}'s freshly built instance was discarded
     * while the obligation stored it, and removing that stored object matched the surviving good
     * edge by value and deleted it instead.
     */
    @Test
    @DisplayName("a refusal from a permanently linked neighbour keeps the edge to it")
    void aRefusalFromALinkedNeighbourKeepsTheEdge() {
        List<GeometricCycleLatticeRobot> robots = neighbourhood();
        GeometricCycleLatticeRobot seed = robots.get(0);

        FaceObligation attempt = attemptInFlight(seed);
        assertNotNull(attempt, "scenario error: the seed never got a walk of its own in flight");
        int refuser = attempt.getChildId();
        assertTrue(hasEdgeTo(seed, refuser), "scenario error: the offer should have drawn an edge");

        // Give the seed a permanent link on which the refuser is its PARENT -- the refuser is
        // standing at the far end of the corner being built, so a walk from it arrives over that
        // corner's twin, which is the one edge whose position check the seed passes.
        HalfEdge incoming = SQUARE.getHalfEdgeById(attempt.getEdgeId()).getTwin();
        seed.enqueueMessage(new PositioningMessage(refuser, seed.getRobotId(),
                incoming.getOrigin().getId(), incoming.getId(),
                incoming.getOrigin().getId(), incoming.getId(),
                new VoltageCertificate(777)));
        boolean linked = false;
        for (int tick = 60; tick <= 80 && !linked; tick++) {
            for (FaceObligation o : seed.executeTimeStep(1.0, tick).after().obligations()) {
                linked |= o.getParentId() == refuser;
            }
        }
        assertTrue(linked, "scenario error: the seed never opened a link with the refuser as parent");

        // Now the refusal, for the seed's OWN walk -- so it routes through the attempt, not through
        // the link just built.
        seed.enqueueMessage(new RejectAssignmentMessage(refuser, seed.getRobotId(),
                SQUARE.getPrimaryRole().getId(), attempt.getEdgeId(), false,
                new VoltageCertificate(seed.getRobotId())));
        seed.executeTimeStep(1.0, 100);

        assertTrue(hasEdgeTo(seed, refuser),
                "the seed deleted its edge to robot " + refuser + " because that robot refused a "
                        + "site it could not take -- even though a permanent link still connects "
                        + "them and they have been relaying for each other. An edge is a claim "
                        + "about topology; one failed offer does not falsify it.");
    }

    /** The other half: a refusal from a robot nothing connects us to does drop the edge. */
    @Test
    @DisplayName("a refusal from an unlinked robot drops the edge to it")
    void aRefusalFromAnUnlinkedRobotDropsTheEdge() {
        // A lone root and one robot loitering at a pose that is NOT a lattice site of the root --
        // 25 units out on the diagonal, where the square lattice's neighbours sit at 70 along the
        // axes. That distinction is the whole test: placeAroundRole cannot be used here, because
        // every robot in it stands on one of the seed's sites and so is a genuine neighbour whose
        // edge must survive a refusal.
        GeometricCycleLatticeRobot seed =
                new GeometricCycleLatticeRobot(0, new OrientedPoint(0, 0, 0), SQUARE);
        GeometricCycleLatticeRobot passerBy =
                new GeometricCycleLatticeRobot(1, new OrientedPoint(25, 15, 0), SQUARE);
        LatticeHarness.makeAllNeighbors(List.of(seed, passerBy));
        seed.promoteToPrimaryRoot();

        FaceObligation attempt = attemptInFlight(seed);
        assertNotNull(attempt, "scenario error: the seed never got a walk of its own in flight");
        int refuser = attempt.getChildId();
        assertEquals(passerBy.getRobotId(), refuser,
                "scenario error: the only candidate should have been the loitering robot");
        assertTrue(hasEdgeTo(seed, refuser), "scenario error: the offer should have drawn an edge");

        seed.enqueueMessage(new RejectAssignmentMessage(refuser, seed.getRobotId(),
                SQUARE.getPrimaryRole().getId(), attempt.getEdgeId(), false,
                new VoltageCertificate(seed.getRobotId())));
        seed.executeTimeStep(1.0, 100);

        assertFalse(hasEdgeTo(seed, refuser),
                "the edge to robot " + refuser + " survived a refusal, though it holds no link with "
                        + "this robot and is not standing on any of its lattice sites. Nothing but "
                        + "the transient attempt ever connected the two, and an attempt that ends "
                        + "leaves no claim behind it.");
    }

    /**
     * The reported case: a root must not lose the edge to a robot <em>it placed itself</em>.
     *
     * <p>The root opens a corner nobody occupies, so candidate selection falls through to nearest
     * and the site goes to a robot that accepts it. That robot arrives and then dead-ends, because
     * there is no one beyond it; the walk fails and the root clears its attempt. The root opens its
     * <em>next</em> corner, picks the same robot again -- still the nearest thing it can see -- and
     * is correctly refused, because that robot is standing on the first corner, not the second.
     *
     * <p>A carried link cannot vouch for it. Links are created when a walk <em>arrives</em>, and the
     * root is the one that sent, so no link at the root ever names the robot it placed. The only
     * record was the attempt, and the attempt is gone by then. Without
     * {@code occupiesAdjacentSite} the edge is deleted between two robots that are lattice
     * neighbours and know it.
     */
    @Test
    @DisplayName("a root keeps the edge to a robot it placed, even after that walk dead-ends")
    void aRootKeepsTheEdgeToTheRobotItPlaced() {
        // The root, one robot standing exactly on one of its corners, and nothing beyond -- so any
        // walk over that corner has nowhere to go and must dead-end.
        GeometricCycleLatticeRobot seed =
                new GeometricCycleLatticeRobot(0, new OrientedPoint(0, 0, 0), SQUARE);
        HalfEdge placedCorner = SQUARE.getOutgoingHalfEdges(SQUARE.getPrimaryRole()).get(0);
        OrientedPoint site = new RigidBodyTransformation(seed.getPosition())
                .compose(placedCorner.getVoltage()).asPose();
        GeometricCycleLatticeRobot placed = new GeometricCycleLatticeRobot(1, site, SQUARE);
        LatticeHarness.makeAllNeighbors(List.of(seed, placed));
        seed.promoteToPrimaryRoot();

        // Run both robots so the placed one actually relays, dead-ends, and reports back.
        List<GeometricCycleLatticeRobot> both = List.of(seed, placed);
        LatticeHarness.tick(both, 60);

        assertTrue(hasEdgeTo(seed, placed.getRobotId()),
                "the root deleted its edge to robot " + placed.getRobotId() + ", the robot standing "
                        + "on its own corner and the one it placed there. No carried link names a "
                        + "robot this root assigned -- links record walks that ARRIVE -- so once "
                        + "the attempt cleared, only the fact that it is sitting on an adjacent "
                        + "lattice site says the two are still neighbours.");
    }

    /**
     * A departed child's edge goes, and the ordering inside the sweep is what makes it go.
     *
     * <p>The link itself survives a departure -- this robot and its parent have not moved -- so the
     * only thing that stops it vouching for the robot that left is {@code release()} clearing the
     * child slot. That release has to happen <em>before</em> the undraw asks
     * {@code isPermanentlyLinkedTo}. Undraw first and the link still names the departed robot, the
     * predicate says keep, and every robot that ever wandered off keeps its line forever.
     */
    @Test
    @DisplayName("a departed child loses its edge, though the link that named it survives")
    void aDepartedChildLosesItsEdge() {
        List<GeometricCycleLatticeRobot> robots = neighbourhood();
        GeometricCycleLatticeRobot seed = robots.get(0);

        // A carried link, so the departed robot is a link's child rather than the attempt's --
        // which is the case where the ordering is observable at all.
        FaceObligation attempt = attemptInFlight(seed);
        assertNotNull(attempt, "scenario error: the seed never got a walk of its own in flight");
        HalfEdge incoming = arrivesOwing(SQUARE.getHalfEdgeById(attempt.getEdgeId()));
        assertNotNull(incoming, "scenario error: no incoming edge owes that corner");
        GeometricCycleLatticeRobot sender = occupantOf(robots, incoming.getTwin());
        assertNotNull(sender, "scenario error: nobody stands where that walk must come from");

        seed.enqueueMessage(new PositioningMessage(sender.getRobotId(), seed.getRobotId(),
                incoming.getOrigin().getId(), incoming.getId(),
                incoming.getOrigin().getId(), incoming.getId(),
                new VoltageCertificate(888)));
        Integer carriedChild = null;
        for (int tick = 60; tick <= 80 && carriedChild == null; tick++) {
            for (FaceObligation o : seed.executeTimeStep(1.0, tick).after().obligations()) {
                if (o.getParentId() == sender.getRobotId() && o.getChildId() != null) {
                    carriedChild = o.getChildId();
                }
            }
        }
        assertNotNull(carriedChild, "scenario error: the seed never carried that walk onward");
        assertTrue(hasEdgeTo(seed, carriedChild), "scenario error: carrying should have drawn an edge");

        // The child walks out of range.
        final int gone = carriedChild;
        seed.clearNeighbors();
        for (GeometricCycleLatticeRobot other : robots) {
            if (other.getRobotId() != seed.getRobotId() && other.getRobotId() != gone) {
                seed.addNeighbor(other);
            }
        }
        seed.executeTimeStep(1.0, 100);

        assertFalse(hasEdgeTo(seed, gone),
                "robot " + gone + " left range and kept its edge. The sweep releases the link's "
                        + "child slot and only then undraws; if those run the other way round the "
                        + "link still vouches for the robot that left and the line never goes.");
    }
}
