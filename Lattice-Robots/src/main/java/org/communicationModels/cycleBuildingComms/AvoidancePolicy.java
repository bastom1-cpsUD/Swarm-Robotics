package org.communicationModels.cycleBuildingComms;

import java.util.HashMap;
import java.util.Map;

import org.communicationModels.Observation;
import org.communicationModels.cycleBuildingComms.Messages.TargetClaimMessage;
import org.graphs.util.OrientedPoint;
import org.graphs.util.RigidBodyTransformation;
import org.robots.AvoidanceState;
import org.robots.GeometricCycleLatticeRobot;
import org.utils.AvoidanceGeometry;
import org.utils.MathUtils;

/**
 * Tick-rate collision policy: decides <em>where</em> a robot should aim this tick when
 * something is in its way, and who should defer to whom.
 *
 * <p>This is the layer above the hard guard, not a replacement for it. The guarantee that
 * bodies never overlap lives entirely in
 * {@code GeometricCycleLatticeRobot.move(double)}, runs at motion rate, and depends on no
 * classification, no message and no state held here. Everything in this class only ever
 * chooses between <em>stall</em> and <em>route around</em> — getting it wrong costs a
 * wasted arc, never a collision. That separation is what lets the classifier below be a
 * heuristic without endangering anything.
 *
 * <p>Owned by composition rather than inlined into {@code CyclebuilderComms}, which is
 * already past 1300 lines and has nothing to do with planar motion planning.
 *
 * <h3>Threading</h3>
 * Every field is touched only from this robot's own tick task, which holds the read lock.
 * The one value read from another thread — the planned waypoint — is handed off through
 * {@code GeometricCycleLatticeRobot.assignedPosition}, which is volatile for that reason.
 */
public final class AvoidancePolicy {

    /**
     * Displacement, in world units, below which a neighbour counts as not having moved
     * this tick. Five percent of one tick's full-speed travel: five hundred times the
     * {@code EPSILON}-bounded arrival snap, and far below any genuine motion.
     */
    private static final double STATIONARY_DISPLACEMENT = 0.5;

    /**
     * Consecutive quiet ticks required before a neighbour is believed to be stationary.
     * More than one because a robot in {@code ROTATE_TO_POINT} translates nothing, and a
     * single sample cannot tell that apart from parked.
     */
    private static final int STATIONARY_CONFIRM_TICKS = 2;

    /**
     * How long a chosen detour side is held before being re-evaluated from scratch.
     *
     * <p>The latch is load-bearing rather than an optimisation. The two candidate sides
     * score almost identically precisely when the obstacle sits on the straight line to
     * the target — which is the case detours exist for — so without hysteresis the choice
     * flips every tick and the robot oscillates in place instead of going around.
     */
    private static final int DETOUR_LATCH_TTL_TICKS = 20;

    /**
     * Slack subtracted from {@code COMM_RANGE} to get the tether radius. Small
     * deliberately: the tether disk is convex, so no allowance for this robot's own
     * motion is required (see {@code AvoidanceGeometry.withinTether}). This covers only
     * float noise and a root being dragged by the mouse.
     *
     * <p>It must stay small for a second reason: a child's own assigned target sits one
     * lattice edge from its parent — 70 on SnubSquare — so a tether radius below that
     * would make the robot's own destination unreachable.
     */
    private static final double TETHER_SAFETY = 1.0;

    /**
     * How long a robot stays committed to stepping aside, refreshed by every fresh
     * request. Deliberately longer than the beacon TTL that carries the request: a
     * commitment only two ticks long would have the evader start, stop, and achieve
     * nothing.
     */
    private static final int EVASION_TTL_TICKS = 6;

    /**
     * Minimum ticks between re-aiming an evasion already under way. Without this, two
     * builders taking turns ping-pong a single evader between two sidesteps forever and it
     * never actually leaves either corridor.
     */
    private static final int EVASION_REAIM_MIN_TICKS = 2;

    /**
     * How close to a corridor's centre line counts as being <em>on</em> it, with no side
     * of its own to escape toward.
     *
     * <p>Well above float noise and well below the body radius, deliberately. Using a
     * tolerance rather than an exact test matters because just off the line the outward
     * direction is geometrically defined but physically meaningless — it would be chosen
     * by rounding error, and could flip between ticks.
     */
    private static final double ON_CORRIDOR_TOLERANCE = 1.0;

    private final GeometricCycleLatticeRobot self;

    // --- stationarity classifier -------------------------------------------------
    /** Last tick's observations, in the frame this robot occupied at that time. */
    private final Map<Integer, OrientedPoint> previousObserved = new HashMap<>();
    /** Consecutive ticks each neighbour has been seen not to move. */
    private final Map<Integer, Integer> quietTicks = new HashMap<>();
    private OrientedPoint previousSelfPose;

    // --- detour latch --------------------------------------------------------------
    private int detourObstacleId = -1;
    private int detourSign = 0;
    private int detourAgeTicks = 0;

    // --- stand-aside: emitting -----------------------------------------------------
    /** Who this robot is asking to move, published on the next beacon. */
    private int standAsideRequest = TargetClaimMessage.NO_REQUEST;

    // --- stand-aside: honouring ----------------------------------------------------
    /**
     * Where this robot has been asked to get out of the way to, in the global frame; null
     * when not evading. Global rather than local because a local target re-applied
     * through a moving self-frame each tick would run away from the robot. Same
     * convention {@code assignedPosition} already uses; on hardware it would be
     * odometry-integrated.
     */
    private OrientedPoint evasionTarget;
    private int evasionTicksRemaining = 0;
    private int evasionAgeTicks = 0;

    // --- reporting (overlay / tick log only) ---------------------------------------
    private AvoidanceState state = AvoidanceState.CLEAR;
    private int blockingObstacleId = -1;

    public AvoidancePolicy(GeometricCycleLatticeRobot self) {
        this.self = self;
    }

    // ---------------------------------------------------------------------------
    // Stationarity
    // ---------------------------------------------------------------------------

    /**
     * Updates the moving/stationary classification from this tick's observations. Call
     * once per tick, immediately after observing and before anything reads the result.
     *
     * <p>Inferring motion from successive observations is legitimate here even though
     * {@code TargetClaimMessage}'s javadoc argues against inference. That argument is
     * about <em>intent</em>: no sensor reveals it, and two robots inferring each other's
     * intent test it against <em>different</em> targets, so the verdicts need not agree.
     * Neither objection applies to current motion state. Displacement is exactly what the
     * position sensor already reports over two samples, and "did that body move" is a
     * property of the body rather than of the observer's goal, so two robots watching each
     * other reach the same verdict.
     *
     * <p>The trap this has to dodge is ego-motion. An {@code Observation} is expressed in
     * <em>this robot's current frame</em>, so a perfectly parked neighbour appears to move
     * whenever this robot does; differencing raw observations would classify everything as
     * moving forever. Both samples are therefore brought into the frame this robot
     * occupied last tick before being compared, using its own odometry — which is what a
     * real platform would do.
     *
     * <p>The result is deliberately a <em>lagging</em> indicator: a robot that has just
     * begun rotating toward a new heading reads as stationary and will be moving within a
     * couple of ticks. That is acceptable because safety never depends on it.
     */
    public void observeMotion(Map<Integer, Observation> observed) {
        OrientedPoint here = new OrientedPoint(self.getPosition());

        if (previousSelfPose != null) {
            // T_prev^-1 * T_now: re-expresses a point given in this tick's frame in the
            // frame this robot occupied last tick.
            RigidBodyTransformation nowIntoPrevFrame =
                    new RigidBodyTransformation(previousSelfPose, here);

            for (Observation obs : observed.values()) {
                int id = obs.getId();
                OrientedPoint before = previousObserved.get(id);
                if (before == null) {
                    // First sighting -- no displacement can be computed yet.
                    quietTicks.remove(id);
                    continue;
                }
                OrientedPoint nowInPrevFrame = nowIntoPrevFrame.apply(obs.getLocalPosition());
                if (before.distance(nowInPrevFrame) < STATIONARY_DISPLACEMENT) {
                    quietTicks.merge(id, 1, Integer::sum);
                } else {
                    quietTicks.put(id, 0);
                }
            }
        }

        previousObserved.clear();
        for (Observation obs : observed.values()) {
            previousObserved.put(obs.getId(), new OrientedPoint(obs.getLocalPosition()));
        }
        // Forget robots that have left range, so a stale "stationary" verdict cannot
        // survive to be applied to a robot that has since come back moving.
        quietTicks.keySet().retainAll(previousObserved.keySet());

        previousSelfPose = here;
    }

    /** Whether a neighbour has been seen not to move for long enough to be believed. */
    public boolean isStationary(int robotId) {
        Integer quiet = quietTicks.get(robotId);
        return quiet != null && quiet >= STATIONARY_CONFIRM_TICKS;
    }

    // ---------------------------------------------------------------------------
    // Planning
    // ---------------------------------------------------------------------------

    /**
     * The pose this robot should actually drive at this tick.
     *
     * <p>The precedence ladder is three branches, not four. "Anchored" and "idle" are not
     * distinguished, because this robot never senses another's role — it simply detours,
     * and (from the next group onward) asks the blocker to move, leaving the recipient to
     * decide whether it can honour that. Deleting the unobservable predicate is the point.
     *
     * <ol>
     *   <li><b>Blocker stationary</b> — route around it.</li>
     *   <li><b>Blocker moving, lower id</b> — hold and let it pass.</li>
     *   <li><b>Blocker moving, higher id</b> — carry on; it defers. The hard guard still
     *       covers the case where it does not.</li>
     * </ol>
     *
     * <p>Rules 2 and 3 mirror the shape of {@code CyclebuilderComms.outranks} — possession
     * first, then lower id — so the collision layer and the contention layer never
     * disagree about who defers.
     *
     * @param trueTarget the lattice target, or wherever the robot is holding
     * @param parent     the parent's pose, or null when there is no link to preserve
     * @return the waypoint to drive at; never null when {@code trueTarget} is non-null
     */
    public OrientedPoint planWaypoint(OrientedPoint trueTarget, OrientedPoint parent) {
        standAsideRequest = TargetClaimMessage.NO_REQUEST;

        if (trueTarget == null) {
            clearDetour();
            state = AvoidanceState.CLEAR;
            blockingObstacleId = -1;
            return null;
        }

        final OrientedPoint here = self.getPosition();
        final double keepOut = GeometricCycleLatticeRobot.KEEP_OUT;

        GeometricCycleLatticeRobot blocker = nearestBlocker(here, trueTarget, keepOut);
        if (blocker == null) {
            clearDetour();
            state = AvoidanceState.CLEAR;
            blockingObstacleId = -1;
            return trueTarget;
        }

        blockingObstacleId = blocker.getRobotId();

        if (!isStationary(blocker.getRobotId())) {
            clearDetour();
            if (blocker.getRobotId() < self.getRobotId()) {
                state = AvoidanceState.HOLDING;
                return new OrientedPoint(here);
            }
            state = AvoidanceState.CLEAR;
            return trueTarget;
        }

        final OrientedPoint obstacle = blocker.getPosition();
        final double reach = Math.min(here.distance(trueTarget),
                                      GeometricCycleLatticeRobot.tickTravel());

        // Standing on the spot this robot has to end up in. No arc fixes that -- go all the
        // way round and the last step in is still refused by the guard -- so the only way
        // through is for the blocker to move. Ask, and keep asking while it is true.
        if (obstacle.distance(trueTarget) < keepOut) {
            standAsideRequest = blocker.getRobotId();
        }

        // Already inside the keep-out circle: the tangent candidates degenerate into pure
        // sidesteps that do not increase separation, so the guard would refuse them. Push
        // straight out instead -- that step can never be vetoed.
        if (here.distance(obstacle) <= keepOut) {
            state = AvoidanceState.DETOURING;
            return AvoidanceGeometry.radialEscape(here, obstacle, reach, self.getRobotId());
        }

        int sign = chooseSide(here, obstacle, trueTarget, parent, keepOut, reach,
                              blocker.getRobotId());
        if (sign == 0) {
            // Neither way round keeps the parent in range, so no detour exists. Ask the
            // blocker to move; holding is the honest answer until it does, and the
            // liveness ladder is what eventually breaks the deadlock if it never does.
            standAsideRequest = blocker.getRobotId();
            state = AvoidanceState.HOLDING;
            return new OrientedPoint(here);
        }

        state = AvoidanceState.DETOURING;
        return AvoidanceGeometry.tangentWaypoint(here, obstacle, keepOut, reach, sign);
    }

    /**
     * The nearest neighbour standing on the straight path to the target.
     *
     * <p>Only the nearest is planned against, and the hard guard absorbs the rest.
     * Planning against a whole set would need a configuration-space search, which this
     * architecture deliberately does not attempt.
     */
    private GeometricCycleLatticeRobot nearestBlocker(OrientedPoint here,
                                                      OrientedPoint target,
                                                      double keepOut) {
        GeometricCycleLatticeRobot best = null;
        double bestDistance = Double.POSITIVE_INFINITY;

        for (GeometricCycleLatticeRobot other : self.getNeighbors()) {
            OrientedPoint obstacle = other.getPosition();
            if (!AvoidanceGeometry.blocks(here, target, obstacle, keepOut)) {
                continue;
            }
            double distance = here.distance(obstacle);
            // Ties broken on lower id so the choice cannot depend on neighbour ordering.
            if (distance < bestDistance
                    || (distance == bestDistance && other.getRobotId() < best.getRobotId())) {
                bestDistance = distance;
                best = other;
            }
        }
        return best;
    }

    /**
     * Which way round the obstacle to go: +1, -1, or 0 for "neither is possible".
     *
     * <p>Both candidates are one {@code reach} from here by construction, so ranking them
     * by remaining distance to the target is the same ordering as ranking by total path
     * length. A side that would break the parent link is rejected outright — and if only
     * the longer way round survives, the longer way round is taken. That is the whole of
     * the "stay within communication range of the parent" requirement; it falls out of
     * the filter rather than needing special handling.
     */
    private int chooseSide(OrientedPoint here, OrientedPoint obstacle, OrientedPoint target,
                           OrientedPoint parent, double keepOut, double reach, int obstacleId) {
        final double tether = GeometricCycleLatticeRobot.COMM_RANGE - TETHER_SAFETY;

        // Reuse the latched side while it is the same obstacle and still feasible.
        if (detourObstacleId == obstacleId && detourSign != 0
                && detourAgeTicks < DETOUR_LATCH_TTL_TICKS) {
            OrientedPoint latched =
                    AvoidanceGeometry.tangentWaypoint(here, obstacle, keepOut, reach, detourSign);
            if (AvoidanceGeometry.withinTether(latched, parent, tether)) {
                detourAgeTicks++;
                return detourSign;
            }
        }

        OrientedPoint counterClockwise =
                AvoidanceGeometry.tangentWaypoint(here, obstacle, keepOut, reach, +1);
        OrientedPoint clockwise =
                AvoidanceGeometry.tangentWaypoint(here, obstacle, keepOut, reach, -1);

        boolean ccwOk = AvoidanceGeometry.withinTether(counterClockwise, parent, tether);
        boolean cwOk  = AvoidanceGeometry.withinTether(clockwise, parent, tether);

        int chosen;
        if (ccwOk && cwOk) {
            chosen = counterClockwise.distance(target) <= clockwise.distance(target) ? +1 : -1;
        } else if (ccwOk) {
            chosen = +1;
        } else if (cwOk) {
            chosen = -1;
        } else {
            clearDetour();
            return 0;
        }

        detourObstacleId = obstacleId;
        detourSign = chosen;
        detourAgeTicks = 0;
        return chosen;
    }

    /**
     * Drops any latched detour. Called whenever the path is clear, and from every role
     * reset — an assignment change invalidates the target the latch was chosen against,
     * and a stale latch would otherwise keep steering around an obstacle that is no longer
     * in the way.
     */
    public void clearDetour() {
        detourObstacleId = -1;
        detourSign = 0;
        detourAgeTicks = 0;
    }

    // ---------------------------------------------------------------------------
    // Stand-aside: honouring a request
    // ---------------------------------------------------------------------------

    /** Who this robot is asking to move, for the next beacon. {@code NO_REQUEST} if nobody. */
    public int standAsideRequest() {
        return standAsideRequest;
    }

    /**
     * Accepts a request to get out of a neighbour's way, and picks somewhere to go.
     *
     * <p>Everything here is in this robot's own frame, with itself at the origin — the
     * corridor is reconstructed from a sensed observation of the requester composed with
     * the target the requester declared in <em>its</em> frame, so no shared origin is
     * assumed anywhere. Only the final answer is converted to global, for the reason given
     * on {@link #evasionTarget}.
     *
     * @param corridorStart the requester's observed pose, in this robot's frame
     * @param corridorEnd   where the requester is heading, in this robot's frame
     * @param observed      this tick's observations, used to avoid stepping into trouble
     * @param askerId       the requester, excluded from the crowding score since it is
     *                      about to move along the corridor anyway
     */
    public void onStandAside(OrientedPoint corridorStart, OrientedPoint corridorEnd,
                             Map<Integer, Observation> observed, int askerId) {
        final OrientedPoint origin = new OrientedPoint(0, 0, 0);
        final double clearance = GeometricCycleLatticeRobot.KEEP_OUT
                + 0.5 * GeometricCycleLatticeRobot.BODY_RADIUS;

        // Already far enough off the path. Doing anything here would move the robot back
        // toward a corridor it is not in, which is the opposite of what was asked.
        double standoff = AvoidanceGeometry.distancePointToSegment(corridorStart, corridorEnd, origin);
        if (standoff >= clearance) {
            return;
        }

        evasionTicksRemaining = EVASION_TTL_TICKS;

        if (evasionTarget != null) {
            if (evasionAgeTicks < EVASION_REAIM_MIN_TICKS) {
                return;
            }
            // Already stepping aside somewhere that is clear of this new corridor too --
            // finish that move rather than starting a fresh one.
            OrientedPoint currentInMyFrame =
                    new RigidBodyTransformation(self.getPosition()).inverse().apply(evasionTarget);
            if (!AvoidanceGeometry.blocks(corridorStart, corridorEnd, currentInMyFrame,
                                          GeometricCycleLatticeRobot.KEEP_OUT)) {
                return;
            }
        }

        OrientedPoint best;
        if (standoff > ON_CORRIDOR_TOLERANCE) {
            // Off to one side already. Leave the way it is leaning -- straight outward
            // from the nearest point of the corridor. There is deliberately no choice of
            // side here: the other side is only reachable by crossing the corridor, which
            // would take the robot through the very path it was asked to clear.
            best = AvoidanceGeometry.escapeFromCorridor(origin, corridorStart, corridorEnd, clearance);
            if (best != null && !keepsSomeoneInRange(best, observed)) {
                best = null;
            }
        } else {
            // Squarely on the corridor, so neither side is "away" and both are equally
            // short. Only here is there a genuine choice, and crowding decides it.
            best = null;
            double bestScore = Double.NEGATIVE_INFINITY;
            for (int sign : new int[] { +1, -1 }) {
                OrientedPoint candidate = AvoidanceGeometry.sidestepOffCorridor(
                        origin, corridorStart, corridorEnd, clearance, sign);
                if (!keepsSomeoneInRange(candidate, observed)) {
                    continue;
                }
                double score = crowdingScore(candidate, observed, askerId);
                if (score > bestScore) {
                    bestScore = score;
                    best = candidate;
                }
            }
        }

        if (best == null) {
            // Nowhere sensible to go. Staying put is better than blundering somewhere
            // worse; the requester will detour or eventually give the spot up.
            return;
        }

        evasionTarget = new RigidBodyTransformation(self.getPosition()).apply(best);
        evasionAgeTicks = 0;
    }

    /**
     * How far the nearest other robot would be if this robot stood at {@code candidate}.
     *
     * <p>Maximising this is what stops the evader clearing one corridor by parking on
     * somebody else's lattice spot — which would trip the occupancy check in
     * {@code makeObservations} and cascade a spurious rejection through the protocol.
     */
    private double crowdingScore(OrientedPoint candidate, Map<Integer, Observation> observed,
                                 int askerId) {
        double nearest = Double.POSITIVE_INFINITY;
        for (Observation obs : observed.values()) {
            if (obs.getId() == askerId) {
                continue;
            }
            nearest = Math.min(nearest, candidate.distance(obs.getLocalPosition()));
        }
        return nearest;
    }

    /**
     * Whether at least one neighbour would still be within communication range. An evader
     * that walks out of the swarm has no parent link to pull it back and is lost for good.
     */
    private boolean keepsSomeoneInRange(OrientedPoint candidate, Map<Integer, Observation> observed) {
        for (Observation obs : observed.values()) {
            if (candidate.distance(obs.getLocalPosition()) <= GeometricCycleLatticeRobot.COMM_RANGE) {
                return true;
            }
        }
        return false;
    }

    /**
     * Ages the evasion commitment by one tick, retiring it on arrival or expiry. Call once
     * per tick, for every role — a robot that stops being unassigned mid-step still needs
     * its commitment cleaned up.
     */
    public void ageEvasion() {
        if (evasionTarget == null) {
            return;
        }
        evasionAgeTicks++;
        evasionTicksRemaining--;
        if (evasionTicksRemaining <= 0
                || self.getPosition().distance(evasionTarget) < MathUtils.EPSILON) {
            cancelEvasion();
        }
    }

    /** Where this robot is stepping aside to, in the global frame, or null. */
    public OrientedPoint evasionGlobalTarget() {
        return evasionTarget;
    }

    /** Abandons any stand-aside commitment. */
    public void cancelEvasion() {
        evasionTarget = null;
        evasionTicksRemaining = 0;
        evasionAgeTicks = 0;
    }

    /** What the policy decided this tick. Overlay and tick log only. */
    public AvoidanceState state() {
        if (evasionTarget != null && state == AvoidanceState.CLEAR) {
            return AvoidanceState.EVADING;
        }
        return state;
    }

    /** The neighbour currently in the way, or -1. Overlay and tick log only. */
    public int blockingObstacleId() {
        return blockingObstacleId;
    }
}
