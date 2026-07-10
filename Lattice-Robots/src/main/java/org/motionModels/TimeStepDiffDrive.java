package org.motionModels;

import org.graphs.OrientedPoint;
import org.robots.LatticeRobot;
import org.utils.MathUtils;

/**
 * A class that represents a differential drive motion model for a robot, breaking down motions into discrete time steps
 */
public class TimeStepDiffDrive extends MotionModel implements LatticeMotionModel{
    private enum MoveState {
        ROTATE_TO_POINT,
        TRANSLATE,
        ROTATE_TO_FINAL,
        DONE
    }
    private double leftAngularVel;
    private double rightAngularVel;
    private MoveState moveState = MoveState.DONE;
    private double timeElapsed;
    
    public static final double MAX_LINEAR_SPEED = 10.0;
    private static final double WHEEL_RADIUS = 4;
    private static final double DISTANCE_BETWEEN_WHEELS = 30;
    private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / WHEEL_RADIUS;
    private final double TIME_TO_ESCAPE_CONGESTION;
    public static final double ASSIGNMENT_CHANGE_THRESHOLD = MAX_LINEAR_SPEED * 0.5;

    public TimeStepDiffDrive() {
        super();
        this.timeElapsed = 0.0;
        this.leftAngularVel = 0.0;
        this.rightAngularVel = 0.0;
        TIME_TO_ESCAPE_CONGESTION = 2* LatticeRobot.COMM_RANGE / MAX_LINEAR_SPEED;
    }

    public static double getTimeToRotateTo(OrientedPoint currentPose, OrientedPoint newPose) {
        double targetAngle = Math.atan2(newPose.y - currentPose.y, newPose.x - currentPose.x);
        double rotateBy = MathUtils.normalizeAngle(targetAngle - currentPose.orientation);
        return Math.abs(rotateBy) / MAX_ANGULAR_SPEED;
    }

    public boolean move(OrientedPoint pose, double dt) {
        if(Double.compare(timeElapsed, TIME_TO_ESCAPE_CONGESTION) >= 0) {
            moveState = MoveState.DONE;
            timeElapsed = 0.0;
            changeState(0, 0);
            return true;
        }

        moveState = MoveState.TRANSLATE;

        changeState(MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

        pose.x = pose.x + (WHEEL_RADIUS / 2) * (leftAngularVel + rightAngularVel) * Math.cos(pose.orientation) * dt;
        pose.y = pose.y + (WHEEL_RADIUS / 2) * (leftAngularVel + rightAngularVel) * Math.sin(pose.orientation) * dt;
        if(leftAngularVel != rightAngularVel) {
            pose.orientation = MathUtils.normalizeAngle(pose.orientation + (WHEEL_RADIUS / DISTANCE_BETWEEN_WHEELS) * (rightAngularVel - leftAngularVel) * dt);    
        }

        distTraveled = distTraveled + MAX_LINEAR_SPEED * dt;
        timeElapsed += dt;
        return false;
    }

    public OrientedPoint getIntermediatePose(OrientedPoint currentPose, OrientedPoint parentPose, OrientedPoint target, double timeStep) {
        double r1 = MAX_LINEAR_SPEED * timeStep;
        double r2 = LatticeRobot.COMM_RANGE - MAX_LINEAR_SPEED * timeStep;
        
        OrientedPoint candidateForSelfDisk = projectTargetToReachableDisk(currentPose, target, r1);

        OrientedPoint candidateForParentDisk = projectTargetToReachableDisk(parentPose, candidateForSelfDisk, r2);

        return candidateForParentDisk;
    }

    private OrientedPoint projectTargetToReachableDisk(OrientedPoint current, OrientedPoint target, double radius) {
        double distance = current.distance(target);

        if(distance < radius) {
            return target;
        }

        double scale = radius / distance;
        double dx = target.x - current.x;
        double dy = target.y - current.y;

        double x = current.x + dx * scale;
        double y = current.y + dy * scale;

        double angle = Math.atan2(target.y - y, target.x - x);

        return new OrientedPoint(x,y, angle);

    }

    public boolean moveTo(OrientedPoint currentPose, OrientedPoint newPose, double dt) {
        moveState = checkNextMoveState(currentPose, newPose);
 
        switch(moveState) {

            case ROTATE_TO_POINT:
                rotateTo(currentPose, newPose, dt);
            break;

            case TRANSLATE:
                translateTo(currentPose, newPose, dt);
            break;

            case ROTATE_TO_FINAL:
                rotateTo(currentPose, newPose.orientation, dt);
            break;

            case DONE: {
                            
                currentPose.x = newPose.x;
                currentPose.y = newPose.y;
                currentPose.orientation = MathUtils.normalizeAngle(newPose.orientation);
                changeState(0, 0);
                return true;
            }
        }

        return false;
    }

    public static MoveState checkNextMoveState(OrientedPoint currentPose, OrientedPoint newPose) {
        
        if(!MathUtils.isZero(currentPose.distance(newPose))) {
            double targetHeading = Math.atan2(newPose.y - currentPose.y, newPose.x - currentPose.x);
            if(!MathUtils.anglesEqual(targetHeading, currentPose.orientation)) {
                return MoveState.ROTATE_TO_POINT;
            }
            return MoveState.TRANSLATE;
        } else {
            if(!MathUtils.anglesEqual(newPose.orientation, currentPose.orientation)) {
                return MoveState.ROTATE_TO_FINAL;
            }
            return MoveState.DONE;
        }
    }

    private boolean rotateTo(OrientedPoint pose, OrientedPoint newPose, double dt) {
        double targetAngle = Math.atan2(newPose.y - pose.y, newPose.x - pose.x);

        return rotateTo(pose, targetAngle, dt);
    }

    private boolean translateTo(OrientedPoint pose, OrientedPoint newPose, double dt) {
        double distance = pose.distance(newPose);

        if(MathUtils.isZero(distance)) {
            pose.x = newPose.x;
            pose.y = newPose.y;
            changeState(0, 0);
            return true;
        }

        changeState(MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

        double linearVelocity = (WHEEL_RADIUS / 2.0) * (leftAngularVel + rightAngularVel);

        double step = linearVelocity * dt;

        if(step >= distance) {
            pose.x = newPose.x;
            pose.y = newPose.y;
            changeState(0, 0);
            distTraveled = distTraveled + distance;
            return true;
        }

        pose.x += step * Math.cos(pose.orientation);
        pose.y += step * Math.sin(pose.orientation);
        distTraveled = distTraveled + linearVelocity * dt;

        return false;
    }
    
    private boolean rotateTo(OrientedPoint pose, double targetAngle, double dt) {
        double rotateBy = MathUtils.normalizeAngle(targetAngle - pose.orientation);
        // tolerance
        if(MathUtils.isZero(rotateBy)) {
            pose.orientation = targetAngle;
            changeState(0, 0);
            return true;
        }

        if(rotateBy > 0.0) {
            changeState(-MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
        } else {
            changeState(MAX_ANGULAR_SPEED, -MAX_ANGULAR_SPEED);
        }

        double angularStep = (WHEEL_RADIUS / DISTANCE_BETWEEN_WHEELS) * (rightAngularVel - leftAngularVel) * dt;

        // prevent overshoot
        if(Math.abs(angularStep) > Math.abs(rotateBy)) {
            angularStep = rotateBy;
        }

        pose.orientation = MathUtils.normalizeAngle(pose.orientation + angularStep);

        return false;
    }
    
    private void changeState(double leftAngVel, double rightAngVel) {
        this.leftAngularVel = leftAngVel;
        this.rightAngularVel = rightAngVel;
    }
}
