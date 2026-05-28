package org.motionModels;

import org.graphs.OrientedPoint;

public class TimeStepDiffDrive extends MotionModel {
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
    
    private static final double MAX_LINEAR_SPEED = 10.0;
    private static final double WHEEL_RADIUS = 4;
    private static final double DISTANCE_BETWEEN_WHEELS = 30;
    private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / WHEEL_RADIUS;
    private final double TIME_TO_ESCAPE_CONGESTION;

    public TimeStepDiffDrive(double commRange) {
        super();
        this.timeElapsed = 0.0;
        this.leftAngularVel = 0.0;
        this.rightAngularVel = 0.0;
        TIME_TO_ESCAPE_CONGESTION = 2* commRange / MAX_LINEAR_SPEED;
    }

    public boolean move(OrientedPoint pose, double dt) {
        if(Double.compare(timeElapsed, TIME_TO_ESCAPE_CONGESTION) > 0) {
            timeElapsed = 0.0;
            changeState(0, 0);
            return true;
        }
        changeState(MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

        pose.x = pose.x + (WHEEL_RADIUS / 2) * (leftAngularVel + rightAngularVel) * Math.cos(pose.orientation) * dt;
        pose.y = pose.y + (WHEEL_RADIUS / 2) * (leftAngularVel + rightAngularVel) * Math.sin(pose.orientation) * dt;
        if(leftAngularVel != rightAngularVel) {
            pose.orientation = normalizeAngle(pose.orientation + (WHEEL_RADIUS / DISTANCE_BETWEEN_WHEELS) * (rightAngularVel - leftAngularVel) * dt);    
        }

        distTraveled = distTraveled + MAX_LINEAR_SPEED * dt;
        timeElapsed += dt;
        return false;
    }

    public boolean moveTo(OrientedPoint currentPose, OrientedPoint newPose, double dt) {
        switch(moveState) {

            case ROTATE_TO_POINT:
                if(rotateTo(currentPose, newPose, dt)) {
                    moveState = MoveState.TRANSLATE;
                }
                break;

            case TRANSLATE:
                if(translateTo(currentPose, newPose, dt)) {
                    moveState = MoveState.ROTATE_TO_FINAL;
                }
                break;

            case ROTATE_TO_FINAL:
                if(rotateTo(currentPose, newPose.orientation, dt)) {
                    moveState = MoveState.DONE;
                    return true;
                }
                break;

            case DONE:
                return true;
        }

        return false;
    }

    public void startMoving() {
        moveState = MoveState.ROTATE_TO_POINT;
    }

    private boolean rotateTo(OrientedPoint pose, OrientedPoint newPose, double dt) {
        double targetAngle = Math.atan2(newPose.y - pose.y, newPose.x - pose.x);

        return rotateTo(pose, targetAngle, dt);
    }

    private boolean translateTo(OrientedPoint pose, OrientedPoint newPose, double dt) {
        double distance = pose.distance(newPose);

        if(isZero(distance)) {
            pose.x = newPose.x;
            pose.y = newPose.y;
            changeState(0, 0);
            return true;
        }

        changeState(MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

        double linearVelocity = (WHEEL_RADIUS / 2.0) * (leftAngularVel + rightAngularVel);

        double step = linearVelocity * dt;

        if(step > distance) {
            step = distance;
        }

        pose.x += step * Math.cos(pose.orientation);
        pose.y += step * Math.sin(pose.orientation);
        distTraveled = distTraveled + linearVelocity * dt;

        return false;
    }
    
    private boolean rotateTo(OrientedPoint pose, double targetAngle, double dt) {
        double rotateBy = normalizeAngle(targetAngle - pose.orientation);
        // tolerance
        if(isZero(rotateBy)) {
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

        pose.orientation = normalizeAngle(pose.orientation + angularStep);

        return false;
    }
    
    private double normalizeAngle(double angle) {
        while(angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        while(angle < -Math.PI) {
            angle += 2 * Math.PI;
        }

        return angle;
    }

    private void changeState(double leftAngVel, double rightAngVel) {
        this.leftAngularVel = leftAngVel;
        this.rightAngularVel = rightAngVel;
    }

    private boolean isZero(double value) {
        return Math.abs(value) < 1e-9;
    }
}
