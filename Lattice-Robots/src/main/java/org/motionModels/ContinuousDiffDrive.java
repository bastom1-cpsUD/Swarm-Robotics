package org.motionModels;

import org.graphs.util.OrientedPoint;
import org.robots.LatticeRobot;

public class ContinuousDiffDrive extends MotionModel {
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

    public ContinuousDiffDrive(double commRange) {
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

    public OrientedPoint getIntermediatePose(OrientedPoint currentPose, OrientedPoint parentPose, OrientedPoint target, double timeStep) {
        if(parentPose == null) {
            return target;
        }
        
        double r1 = MAX_LINEAR_SPEED * timeStep;
        double r2 = LatticeRobot.COMM_RANGE - MAX_LINEAR_SPEED * timeStep;
        
        if(currentPose.distance(target) <= r1) {
            if(parentPose.distance(target) <= r1 + r2) {
                return target;
            }
        }
        //Let the circle of the child be of r0, and the circle of the parent be r1
        double dx = parentPose.x - currentPose.x;
        double dy = parentPose.y - currentPose.y;

        double distance = Math.sqrt(dx * dx + dy * dy);

        //Let a be the distance of the child to the projection point of the intersection
        double a = (r1 * r1 - r2 * r2 + distance * distance) / (2 * distance);

        double h = Math.sqrt(r1*r1 - a * a);

        double x2 = currentPose.x + (dx * a / distance);
        double y2 = currentPose.y + (dy * a / distance);

         // 6. Calculate offsets to the actual intersection points
        double rx = -dy * (h / distance);
        double ry = dx * (h / distance);

        // 7. Final intersection coordinates
        double p1x = x2;
        double p1y = y2;
        double p2x = x2 - rx;
        double p2y = y2 - ry;
        
        double angle1 = Math.atan2(p1y - currentPose.y, p1x - currentPose.x);
        double angle2 = Math.atan2(p2y - currentPose.y, p2x - currentPose.x);

        OrientedPoint option1 = new OrientedPoint(p1x, p1y, angle1);
        OrientedPoint option2 = new OrientedPoint(p2x, p2y, angle2);
        
        double distanceToTarget1 = option1.distance(target);
        double distanceToTarget2 = option2.distance(target);

        return distanceToTarget1 < distanceToTarget2 ? option1 : option2;
    }

    public boolean moveTo(OrientedPoint currentPose, OrientedPoint newPose, double dt) {
        double remainingDt = dt;
        switch(moveState) {

            case ROTATE_TO_POINT:
                remainingDt = rotateToWithRemaining(currentPose, newPose, remainingDt);
                if(moveState == MoveState.TRANSLATE && Double.compare(0.0, remainingDt) < 0) {
                   remainingDt = translateToWithRemaining(currentPose, newPose, remainingDt);
                    if(moveState == MoveState.ROTATE_TO_FINAL && Double.compare(0.0, remainingDt) < 0) {
                        remainingDt = rotateToFinalWithRemaining(currentPose, newPose, remainingDt); 
                    }
                }
                break;

            case TRANSLATE:
                remainingDt = translateToWithRemaining(currentPose, newPose, remainingDt);
                if(moveState == MoveState.ROTATE_TO_FINAL && Double.compare(0.0, remainingDt) < 0) {
                    remainingDt = rotateToFinalWithRemaining(currentPose, newPose, remainingDt); 
                }
                break;

            case ROTATE_TO_FINAL:
                remainingDt = rotateToFinalWithRemaining(currentPose, newPose, remainingDt); 
                break;
            case DONE:
                return true;
        }

        return false;
    }

    public void startMoving() {
        moveState = MoveState.ROTATE_TO_POINT;
    }

    private double rotateToWithRemaining(OrientedPoint currentPose, OrientedPoint newPose, double remainingDt) {
        double targetAngle = Math.atan2(newPose.y - currentPose.y, newPose.x - currentPose.x);
    
        return rotateToAngleWithRemaining(currentPose, targetAngle, remainingDt);
    }

    private double translateToWithRemaining(OrientedPoint pose, OrientedPoint newPose, double remainingDt) {
        double distance = pose.distance(newPose);

        if(isZero(distance)) {
            pose.x = newPose.x;
            pose.y = newPose.y;
            changeState(0, 0);
            moveState = MoveState.ROTATE_TO_FINAL;
            return remainingDt;
        }

        changeState(MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

        double linearVelocity = (WHEEL_RADIUS / 2.0) * (leftAngularVel + rightAngularVel);
        double timeToTranslate = distance / linearVelocity;

        if(Double.compare(timeToTranslate, remainingDt) < 0) {
            pose.x = newPose.x;
            pose.y = newPose.y;
            distTraveled += distance;
            changeState(0, 0);
            moveState = MoveState.ROTATE_TO_FINAL;
            return remainingDt - timeToTranslate;
        }


        double step = linearVelocity * remainingDt;
        pose.x += step * Math.cos(pose.orientation);
        pose.y += step * Math.sin(pose.orientation);
        distTraveled += step;

        return 0.0;
    }
    
    private double rotateToFinalWithRemaining(OrientedPoint currentPose, OrientedPoint target, double remainingDt) {
        double remaining = rotateToAngleWithRemaining(currentPose, target.orientation, remainingDt);
        if(moveState == MoveState.DONE) {
            return remaining;
        }

        return 0.0;
    }

    private double rotateToAngleWithRemaining(OrientedPoint pose, double targetAngle, double remainingDt) {
        //Calculate angle difference
        double rotateBy = normalizeAngle(targetAngle - pose.orientation);
        
        // tolerance
        if(isZero(rotateBy)) {
            pose.orientation = targetAngle;
            changeState(0, 0);
            moveState = (moveState == MoveState.ROTATE_TO_POINT) ? MoveState.TRANSLATE : MoveState.DONE;
            return remainingDt;
        }

        double angularSpeed = (WHEEL_RADIUS / DISTANCE_BETWEEN_WHEELS) * 2 * MAX_ANGULAR_SPEED;
        double timeToRotate = Math.abs(rotateBy) /angularSpeed;

        if(timeToRotate <= remainingDt) {
            pose.orientation = targetAngle;
            changeState(0, 0);
            moveState = (moveState == MoveState.ROTATE_TO_POINT) ? MoveState.TRANSLATE : MoveState.DONE;
            return remainingDt - timeToRotate;
        }

        if(rotateBy > 0.0) {
            changeState(-MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
        } else {
            changeState(MAX_ANGULAR_SPEED, -MAX_ANGULAR_SPEED);
        }

       double angularStep = angularSpeed * remainingDt;
        pose.orientation = normalizeAngle(pose.orientation + angularStep);
        return 0.0;
    }
    
    //EDIT FOR PROPER ANGLE PRESERVATION (NEW ANGLE PRESERVATION EXISTS)
    // Duplicate of MathUtils.normalizeAngle, which is the canonical one. Not deleted
    // here because the two are not identical: this loop uses `angle < -Math.PI` where
    // MathUtils uses `<=`, so this one leaves exactly -pi alone while MathUtils maps it
    // to +pi. Swapping them is a real, if measure-zero, motion-model behaviour change
    // and deserves its own commit.
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

    public double getMaxSpeed() {
        return MAX_LINEAR_SPEED;
    }
}
