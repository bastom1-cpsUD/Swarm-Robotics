package org.DiffDriveModel;

import java.awt.Polygon;

import org.graphs.OrientedPoint;

public class DiffDriveRobot {
    private enum MoveState {
        ROTATE_TO_POINT,
        TRANSLATE,
        ROTATE_TO_FINAL,
        DONE
    }

    //Current position & angle
    private OrientedPoint pose;

    //Set dimensions of robot model
    private static final double WHEEL_RADIUS = 4;
    private static final double DISTANCE_BETWEEN_WHEELS = 30;

    //State Modifiers
    private double leftAngularVel;
    private double rightAngularVel;

    //Fixed Position
    private double ROBOT_SIZE = 40;

    //Max angular velocity of wheels
    private static final double MAX_LINEAR_SPEED = 5;
    private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / WHEEL_RADIUS;
    //Distance traveled
    private double distTraveled;

    private MoveState moveState = MoveState.DONE;

    public DiffDriveRobot() {
        pose = new OrientedPoint(500, 500, 0);
    }

    public void changeState(double leftAngVel, double rightAngVel) {
        this.leftAngularVel = leftAngVel;
        this.rightAngularVel = rightAngVel;
    }

    public double[] IVK(double xVel, double angularVel) {
        double leftAngVel = (xVel - (DISTANCE_BETWEEN_WHEELS / 2) * angularVel) / WHEEL_RADIUS;
        double rightAngVel = (xVel + (DISTANCE_BETWEEN_WHEELS / 2) * angularVel) / WHEEL_RADIUS;

        return new double[] {leftAngVel, rightAngVel};
    }

    public void move(double dt) {
        pose.x = pose.x + (WHEEL_RADIUS / 2) * (leftAngularVel + rightAngularVel) * Math.cos(pose.orientation) * dt;
        pose.y = pose.y + (WHEEL_RADIUS / 2) * (leftAngularVel + rightAngularVel) * Math.sin(pose.orientation) * dt;
        if(leftAngularVel != rightAngularVel) {
            pose.orientation = normalizeAngle(pose.orientation + (WHEEL_RADIUS / DISTANCE_BETWEEN_WHEELS) * (rightAngularVel - leftAngularVel) * dt);    
        }

        distTraveled = distTraveled + (WHEEL_RADIUS * (leftAngularVel + rightAngularVel) / 2) * dt;
    }

    public void move(double xVel, double angularVel, double dt) {
        double[] velKinetics = IVK(xVel, angularVel);
        changeState(velKinetics[0], velKinetics[1]);

        move(dt);
    }

    public void startMoving() {
        moveState = MoveState.ROTATE_TO_POINT;
    }

    public boolean moveTo(OrientedPoint newPose, double dt) {
        switch(moveState) {

            case ROTATE_TO_POINT:
                if(rotateTo(newPose, dt)) {
                    moveState = MoveState.TRANSLATE;
                }
                break;

            case TRANSLATE:
                if(translateTo(newPose, dt)) {
                    moveState = MoveState.ROTATE_TO_FINAL;
                }
                break;

            case ROTATE_TO_FINAL:
                if(rotateTo(newPose.orientation, dt)) {
                    moveState = MoveState.DONE;
                    return true;
                }
                break;

            case DONE:
                return true;
        }

        return false;
    }

    private boolean rotateTo(OrientedPoint newPose, double dt) {
        double targetAngle = Math.atan2(newPose.y - pose.y, newPose.x - pose.x);

        return rotateTo(targetAngle, dt);
    }

    private boolean translateTo(OrientedPoint newPose, double dt) {
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
    
    private boolean rotateTo(double targetAngle, double dt) {
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

    private boolean isZero(double value) {
        return Math.abs(value) < 1e-9;
    }

    public Polygon draw() {

        Polygon p = new Polygon();
        double s = ROBOT_SIZE;
        double R = s / Math.sqrt(3.0);
        double theta0 = pose.orientation;
        int[] xcoords = new int[4];
        int[] ycoords = new int[4]; 
        //Calculate first triangle vertices
        double radius = R * 1.20;
        double angle = theta0; // 0 angle for the first vertex 
        xcoords[0] =  (int) Math.round(pose.x + radius * Math.cos(angle));
        ycoords[0] =  (int) Math.round(pose.y + radius * Math.sin(angle)); 
        //Calculate second triangle vertices
        angle = theta0 + 2 * Math.PI / 3.0; // 120 degrees or 2π/3 radians
        xcoords[1] =  (int) Math.round(pose.x + R * Math.cos(angle));
        ycoords[1] =  (int) Math.round(pose.y + R * Math.sin(angle)); 
        //Calculate third vertice (for flag like tail)
        radius = R * 0.1;
        angle = theta0 + Math.PI; /// 180 degrees or π radians 
        xcoords[2] =  (int) Math.round(pose.x + radius * Math.cos(angle));
        ycoords[2] =  (int) Math.round(pose.y + radius * Math.sin(angle)); 
        //Calculate fourth triangle vertices
        angle = theta0 + 4 * Math.PI / 3.0; // 240 degrees or 4π/3 radians
        xcoords[3] =  (int) Math.round(pose.x + R * Math.cos(angle));
        ycoords[3] =  (int) Math.round(pose.y + R * Math.sin(angle));

        for(int i = 0; i < xcoords.length; i++) {
            p.addPoint(xcoords[i], ycoords[i]);
        }

        return p;
    }

    public OrientedPoint getPosition() {
        return pose;
    }

    @Override   
    public String toString() {
        return "DiffDriveRobot[Position: " + pose + " LeftAng: " + leftAngularVel + " RightAng: " + rightAngularVel + " Distance Traveled: " + distTraveled + " meters]";  
    }
}
