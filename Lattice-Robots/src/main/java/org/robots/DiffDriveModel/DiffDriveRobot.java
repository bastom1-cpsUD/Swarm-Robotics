package org.robots.DiffDriveModel;

import java.awt.Polygon;
import java.awt.geom.Point2D;

public class DiffDriveRobot {
    //Current position & angle
    private double x;
    private double y;
    private double theta;

    //Set dimensions of robot model
    private final double WHEEL_RADIUS = 10;
    private final double DISTANCE_BETWEEN_WHEELS = 1;

    //State Modifiers
    private double leftAngularVel;
    private double rightAngularVel;

    //Fixed Position
    private double ROBOT_SIZE = 40;


    //Distance traveled
    private double distTraveled;
    public DiffDriveRobot() {
        x = 500;
        y = 500;
        theta = 0;
    }

    public void changeState(double leftAngularVel, double rightAngularVel) {
        this.leftAngularVel = leftAngularVel;
        this.rightAngularVel = rightAngularVel;
    }

    public double[] IVK(double xVel, double angularVel) {
        double leftAngVel = (xVel - (DISTANCE_BETWEEN_WHEELS / 2) * angularVel) / WHEEL_RADIUS;
        double rightAngVel = (xVel + (DISTANCE_BETWEEN_WHEELS / 2) * angularVel) / WHEEL_RADIUS;

        return new double[] {leftAngVel, rightAngVel};
    }

    public void move(double dt) {
        this.x = x + (WHEEL_RADIUS / 2) * (leftAngularVel + rightAngularVel) * Math.cos(theta) * dt;
        this.y = y + (WHEEL_RADIUS / 2) * (leftAngularVel + rightAngularVel) * Math.sin(theta) * dt;
        if(leftAngularVel != rightAngularVel) {
            this.theta = theta + (WHEEL_RADIUS / DISTANCE_BETWEEN_WHEELS) * (rightAngularVel - leftAngularVel) * dt;    
        }

        distTraveled = distTraveled + ( WHEEL_RADIUS * (leftAngularVel + rightAngularVel) / 2) * dt;
    }

    public void move(double xVel, double angularVel, double dt) {
        double[] velKinetics = IVK(xVel, angularVel);
        changeState(velKinetics[0], velKinetics[1]);

        move(dt);
    }

    public Polygon draw() {

        Polygon p = new Polygon();
        double s = ROBOT_SIZE;
        double R = s / Math.sqrt(3.0);
        double theta0 = theta;
        int[] xcoords = new int[4];
        int[] ycoords = new int[4]; 
        //Calculate first triangle vertices
        double radius = R * 1.20;
        double angle = theta0; // 0 angle for the first vertex 
        xcoords[0] =  (int) Math.round(x + radius * Math.cos(angle));
        ycoords[0] =  (int) Math.round(y + radius * Math.sin(angle)); 
        //Calculate second triangle vertices
        angle = theta0 + 2 * Math.PI / 3.0; // 120 degrees or 2π/3 radians
        xcoords[1] =  (int) Math.round(x + R * Math.cos(angle));
        ycoords[1] =  (int) Math.round(y + R * Math.sin(angle)); 
        //Calculate third vertice (for flag like tail)
        radius = R * 0.1;
        angle = theta0 + Math.PI; /// 180 degrees or π radians 
        xcoords[2] =  (int) Math.round(x + radius * Math.cos(angle));
        ycoords[2] =  (int) Math.round(y + radius * Math.sin(angle)); 
        //Calculate fourth triangle vertices
        angle = theta0 + 4 * Math.PI / 3.0; // 240 degrees or 4π/3 radians
        xcoords[3] =  (int) Math.round(x + R * Math.cos(angle));
        ycoords[3] =  (int) Math.round(y + R * Math.sin(angle));

        for(int i = 0; i < xcoords.length; i++) {
            p.addPoint(xcoords[i], ycoords[i]);
        }

        return p;
    }

    public Point2D.Double getPosition() {
        return new Point2D.Double(x, y);
    }

    @Override   
    public String toString() {
        return "DiffDriveRobot[Position: (" + x + "," + y + ")" + " LeftAng: " + leftAngularVel + " RightAng: " + rightAngularVel + " Distance Traveled: " + distTraveled + " meters]";  
    }
}
