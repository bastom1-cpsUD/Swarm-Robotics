package org.Transformations;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.RenderingHints;
import java.awt.geom.Path2D;

/**
 * A simple Robot class representing a robot with position and orientation.
 */
public class Robot {
    
    /**
     * The position of the robot represented as an OrientedPoint (x, y, orientation).
     */
    private OrientedPoint position;
    
    /**
     * Constructor to initialize the robot's position and orientation.
     * @param x The x-coordinate of the robot's position.
     * @param y The y-coordinate of the robot's position.
     * @param orientation The orientation of the robot in radians.
     */
    public Robot(double x, double y, double orientation) {
        this.position = new OrientedPoint(x, y, orientation);
    }

    public OrientedPoint getPosition() {
        return position;
    }

    public void setPosition(OrientedPoint position) {
        this.position = position;
    }

    public void draw(Graphics2D g2d) {
        // Drawing logic for the robot
        double s = 80.0;
        double R = s / Math.sqrt(3.0);
        double theta0 = position.getOrientation();
        long[] xcoords = new long[4];
        long[] ycoords = new long[4]; 
        //Calculate first triangle vertices
        double radius = R * 1.20;
        double angle = theta0; // 0 angle for the first vertex 
        xcoords[0] =  Math.round(position.x + radius * Math.cos(angle));
        ycoords[0] =  Math.round(position.y + radius * Math.sin(angle)); 
        //Calculate second triangle vertices
        angle = theta0 + 2 * Math.PI / 3.0; // 120 degrees or 2π/3 radians
        xcoords[1] =  Math.round(position.x + R * Math.cos(angle));
        ycoords[1] =  Math.round(position.y + R * Math.sin(angle)); 
        //Calculate third vertice (for flag like tail)
        radius = R * 0.1;
        angle = theta0 + Math.PI; /// 180 degrees or π radians 
        xcoords[2] =  Math.round(position.x + radius * Math.cos(angle));
        ycoords[2] =  Math.round(position.y + radius * Math.sin(angle)); 
        //Calculate fourth triangle vertices
        angle = theta0 + 4 * Math.PI / 3.0; // 240 degrees or 4π/3 radians
        xcoords[3] =  Math.round(position.x + R * Math.cos(angle));
        ycoords[3] =  Math.round(position.y + R * Math.sin(angle));

        //Enable anti-aliasing for smoother rendering
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        //Draw the robot as a filled polygon
        Path2D.Double shape = new Path2D.Double();
        shape.moveTo(xcoords[0], ycoords[0]);
        shape.lineTo(xcoords[1], ycoords[1]);
        shape.lineTo(xcoords[2], ycoords[2]);
        shape.lineTo(xcoords[3], ycoords[3]);
        shape.closePath();

        g2d.setColor(Color.BLACK);
        g2d.fill(shape);
    }
}

