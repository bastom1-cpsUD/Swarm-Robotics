package org.drawingModels;

import java.awt.Shape;

import org.graphs.util.OrientedPoint;

import java.awt.Polygon;

public class TriangularModel extends DrawingModel{
    
    /**
     * The size of the sides of the triangle representing the robot.
     */
    private static final double ROBOT_SIZE = 30;

    public TriangularModel() {
        currentModel = new Polygon();
    }
    /** {@inheritDoc} */
    @Override
    public Shape draw(OrientedPoint position) {
        Polygon p = (Polygon) currentModel;
        // Update polygon points based on current position and orientation
        double s = ROBOT_SIZE;
        double R = s / Math.sqrt(3.0);
        double theta0 = position.getOrientation();
        int[] xcoords = new int[4];
        int[] ycoords = new int[4]; 
        //Calculate first triangle vertices
        double radius = R * 1.20;
        double angle = theta0; // 0 angle for the first vertex 
        xcoords[0] =  (int) Math.round(position.x + radius * Math.cos(angle));
        ycoords[0] =  (int) Math.round(position.y + radius * Math.sin(angle)); 
        //Calculate second triangle vertices
        angle = theta0 + 2 * Math.PI / 3.0; // 120 degrees or 2π/3 radians
        xcoords[1] =  (int) Math.round(position.x + R * Math.cos(angle));
        ycoords[1] =  (int) Math.round(position.y + R * Math.sin(angle)); 
        //Calculate third vertice (for flag like tail)
        radius = R * 0.1;
        angle = theta0 + Math.PI; /// 180 degrees or π radians 
        xcoords[2] =  (int) Math.round(position.x + radius * Math.cos(angle));
        ycoords[2] =  (int) Math.round(position.y + radius * Math.sin(angle)); 
        //Calculate fourth triangle vertices
        angle = theta0 + 4 * Math.PI / 3.0; // 240 degrees or 4π/3 radians
        xcoords[3] =  (int) Math.round(position.x + R * Math.cos(angle));
        ycoords[3] =  (int) Math.round(position.y + R * Math.sin(angle));;

        p.reset();

        for(int i = 0; i < xcoords.length; i++) {
            p.addPoint(xcoords[i], ycoords[i]);
        }

        return p;
    }

    /** {@inheritDoc} */
    @Override
    public boolean contains(OrientedPoint position, int x, int y) {
        draw(position);
        return currentModel.contains(x, y);
    }
}
