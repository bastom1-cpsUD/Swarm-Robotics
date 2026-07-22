package org.drawingModels;

import java.awt.Shape;

import org.graphs.util.OrientedPoint;

/**
 * An abstract class representing a drawing model for robots. This class defines the structure for drawing models, which include a method to draw the model,
 * check if a point is contained within the model, and a protected field for the current model's shape.
 */
public abstract class DrawingModel {
    protected Shape currentModel;

    /**
     * Draws the model based on the given position and orientation of the robot.
     * @param position the current position and orientation of the robot
     * @return a Shape representing the model based on the robot's position and orientation
     */
    public abstract Shape draw(OrientedPoint position);

    /**
     * Determines if a given point (x,y) is contained within the model based on the robot's current position and orientation.
     * @param position the current position and orientation of the robot
     * @param x the x coordinate of the point
     * @param y the y coordinate of the point
     * @return true if the point is contained within the model, false otherwise
     */
    public abstract boolean contains(OrientedPoint position, int x, int y);
    
}
