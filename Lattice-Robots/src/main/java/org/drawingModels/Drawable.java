package org.drawingModels;

import java.awt.Shape;

/**
 * An interface representing a drawable object in the drawing model. Any class that implements this interface must provide a method to draw itself and a method to determine if a point is contained within iself.
 */
public interface Drawable {
    
    /**
     * Draws the object and returns a Shape of its visual representation
     * @return a Shape representing the visual representation of the object
     */
    public Shape draw();

    /**
     * Determines if a given point (x,y) is contained within the object
     * @param x the x coordinate of the point
     * @param y the y coordinate of the point
     * @return true if the point is contained within the ojbect, false otherwise
     */
    public boolean contains(int x, int y);
}
