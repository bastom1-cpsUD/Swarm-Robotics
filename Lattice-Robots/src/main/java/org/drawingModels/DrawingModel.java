package org.drawingModels;

import java.awt.Shape;

import org.graphs.OrientedPoint;

public abstract class DrawingModel {
    protected Shape currentModel;

    public abstract Shape draw(OrientedPoint position);
    public abstract boolean contains(OrientedPoint position, int x, int y);
    
}
