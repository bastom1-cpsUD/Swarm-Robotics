package org.drawingModels;

import java.awt.Shape;


public interface Drawable {
    
    public Shape draw();
    public boolean contains(int x, int y);
}
