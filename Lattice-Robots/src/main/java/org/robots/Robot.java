package org.robots;

import java.awt.Shape;
import org.drawingModels.Drawable;
import org.drawingModels.DrawingModel;
import org.graphs.OrientedPoint;
import org.motionModels.MotionModel;
import org.motionModels.Movable;

public class Robot implements Movable, Drawable {
    protected final int robotId;
    protected OrientedPoint pose;
    protected final MotionModel motionModel;
    protected final DrawingModel drawingModel;

    public Robot(int id, OrientedPoint pose, MotionModel motionModel, DrawingModel drawingModel) {
        this.robotId = id;
        this.pose = pose;
        this.motionModel = motionModel;
        this.drawingModel = drawingModel;
    }
     
    public int getRobotId() {
        return robotId;
    }

    public OrientedPoint getPosition() {
        return pose;
    }

    public void setPosition(OrientedPoint newPose) {
        this.pose = newPose;
    }

    /** {@inheritDoc} */
    @Override
    public void move(double dt) {
        motionModel.move(pose, dt);
    }

    /** {@inheritDoc} */
    @Override
    public void startMoving() {
        motionModel.startMoving();
    }
    /** {@inheritDoc} */
    @Override
    public boolean moveTo(OrientedPoint target, double dt) {
        return motionModel.moveTo(pose, target, dt);
    }

    /** {@inheritDoc} */
    @Override
    public double getDistanceTraveled() {
        return motionModel.getDistanceTraveled();
    }
    /** {@inheritDoc} */
    @Override
    public Shape draw() {
        return drawingModel.draw(pose);
    }
    /** {@inheritDoc} */
    @Override 
    public boolean contains(int x, int y) {
        return  drawingModel.contains(pose, x, y);
    }
}
