package org.transformations;
/**
 * An extension of Point2D.Double that includes an orientation (angle in radians).
 */
public class OrientedPoint extends java.awt.geom.Point2D.Double {
    
    private double orientation; // in radians

    /**
     * Constructor to initialize the oriented point.
     * @param x The x-coordinate of the point.
     * @param y The y-coordinate of the point.
     * @param orientation The orientation in radians.
     */
    public OrientedPoint(double x, double y, double orientation) {
        super(x, y);
        this.orientation = orientation;
    }

    public double getOrientation() {
        return orientation;
    }

    public double getOrientationInDegrees() {
        return Math.toDegrees(orientation);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof OrientedPoint)) return false;
        OrientedPoint other = (OrientedPoint) obj;
        return java.lang.Double.compare(this.x, other.x) == 0
                && java.lang.Double.compare(this.y, other.y) == 0
                && java.lang.Double.compare(this.orientation, other.orientation) == 0;
    }

    @Override
    public int hashCode() {
        return java.util.Objects.hash(x, y, orientation);
    }

    @Override
    public String toString() {
        return "OrientedPoint[x: " + x + ", y: " + y + ", orientation (radians): " + orientation + ", orientation (degrees): " + getOrientationInDegrees() + "]";
    }
}
