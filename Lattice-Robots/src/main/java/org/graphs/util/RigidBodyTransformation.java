package org.graphs.util;

import Jama.Matrix;

/**
 * A class representing a rigid body transformation in 2D space, consisting of a rotation and a translation.
 */
public class RigidBodyTransformation {
    /** The 3x3 transformation matrix. */
    Matrix matrix;

    /**
     * Constructs a transformation that maps the 'from' oriented point to the 'to' oriented point.
     * @param from The starting oriented point.
     * @param to The target oriented point.
     */
    public RigidBodyTransformation(OrientedPoint from, OrientedPoint to) {
        OrientedPoint delta = new OrientedPoint(to.x - from.x, to.y - from.y, to.getOrientation() - from.getOrientation());
        double cosTheta = Math.cos(delta.getOrientation());
        double sinTheta = Math.sin(delta.getOrientation());
        this.matrix = new Matrix(new double[][] {
            {cosTheta, -sinTheta, delta.x},
            {sinTheta, cosTheta, delta.y},
            {0, 0, 1}
        });
    }

    /**
     * Constructs a transformation that maps the origin to the given point
     * @param to a given point in 2D space with an orientation
     */
    public RigidBodyTransformation(OrientedPoint to) {
        double cosTheta = Math.cos(to.getOrientation());
        double sinTheta = Math.sin(to.getOrientation());
        this.matrix = new Matrix(new double[][] {
            {cosTheta, -sinTheta, to.x},
            {sinTheta, cosTheta, to.y},
            {0, 0, 1}
        });
    }
    
    public RigidBodyTransformation() {
        this.matrix = Matrix.identity(3, 3);
    }

    private RigidBodyTransformation(Matrix matrix) {
        this.matrix = matrix;
    }

    /**
     * Returns the identity transformation (no rotation, no translation).
     * @return The identity transformation.
     */
    public static RigidBodyTransformation identity() {
        return new RigidBodyTransformation(new OrientedPoint(0, 0, 0));
    }

    /**
     * Returns the inverse of this transformation.
     * @return The inverse transformation.
     */
    public RigidBodyTransformation inverse() {
        Matrix inverseMatrix = this.matrix.inverse();
        return new RigidBodyTransformation(inverseMatrix);
    }

    /**
     * Composes this transformation with another, applied after this one.
     * @param other the transformation expressed relative to this one's target frame
     * @return the combined transformation from this one's source frame to other's target frame
     */
    public RigidBodyTransformation compose(RigidBodyTransformation other) {
        return new RigidBodyTransformation(this.matrix.times(other.matrix));
    }

    /**
     * Applies a transformation via matrix multiplication
     * @param point the point that undergoes matrix multiplication
     * @return the transformed point
     */
    public OrientedPoint apply(OrientedPoint point) {
        Matrix pointMatrix = new Matrix(new double[][] {{point.x}, {point.y}, {1}});
        Matrix result = this.matrix.times(pointMatrix);
        double x = result.get(0, 0);
        double y = result.get(1, 0);
        double orientation = Math.atan2(this.matrix.get(1, 0), this.matrix.get(0, 0));
        return new OrientedPoint(x, y, orientation);
    }

    /**
     * Determines whether the a provided Rigid Body Transformation is an inverse
     * @param other a second Rigid Body Transformation
     * @return whether the provided transformation is an inverse
     */
    public boolean isInverse(RigidBodyTransformation other) {
        Matrix result = this.matrix.times(other.matrix);

        int rows = result.getRowDimension();
        int cols = result.getColumnDimension();

        if (rows != cols) {
            return false;
        }

        double epsilon = 1e-9;

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {

                double expected = (i == j) ? 1.0 : 0.0;

                if (Math.abs(result.get(i, j) - expected) > epsilon) {
                    return false;
                }
            }
        }

        return true;
    }

    /**
     * Determines whether this transformation is approximately the identity
     * transformation within a specified tolerance.
     *
     * @param epsilon the maximum allowable error for each matrix entry
     * @return true if this transformation is approximately the identity matrix
     */
    public boolean isApproximatelyIdentity(double epsilon) {
        int rows = matrix.getRowDimension();
        int cols = matrix.getColumnDimension();

        if (rows != cols) {
            return false;
        }

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                double expected = (i == j) ? 1.0 : 0.0;

                if (Math.abs(matrix.get(i, j) - expected) > epsilon) {
                    return false;
                }
            }
        }

        return true;
    }
}
