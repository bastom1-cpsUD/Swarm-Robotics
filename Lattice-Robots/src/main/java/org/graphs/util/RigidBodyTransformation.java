package org.graphs.util;

import Jama.Matrix;

import org.utils.MathUtils;

/**
 * A class representing a rigid body transformation in 2D space, consisting of a rotation and a translation.
 */
public class RigidBodyTransformation {
    /** The 3x3 transformation matrix. */
    Matrix matrix;

    /**
     * Constructs the transformation expressing {@code to} in {@code from}'s frame, i.e.
     * {@code T_from^-1 * T_to}, where both are poses in a common parent frame.
     * Equivalently: the unique T for which
     * {@code new RigidBodyTransformation(from).compose(T)} equals
     * {@code new RigidBodyTransformation(to)}.
     *
     * <p>Do NOT call this with a {@code to} that is already expressed relative to
     * {@code from} -- that subtracts {@code from} a second time. For an
     * already-relative pose use the single-argument constructor instead.
     *
     * @param from The starting oriented point.
     * @param to The target oriented point.
     */
    public RigidBodyTransformation(OrientedPoint from, OrientedPoint to) {
        double deltaTheta = to.getOrientation() - from.getOrientation();
        double cosTheta = Math.cos(deltaTheta);
        double sinTheta = Math.sin(deltaTheta);

        // The translation is (to - from) expressed in `from`'s frame: R(-theta_from) * d.
        // Leaving it unrotated, as this used to, is only correct when from's own
        // orientation is zero.
        double dx = to.x - from.x;
        double dy = to.y - from.y;
        double cosFrom = Math.cos(from.getOrientation());
        double sinFrom = Math.sin(from.getOrientation());
        double tx =  cosFrom * dx + sinFrom * dy;
        double ty = -sinFrom * dx + cosFrom * dy;

        this.matrix = new Matrix(new double[][] {
            {cosTheta, -sinTheta, tx},
            {sinTheta, cosTheta, ty},
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
     * The rotation this transformation induces, in radians: the angle of the source
     * frame's x-axis expressed in the target frame. Exact, since the upper-left block
     * is a proper rotation. The result lies in [-pi, pi] (atan2's codomain).
     * @return this transformation's own rotation angle
     */
    public double getRotation() {
        return Math.atan2(this.matrix.get(1, 0), this.matrix.get(0, 0));
    }

    /**
     * This transformation viewed as a pose: the image of the origin, carrying this
     * transformation's own rotation. Exactly equivalent to, and the intended
     * replacement for, {@code apply(new OrientedPoint(0, 0, 0))}.
     * @return the pose this transformation maps the identity pose to
     */
    public OrientedPoint asPose() {
        return new OrientedPoint(matrix.get(0, 2), matrix.get(1, 2), getRotation());
    }

    /**
     * Applies this transformation to a pose. The position is transformed by matrix
     * multiplication; the orientation is composed additively, because a homogeneous
     * multiply cannot carry the point's own theta through -- the third slot of the
     * column vector holds the homogeneous 1, not an angle. The rotation between the
     * two frames is still stored in the matrix (see {@link #getRotation()}); it just
     * has to be applied separately.
     *
     * @param point the pose to transform
     * @return the pose expressed in this transformation's target frame, with its
     *         orientation normalized to (-pi, pi]
     */
    public OrientedPoint apply(OrientedPoint point) {
        Matrix pointMatrix = new Matrix(new double[][] {{point.x}, {point.y}, {1}});
        Matrix result = this.matrix.times(pointMatrix);
        double x = result.get(0, 0);
        double y = result.get(1, 0);
        // Normalizing is canonical-form hygiene, not a wrap fix: the sum of two
        // (-pi, pi] values lands in (-2pi, 2pi], and OrientedPoint.equals compares
        // orientation with an exact Double.compare.
        double orientation = MathUtils.normalizeAngle(point.getOrientation() + getRotation());
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

    /**
     * Determines whether this transformation is approximately the identity, testing
     * translation and rotation against separate tolerances.
     *
     * <p>Prefer this over {@link #isApproximatelyIdentity(double)} for any geometric
     * decision. That overload compares raw matrix entries against one scalar, which
     * silently conflates two different units: the off-diagonal entries are
     * dimensionless sines and cosines, while the third column is a translation in
     * lattice units (tens). A tolerance loose enough to admit real accumulated
     * position drift over a face therefore also admits a rotation that is wildly
     * wrong, and a tolerance tight enough to pin the rotation rejects every real
     * closure. The two must be sized independently.
     *
     * @param positionEpsilon maximum translation magnitude, in lattice units
     * @param angleEpsilon maximum absolute rotation, in radians
     * @return true if this transformation is within both tolerances of the identity
     */
    public boolean isApproximatelyIdentity(double positionEpsilon, double angleEpsilon) {
        // The third column is the image of the origin -- the same quantity asPose()
        // reports -- read directly here to avoid allocating a pose per test. Its
        // magnitude is the true distance between the two frames' origins: the stored
        // translation is the offset rotated into the source frame, and rotation
        // preserves length.
        double dx = matrix.get(0, 2);
        double dy = matrix.get(1, 2);
        if (Math.hypot(dx, dy) > positionEpsilon) {
            return false;
        }

        // getRotation() is an atan2, so it is already wrapped into [-pi, pi] and needs
        // no further normalization before taking its magnitude.
        return Math.abs(getRotation()) <= angleEpsilon;
    }
}
