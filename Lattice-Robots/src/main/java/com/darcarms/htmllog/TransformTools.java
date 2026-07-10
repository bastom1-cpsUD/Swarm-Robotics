package com.darcarms.htmllog;

import java.awt.Graphics2D;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.Path2D;
import java.awt.geom.Point2D;
import java.util.Objects;

/**
 * Geometry and coordinate-transform helpers for Java2D-backed HtmlLog output.
 *
 * <p>{@code TransformTools} provides utilities for mapping logical drawing
 * coordinates into image coordinates. The most important methods are
 * {@link #stretchRectangle(Rect, Rect)} and
 * {@link #fitRectangle(Rect, Rect, double)}, which build
 * {@link AffineTransform} objects for Java2D rendering.</p>
 *
 * <p>A common pattern is to map mathematical coordinates, where positive
 * {@code y} points upward, into Java2D image coordinates, where positive
 * {@code y} points downward:</p>
 *
 * <pre>{@code
 * Rect user = new Rect(-10, 0, 10, 10);
 * Rect device = new Rect(0, height, width, 0);
 *
 * AffineTransform tx = TransformTools.fitRectangle(user, device, 0.1);
 * graphics.setTransform(tx);
 * }</pre>
 *
 * <p>This class contains only static utility methods and cannot be
 * instantiated.</p>
 */
public final class TransformTools {

    private TransformTools() {
    }

    /**
     * Returns a transform that maps one rectangle into another, stretching if
     * necessary.
     *
     * <p>The returned transform maps {@code userAabb.minX()} to
     * {@code deviceAabb.minX()}, {@code userAabb.maxX()} to
     * {@code deviceAabb.maxX()}, {@code userAabb.minY()} to
     * {@code deviceAabb.minY()}, and {@code userAabb.maxY()} to
     * {@code deviceAabb.maxY()}.</p>
     *
     * <p>This method does not preserve aspect ratio. Use
     * {@link #fitRectangle(Rect, Rect, double)} when the source rectangle should
     * be scaled uniformly and centered.</p>
     *
     * @param userAabb the source/user-space rectangle
     * @param deviceAabb the destination/device-space rectangle
     * @return an affine transform mapping {@code userAabb} into
     *         {@code deviceAabb}
     * @throws NullPointerException if either rectangle is null
     * @throws IllegalArgumentException if either rectangle has zero width or
     *                                  zero height
     */
    public static AffineTransform stretchRectangle(Rect userAabb, Rect deviceAabb) {
        Objects.requireNonNull(userAabb, "userAabb");
        Objects.requireNonNull(deviceAabb, "deviceAabb");
        requireNonDegenerate(userAabb, "userAabb");
        requireNonDegenerate(deviceAabb, "deviceAabb");

        double sx = deviceAabb.width() / userAabb.width();
        double sy = deviceAabb.height() / userAabb.height();
        double tx = deviceAabb.minX() - sx * userAabb.minX();
        double ty = deviceAabb.minY() - sy * userAabb.minY();

        return new AffineTransform(sx, 0.0, 0.0, sy, tx, ty);
    }

    /**
     * Returns a transform that fits one rectangle inside another while
     * preserving aspect ratio.
     *
     * <p>The source rectangle is first expanded by a padding amount derived
     * from its larger dimension. The padded rectangle is then uniformly scaled
     * as large as possible into {@code deviceAabb}. Any remaining slack is
     * centered.</p>
     *
     * <p>Use a destination rectangle such as {@code new Rect(0, height, width,
     * 0)} to flip a mathematical {@code y}-up coordinate system into Java2D's
     * default {@code y}-down image coordinate system.</p>
     *
     * @param userAabb the source/user-space rectangle to fit
     * @param deviceAabb the destination/device-space rectangle
     * @param padding fractional padding relative to the larger user-space
     *                dimension; for example, {@code 0.1} adds ten percent
     *                padding
     * @return an affine transform that maps the padded, aspect-preserved source
     *         rectangle into the destination rectangle
     * @throws NullPointerException if either rectangle is null
     * @throws IllegalArgumentException if {@code padding} is negative or either
     *                                  rectangle has zero width or zero height
     */
    public static AffineTransform fitRectangle(Rect userAabb, Rect deviceAabb, double padding) {
        Objects.requireNonNull(userAabb, "userAabb");
        Objects.requireNonNull(deviceAabb, "deviceAabb");
        requireNonDegenerate(userAabb, "userAabb");
        requireNonDegenerate(deviceAabb, "deviceAabb");

        if (padding < 0.0) {
            throw new IllegalArgumentException("padding must be non-negative");
        }

        double userWidth = Math.abs(userAabb.width());
        double userHeight = Math.abs(userAabb.height());
        double pad = Math.max(userWidth, userHeight) * padding;
        Rect padded = userAabb.padded(pad);

        double paddedWidth = Math.abs(padded.width());
        double paddedHeight = Math.abs(padded.height());
        double deviceWidth = Math.abs(deviceAabb.width());
        double deviceHeight = Math.abs(deviceAabb.height());

        double scale = Math.min(deviceWidth / paddedWidth, deviceHeight / paddedHeight);

        double fittedUserWidth = deviceWidth / scale;
        double fittedUserHeight = deviceHeight / scale;

        double centerX = padded.centerX();
        double centerY = padded.centerY();

        Rect fittedUserAabb = new Rect(
                centerX - fittedUserWidth / 2.0,
                centerY - fittedUserHeight / 2.0,
                centerX + fittedUserWidth / 2.0,
                centerY + fittedUserHeight / 2.0
        );

        return stretchRectangle(fittedUserAabb, deviceAabb);
    }

    /**
     * Creates a triangular arrowhead shape at the end of a directed segment.
     *
     * <p>The arrowhead is placed at {@code (endX, endY)} and points in the
     * direction from {@code (startX, startY)} toward {@code (endX, endY)}. The
     * returned shape is a closed triangular {@link Path2D} in the current user
     * coordinate system.</p>
     *
     * @param startX x-coordinate of the segment start
     * @param startY y-coordinate of the segment start
     * @param endX x-coordinate of the arrow tip
     * @param endY y-coordinate of the arrow tip
     * @param length length of each arrowhead side
     * @param spreadRadians angle between the two arrowhead sides, in radians
     * @return a closed triangular arrowhead shape
     * @throws IllegalArgumentException if {@code length} or
     *                                  {@code spreadRadians} is negative
     */
    public static Shape arrowhead(
            double startX,
            double startY,
            double endX,
            double endY,
            double length,
            double spreadRadians
    ) {
        if (length < 0.0) {
            throw new IllegalArgumentException("length must be non-negative");
        }
        if (spreadRadians < 0.0) {
            throw new IllegalArgumentException("spreadRadians must be non-negative");
        }

        double vx = startX - endX;
        double vy = startY - endY;

        double angleCenter = Math.atan2(vy, vx);
        double angleLeft = angleCenter + spreadRadians / 2.0;
        double angleRight = angleCenter - spreadRadians / 2.0;

        double leftX = endX + length * Math.cos(angleLeft);
        double leftY = endY + length * Math.sin(angleLeft);
        double rightX = endX + length * Math.cos(angleRight);
        double rightY = endY + length * Math.sin(angleRight);

        Path2D path = new Path2D.Double();
        path.moveTo(endX, endY);
        path.lineTo(leftX, leftY);
        path.lineTo(rightX, rightY);
        path.closePath();

        return path;
    }

    /**
     * Draws an arrowhead using the current {@link Graphics2D} paint, stroke,
     * transform, and rendering hints.
     *
     * <p>If {@code filled} is true, the arrowhead is filled first and then
     * stroked. If {@code filled} is false, only the outline is drawn.</p>
     *
     * @param g the graphics context to draw into
     * @param startX x-coordinate of the segment start
     * @param startY y-coordinate of the segment start
     * @param endX x-coordinate of the arrow tip
     * @param endY y-coordinate of the arrow tip
     * @param length length of each arrowhead side
     * @param spreadRadians angle between the two arrowhead sides, in radians
     * @param filled whether to fill the arrowhead before stroking it
     * @throws NullPointerException if {@code g} is null
     * @throws IllegalArgumentException if {@code length} or
     *                                  {@code spreadRadians} is negative
     */
    public static void drawArrowhead(
            Graphics2D g,
            double startX,
            double startY,
            double endX,
            double endY,
            double length,
            double spreadRadians,
            boolean filled
    ) {
        Objects.requireNonNull(g, "g");

        Shape shape = arrowhead(startX, startY, endX, endY, length, spreadRadians);

        if (filled) {
            g.fill(shape);
        }

        g.draw(shape);
    }

    /**
     * Draws a string that remains upright even when the current graphics
     * transform flips or scales the coordinate system.
     *
     * <p>The text position is supplied in the current user coordinate system.
     * This method transforms that point into device coordinates, temporarily
     * resets the graphics transform to identity, draws the string, and then
     * restores the original transform.</p>
     *
     * <p>This is useful after applying a transform such as
     * {@link #fitRectangle(Rect, Rect, double)} with a vertically flipped device
     * rectangle. Shapes can be drawn in logical coordinates, while labels still
     * appear upright in the generated image.</p>
     *
     * @param g the graphics context to draw into
     * @param text the string to draw
     * @param userX x-coordinate of the text anchor in user coordinates
     * @param userY y-coordinate of the text anchor in user coordinates
     * @throws NullPointerException if {@code g} or {@code text} is null
     */
    public static void drawUprightString(Graphics2D g, String text, double userX, double userY) {
        Objects.requireNonNull(g, "g");
        Objects.requireNonNull(text, "text");

        AffineTransform original = g.getTransform();
        Point2D device = original.transform(new Point2D.Double(userX, userY), null);

        g.setTransform(new AffineTransform());
        g.drawString(text, (float) device.getX(), (float) device.getY());
        g.setTransform(original);
    }

    private static void requireNonDegenerate(Rect rect, String name) {
        if (rect.isDegenerate()) {
            throw new IllegalArgumentException(name + " must have nonzero width and height");
        }
    }
}