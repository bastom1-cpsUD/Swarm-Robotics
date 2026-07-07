package com.darcarms.htmllog;

/**
 * Represents an axis-aligned rectangle using lower-left and upper-right
 * coordinate values.
 *
 * <p>{@code Rect} is used throughout HtmlLog for coordinate mapping. In a
 * typical mapped drawing, one rectangle describes the user's logical or
 * mathematical coordinate system, and another rectangle describes the output
 * image's device coordinate system.</p>
 *
 * <p>For example, a user-space rectangle might be:</p>
 *
 * <pre>{@code
 * Rect world = new Rect(-10, 0, 10, 10);
 * }</pre>
 *
 * <p>while a Java2D image-space rectangle might be:</p>
 *
 * <pre>{@code
 * Rect device = new Rect(0, height, width, 0);
 * }</pre>
 *
 * <p>The second example intentionally has {@code minY > maxY}. This is useful
 * when mapping mathematical coordinates, where positive {@code y} points up,
 * into Java2D image coordinates, where positive {@code y} points down.</p>
 *
 * @param minX the x-coordinate of the rectangle's lower-left corner in the
 *             source coordinate convention
 * @param minY the y-coordinate of the rectangle's lower-left corner in the
 *             source coordinate convention
 * @param maxX the x-coordinate of the rectangle's upper-right corner in the
 *             source coordinate convention
 * @param maxY the y-coordinate of the rectangle's upper-right corner in the
 *             source coordinate convention
 */
public record Rect(double minX, double minY, double maxX, double maxY) {

    /**
     * Returns the signed width of this rectangle.
     *
     * <p>The value is computed as {@code maxX - minX}. It is usually positive,
     * but the method intentionally does not force positivity so that rectangles
     * can represent coordinate systems with reversed axes when needed.</p>
     *
     * @return the signed rectangle width
     */
    public double width() {
        return maxX - minX;
    }

    /**
     * Returns the signed height of this rectangle.
     *
     * <p>The value is computed as {@code maxY - minY}. It may be negative for
     * device-space rectangles that intentionally flip the vertical axis.</p>
     *
     * @return the signed rectangle height
     */
    public double height() {
        return maxY - minY;
    }

    /**
     * Returns the horizontal center coordinate of this rectangle.
     *
     * @return the midpoint between {@code minX} and {@code maxX}
     */
    public double centerX() {
        return (minX + maxX) / 2.0;
    }

    /**
     * Returns the vertical center coordinate of this rectangle.
     *
     * @return the midpoint between {@code minY} and {@code maxY}
     */
    public double centerY() {
        return (minY + maxY) / 2.0;
    }

    /**
     * Returns a new rectangle expanded in all directions by {@code amount}.
     *
     * <p>This method is most often used to add visual padding around a
     * user-space drawing before fitting it into an output image.</p>
     *
     * @param amount the amount to subtract from the minimum coordinates and add
     *               to the maximum coordinates
     * @return a padded rectangle
     */
    public Rect padded(double amount) {
        return new Rect(minX - amount, minY - amount, maxX + amount, maxY + amount);
    }

    /**
     * Returns whether this rectangle has zero width or zero height.
     *
     * @return {@code true} if either {@link #width()} or {@link #height()} is
     *         zero; otherwise {@code false}
     */
    public boolean isDegenerate() {
        return width() == 0.0 || height() == 0.0;
    }
}