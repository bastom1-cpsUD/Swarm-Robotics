package com.darcarms.htmllog;

import javax.imageio.ImageIO;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.util.Objects;

/**
 * Auto-closeable Java2D drawing surface that writes a PNG asset when closed.
 *
 * <p>A {@code LogGraphics} instance owns a {@link BufferedImage}, its associated
 * {@link Graphics2D} context, and the output path where the image will be
 * written. Users draw into the graphics context returned by {@link #graphics()}.
 * When the instance is closed, the graphics context is disposed, the image is
 * written as a PNG file, and the image is embedded in the associated
 * {@link HtmlLog}.</p>
 *
 * <p>Instances are normally created through {@link HtmlLog#graphics(String, int, int)}
 * or {@link HtmlLog#mappedGraphics(String, Rect, int, int)} rather than
 * constructed directly.</p>
 *
 * <pre>{@code
 * try (LogGraphics canvas = log.graphics("drawing-", 600, 400)) {
 *     Graphics2D g = canvas.graphics();
 *     g.drawLine(50, 50, 550, 350);
 * }
 * }</pre>
 *
 * <p>Because this class implements {@link AutoCloseable}, it is intended to be
 * used with try-with-resources. Closing is idempotent: closing the same instance
 * more than once has no additional effect.</p>
 */
public final class LogGraphics implements AutoCloseable {
    private final HtmlLog log;
    private final Path path;
    private final String filename;
    private final BufferedImage image;
    private final Graphics2D graphics;
    private final String style;
    private boolean closed;

    /**
     * Creates a Java2D drawing surface for a log asset.
     *
     * <p>This constructor is package-private because callers should normally
     * create drawing surfaces through {@link HtmlLog}. If {@code userAabb} is
     * non-null, the graphics context is initialized with a transform that maps
     * the supplied user-space rectangle into the output image.</p>
     *
     * @param log the log that will receive the embedded image
     * @param asset the asset file where the PNG should be written
     * @param width image width in pixels
     * @param height image height in pixels
     * @param userAabb optional user-space rectangle for mapped drawing; pass
     *                 null for ordinary pixel-space drawing
     * @param padding fractional padding used when fitting {@code userAabb}
     *                into the image
     * @param background optional background color; pass null for a transparent
     *                   background
     * @param style optional CSS style string for the generated {@code img} tag
     * @throws NullPointerException if {@code log} or {@code asset} is null
     * @throws IllegalArgumentException if {@code width} or {@code height} is not
     *                                  positive
     */
    LogGraphics(
            HtmlLog log,
            AssetFile asset,
            int width,
            int height,
            Rect userAabb,
            double padding,
            Color background,
            String style
    ) {
        Objects.requireNonNull(asset, "asset");
        if (width <= 0 || height <= 0) {
            throw new IllegalArgumentException("width and height must be positive");
        }

        this.log = Objects.requireNonNull(log, "log");
        this.path = asset.path();
        this.filename = asset.filename();
        this.style = style == null ? "" : style;
        this.image = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        this.graphics = image.createGraphics();

        graphics.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        graphics.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
        graphics.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);

        if (background != null) {
            graphics.setColor(background);
            graphics.fillRect(0, 0, width, height);
        }

        graphics.setColor(Color.BLACK);

        if (userAabb != null) {
            Rect device = new Rect(0, height, width, 0);
            AffineTransform tx = TransformTools.fitRectangle(userAabb, device, padding);
            graphics.setTransform(tx);
        }
    }

    /**
     * Returns the Java2D graphics context for this image.
     *
     * <p>The returned context remains owned by this {@code LogGraphics}
     * instance. Callers should not dispose it directly; it is disposed
     * automatically by {@link #close()}.</p>
     *
     * @return the graphics context used to draw into the backing image
     * @throws IllegalStateException if this drawing surface has already been
     *                               closed
     */
    public Graphics2D graphics() {
        ensureOpen();
        return graphics;
    }

    /**
     * Returns the backing image.
     *
     * <p>The returned image is the image that will be written as a PNG when this
     * object is closed. Mutating it after {@link #close()} is possible but will
     * not update the already-written PNG file.</p>
     *
     * @return the backing buffered image
     */
    public BufferedImage image() {
        return image;
    }

    /**
     * Returns the filesystem path where the PNG asset will be written.
     *
     * @return the output path for the generated PNG file
     */
    public Path path() {
        return path;
    }

    /**
     * Returns the filename used to reference the image from the generated HTML.
     *
     * @return the relative image filename
     */
    public String filename() {
        return filename;
    }

    /**
     * Returns whether this drawing surface has already been closed.
     *
     * @return {@code true} if {@link #close()} has been called; otherwise
     *         {@code false}
     */
    public boolean isClosed() {
        return closed;
    }

    /**
     * Closes this drawing surface, writes the PNG asset, and embeds it in the
     * associated log.
     *
     * <p>This method disposes the {@link Graphics2D} context, writes the
     * backing image to {@link #path()} using {@link ImageIO}, and appends an
     * {@code img} tag to the associated {@link HtmlLog}. Closing is idempotent:
     * if the surface is already closed, this method returns without doing
     * anything.</p>
     *
     * @throws UncheckedIOException if the PNG file cannot be written
     */
    @Override
    public void close() {
        if (closed) {
            return;
        }

        closed = true;
        graphics.dispose();

        try {
            ImageIO.write(image, "png", path.toFile());
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }

        log.embedImage(filename, style);
    }

    private void ensureOpen() {
        if (closed) {
            throw new IllegalStateException("LogGraphics is already closed");
        }
    }
}