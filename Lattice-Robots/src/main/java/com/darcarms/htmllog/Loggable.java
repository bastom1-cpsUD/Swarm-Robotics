package com.darcarms.htmllog;

/**
 * Interface for objects that know how to render themselves into an
 * {@link HtmlLog}.
 *
 * <p>{@code Loggable} gives application/domain objects a clean way to
 * participate in HtmlLog's visual logging system. When
 * {@link HtmlLog#show(Object)} receives an object that implements this
 * interface, the logger delegates rendering to {@link #show(HtmlLog)}.</p>
 *
 * <p>Typical implementations use the supplied log to add headings, text,
 * nested groups, images, or Java2D drawings. For example, a geometry class
 * might create a mapped drawing with {@link HtmlLog#mappedGraphics(String, Rect, int, int)}
 * and draw itself into the returned {@link LogGraphics} object.</p>
 *
 * <pre>{@code
 * public final class PolygonModel implements Loggable {
 *     @Override
 *     public void show(HtmlLog log) throws Exception {
 *         log.heading("PolygonModel");
 *
 *         try (LogGraphics canvas =
 *                  log.mappedGraphics("polygon-", bounds(), 600, 400)) {
 *             draw(canvas.graphics());
 *         }
 *     }
 * }
 * }</pre>
 *
 * <p>Implementations may throw {@link Exception}. This keeps the interface
 * flexible for rendering code that writes files, generates images, or performs
 * other operations that may fail.</p>
 */
@FunctionalInterface
public interface Loggable {

    /**
     * Writes a representation of this object into the supplied log.
     *
     * <p>The method may write any supported log content, including text,
     * headings, groups, raw HTML, images, and Java2D-generated graphics.</p>
     *
     * @param log the log to write into
     * @throws Exception if rendering fails
     */
    void show(HtmlLog log) throws Exception;
}