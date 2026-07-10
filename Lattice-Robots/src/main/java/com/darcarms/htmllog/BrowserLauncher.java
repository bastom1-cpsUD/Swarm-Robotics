package com.darcarms.htmllog;

import java.io.IOException;
import java.nio.file.Path;

/**
 * Strategy interface for opening a generated HtmlLog file in a browser.
 *
 * <p>This package-private seam keeps browser launching testable without
 * requiring tests to use {@link java.awt.Desktop} or open a real browser.</p>
 */
@FunctionalInterface
interface BrowserLauncher {

    /**
     * Opens the supplied generated HTML file.
     *
     * @param indexFile the generated {@code index.html} file
     * @throws IOException if browser launching fails
     */
    void open(Path indexFile) throws IOException;
}