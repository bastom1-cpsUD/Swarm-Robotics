package com.darcarms.htmllog;

import java.awt.Desktop;
import java.io.IOException;
import java.nio.file.Path;

/**
 * Browser launcher that uses {@link Desktop} to open a generated HtmlLog file.
 *
 * <p>This class is a small platform adapter around Java's desktop integration.
 * It is intentionally isolated from {@link HtmlLog} so the core logger can be
 * tested without opening a real browser or depending on desktop support in the
 * test environment.</p>
 */
final class DesktopBrowserLauncher implements BrowserLauncher {

    /**
     * Shared launcher instance.
     */
    static final DesktopBrowserLauncher INSTANCE = new DesktopBrowserLauncher();

    private DesktopBrowserLauncher() {
    }

    /**
     * Opens the supplied generated HTML file using Java desktop integration.
     *
     * <p>Opening is best-effort. If desktop browsing is unavailable, this
     * method returns without doing anything.</p>
     *
     * @param indexFile the generated {@code index.html} file
     * @throws IOException if {@link Desktop#browse(java.net.URI)} fails
     */
    @Override
    public void open(Path indexFile) throws IOException {
        if (!Desktop.isDesktopSupported()) {
            return;
        }

        Desktop desktop = Desktop.getDesktop();

        if (desktop.isSupported(Desktop.Action.BROWSE)) {
            desktop.browse(indexFile.toUri());
        }
    }
}