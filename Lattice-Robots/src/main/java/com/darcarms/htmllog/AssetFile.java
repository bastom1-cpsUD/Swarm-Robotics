package com.darcarms.htmllog;

import java.nio.file.Path;
import java.util.Objects;

/**
 * Represents a generated asset file stored beside an {@link HtmlLog} HTML file.
 *
 * <p>An asset is any file that the log references from {@code index.html},
 * such as a generated PNG image or serialized object snapshot. The
 * {@code path} component identifies where the file is written on disk, while
 * {@code filename} is the relative name used inside the generated HTML.</p>
 *
 * @param path the filesystem path where the asset should be written
 * @param filename the filename to reference from {@code index.html}
 */
public record AssetFile(Path path, String filename) {

    /**
     * Creates an asset file descriptor.
     *
     * @throws NullPointerException if {@code path} or {@code filename} is null
     */
    public AssetFile {
        Objects.requireNonNull(path, "path");
        Objects.requireNonNull(filename, "filename");
    }
}