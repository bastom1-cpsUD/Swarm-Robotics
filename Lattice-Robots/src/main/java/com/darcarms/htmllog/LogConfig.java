package com.darcarms.htmllog;

/**
 * Complete logging configuration resolved for a particular logging call.
 *
 * <p>{@code LogConfig} contains the effective settings that determine whether
 * a logging action should write output, emit images, write serialized object
 * snapshots, or open the generated HTML file in a browser.</p>
 *
 * <p>Most callers do not create {@code LogConfig} directly. Instead,
 * {@link HtmlLog#getConfig()} starts from a default configuration and applies
 * each matching {@link ConfigRule}'s {@link LogConfigPatch} to produce one
 * complete configuration.</p>
 *
 * <p>This type is immutable. Methods such as {@link #withPatch(LogConfigPatch)}
 * return a new configuration rather than modifying the existing one.</p>
 *
 * @param active whether logging output should be written at all
 * @param images whether image assets should be written and embedded
 * @param serializables whether {@link java.io.Serializable} objects should be
 *                      written as linked {@code .ser} snapshots
 * @param browser whether the generated HTML log should be opened in a browser
 *                when first created
 * @param browserNew browser-opening mode, interpreted by {@link HtmlLog}
 */
public record LogConfig(
        boolean active,
        boolean images,
        boolean serializables,
        boolean browser,
        int browserNew
) {

    /**
     * Returns the default HtmlLog configuration.
     *
     * <p>The defaults are intended for interactive visual debugging:
     * logging is active, images are enabled, serializable snapshots are enabled,
     * and browser opening is enabled.</p>
     *
     * @return the default logging configuration
     */
    public static LogConfig defaults() {
        return new LogConfig(true, true, true, true, 2);
    }

    /**
     * Applies a partial configuration override to this configuration.
     *
     * <p>Only values explicitly present in {@code patch} are changed. All other
     * values are preserved from this configuration.</p>
     *
     * @param patch the partial configuration override to apply
     * @return a new configuration containing this configuration with the patch
     *         applied
     * @throws NullPointerException if {@code patch} is null
     */
    public LogConfig withPatch(LogConfigPatch patch) {
        return new LogConfig(
                patch.active().orElse(active),
                patch.images().orElse(images),
                patch.serializables().orElse(serializables),
                patch.browser().orElse(browser),
                patch.browserNew().orElse(browserNew)
        );
    }
}