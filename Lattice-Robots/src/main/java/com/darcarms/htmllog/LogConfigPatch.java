package com.darcarms.htmllog;

import java.util.Optional;

/**
 * Partial override for a {@link LogConfig}.
 * 
 * <p>{@code LogConfigPatch} is used by {@link ConfigRule}. Each rule may
 * change only the settings it cares about, while leaving all other settings
 * unchanged. This allows small targeted overrides, such as disabling images or
 * serializable snapshots, without restating the full configuration.</p>
 *
 * <p>All fields are optional. An empty optional means "do not change this
 * setting." To apply a patch to a complete configuration, use
 * {@link LogConfig#withPatch(LogConfigPatch)}.</p>
 *
 * @param active optional override for whether logging output should be written
 *               at all
 * @param images optional override for whether image assets should be written
 *               and embedded
 * @param serializables optional override for whether {@link java.io.Serializable}
 *                      objects should be written as linked {@code .ser}
 *                      snapshots
 * @param browser optional override for whether the log should open in a browser
 *                when first created
 * @param browserNew optional override for the browser-opening mode used by
 *                   {@link HtmlLog}
 */
public record LogConfigPatch(
        Optional<Boolean> active,
        Optional<Boolean> images,
        Optional<Boolean> serializables,
        Optional<Boolean> browser,
        Optional<Integer> browserNew
) {

    /**
     * Creates a configuration patch.
     *
     * <p>Null optionals are normalized to {@link Optional#empty()} so callers
     * cannot accidentally create a patch whose optional fields are themselves
     * null.</p>
     *
     * @param active optional override for active logging
     * @param images optional override for image logging
     * @param serializables optional override for serializable snapshots
     * @param browser optional override for browser opening
     * @param browserNew optional override for browser-opening mode
     */
    public LogConfigPatch {
        active = active == null ? Optional.empty() : active;
        images = images == null ? Optional.empty() : images;
        serializables = serializables == null ? Optional.empty() : serializables;
        browser = browser == null ? Optional.empty() : browser;
        browserNew = browserNew == null ? Optional.empty() : browserNew;
    }

    /**
     * Returns an empty patch that changes no settings.
     *
     * @return a patch with no overrides
     */
    public static LogConfigPatch empty() {
        return builder().build();
    }

    /**
     * Returns a new builder for constructing a configuration patch.
     *
     * @return a new patch builder
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Mutable builder for {@link LogConfigPatch}.
     *
     * <p>The builder uses boxed values internally so that unset values can be
     * distinguished from explicitly supplied {@code false} or {@code 0}
     * values.</p>
     */
    public static final class Builder {
        private Boolean active;
        private Boolean images;
        private Boolean serializables;
        private Boolean browser;
        private Integer browserNew;

        private Builder() {
        }

        /**
         * Sets whether logging should be active.
         *
         * @param active {@code true} to write log output; {@code false} to
         *               suppress output
         * @return this builder
         */
        public Builder active(boolean active) {
            this.active = active;
            return this;
        }

        /**
         * Sets whether image assets should be written and embedded.
         *
         * @param images {@code true} to write image assets; {@code false} to
         *               suppress image output
         * @return this builder
         */
        public Builder images(boolean images) {
            this.images = images;
            return this;
        }

        /**
         * Sets whether serializable object snapshots should be written.
         *
         * @param serializables {@code true} to write {@code .ser} snapshots;
         *                      {@code false} to suppress them
         * @return this builder
         */
        public Builder serializables(boolean serializables) {
            this.serializables = serializables;
            return this;
        }

        /**
         * Sets whether the generated HTML log should open in a browser.
         *
         * @param browser {@code true} to open the log when first created;
         *                {@code false} to avoid opening it automatically
         * @return this builder
         */
        public Builder browser(boolean browser) {
            this.browser = browser;
            return this;
        }

        /**
         * Sets the browser-opening mode.
         *
         * <p>The meaning of this value is interpreted by {@link HtmlLog}.</p>
         *
         * @param browserNew the browser-opening mode
         * @return this builder
         */
        public Builder browserNew(int browserNew) {
            this.browserNew = browserNew;
            return this;
        }

        /**
         * Builds the immutable configuration patch.
         *
         * @return a new patch containing the values set on this builder
         */
        public LogConfigPatch build() {
            return new LogConfigPatch(
                    Optional.ofNullable(active),
                    Optional.ofNullable(images),
                    Optional.ofNullable(serializables),
                    Optional.ofNullable(browser),
                    Optional.ofNullable(browserNew)
            );
        }
    }
}