package info.openrocket.swing.util;

import com.google.inject.*;
import com.google.inject.util.Modules;
import info.openrocket.core.startup.Application;
import info.openrocket.core.plugin.PluginModule;
import info.openrocket.core.preferences.ApplicationPreferences;
import info.openrocket.core.l10n.*;
import info.openrocket.swing.gui.util.SwingPreferences;
import org.junit.jupiter.api.BeforeAll;

public class EnsembleSwingTestCase {
    /** Initializes Swing application services and isolated preferences for ensemble UI tests. */
    @BeforeAll
    public static void setupEnsembleSwing() {
        var preferences =
                new SwingPreferences() {
                    /**
                     * Builds or reads the get boolean fixture for the regression assertions.
                     *
                     * @param key configuration or lookup key
                     * @param fallback value used when no explicit setting is available
                     * @return true when the condition described above holds
                     */
                    @Override
                    public boolean getBoolean(String key, boolean fallback) {
                        return fallback;
                    }

                    /**
                     * Builds or reads the get double fixture for the regression assertions.
                     *
                     * @param key configuration or lookup key
                     * @param fallback value used when no explicit setting is available
                     * @return computed numeric value
                     */
                    @Override
                    public double getDouble(String key, double fallback) {
                        return fallback;
                    }

                    /**
                     * Builds or reads the get int fixture for the regression assertions.
                     *
                     * @param key configuration or lookup key
                     * @param fallback value used when no explicit setting is available
                     * @return computed integer value
                     */
                    @Override
                    public int getInt(String key, int fallback) {
                        return fallback;
                    }

                    /**
                     * Builds or reads the get string fixture for the regression assertions.
                     *
                     * @param key configuration or lookup key
                     * @param fallback value used when no explicit setting is available
                     * @return formatted or resolved text
                     */
                    @Override
                    public String getString(String key, String fallback) {
                        return fallback;
                    }
                };
        Application.setInjector(
                Guice.createInjector(
                        Modules.override(new info.openrocket.swing.ServicesForTesting())
                                .with(
                                        new AbstractModule() {
                                            /**
                                             * Configures isolated application dependencies for
                                             * these regression checks.
                                             */
                                            @Override
                                            protected void configure() {
                                                bind(ApplicationPreferences.class)
                                                        .toInstance(preferences);
                                                bind(Translator.class)
                                                        .toInstance(
                                                                new ResourceBundleTranslator(
                                                                        "l10n.messages"));
                                            }
                                        }),
                        new PluginModule()));
    }
}
