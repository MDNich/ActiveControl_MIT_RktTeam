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
    @BeforeAll public static void setupEnsembleSwing() {
        var preferences=new SwingPreferences() {
            @Override public boolean getBoolean(String key,boolean fallback) { return fallback; }
            @Override public double getDouble(String key,double fallback) { return fallback; }
            @Override public int getInt(String key,int fallback) { return fallback; }
            @Override public String getString(String key,String fallback) { return fallback; }
        };
        Application.setInjector(Guice.createInjector(Modules.override(new info.openrocket.swing.ServicesForTesting()).with(new AbstractModule() {
            @Override protected void configure() {
                bind(ApplicationPreferences.class).toInstance(preferences);
                bind(Translator.class).toInstance(new ResourceBundleTranslator("l10n.messages"));
            }
        }),new PluginModule()));
    }
}
