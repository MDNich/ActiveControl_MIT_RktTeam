package info.openrocket.swing.gui.main;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.SplashScreen;
import java.awt.font.GlyphVector;
import java.awt.geom.Rectangle2D;

import info.openrocket.core.util.BuildProperties;

/**
 * Helper methods for manipulating the Java runtime splash screen.
 * <p>
 * Notes:
 * SplashScreen.update() takes randomly between 4 and 500 ms to complete,
 * even after it has been called ~100 times (and therefore pre-compiled).
 * Therefore it cannot be relied upon to perform for example color fades.
 * Care should be taken to call update() only once or twice per second.
 * 
 * @author Sampo Niskanen <sampo.niskanen@iki.fi>
 */
public class Splash {
	
	// The right edge of the text base line for the version string
	private static final int VERSION_POSITION_X = 617;
	private static final int VERSION_POSITION_Y = 150;
	private static final Font VERSION_FONT = new Font(Font.SANS_SERIF, Font.PLAIN, 14);
	private static final Color VERSION_COLOR = Color.WHITE;
	
	
	/**
	 * Initialize the splash screen with additional information.  This method should
	 * be called as soon as reasonably possible during program startup.
	 * 
	 * @return	<code>true</code> if the splash screen could be successfully initialized
	 */
	public static boolean init() {
		// Get the splash screen
		SplashScreen s = getSplashScreen();
		if (s == null)
			return false;
		
		// Create graphics context and set antialiasing on
		Graphics2D g2 = s.createGraphics();
		g2.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL,
				RenderingHints.VALUE_STROKE_NORMALIZE);
		g2.setRenderingHint(RenderingHints.KEY_RENDERING,
				RenderingHints.VALUE_RENDER_QUALITY);
		g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
				RenderingHints.VALUE_ANTIALIAS_ON);
		
		// Draw the version number
		drawVersionNumber(g2);
		
		// Update the splash screen
		s.update();
		return true;
	}
	
	
	
	private static void drawVersionNumber(Graphics2D g2) {
		String text = "MIT Edition (v6)";
		String text2 = "Marc D. Nichițiu (RT)";
		String text3 = "nichitiu@mit.edu";
		GlyphVector gv = VERSION_FONT.createGlyphVector(g2.getFontRenderContext(), text);
		GlyphVector gv2 = VERSION_FONT.createGlyphVector(g2.getFontRenderContext(), text2);
		GlyphVector gv3 = VERSION_FONT.createGlyphVector(g2.getFontRenderContext(), text3);

		Rectangle2D rect = gv.getVisualBounds();
		Rectangle2D rect2 = gv2.getVisualBounds();
		Rectangle2D rect3 = gv3.getVisualBounds();
		double width = rect.getWidth();
		double width2 = rect2.getWidth();
		double width3 = rect3.getWidth();

		g2.setColor(VERSION_COLOR);
		g2.drawGlyphVector(gv, (float) (VERSION_POSITION_X - width), VERSION_POSITION_Y);
		g2.drawGlyphVector(gv2, (float) (VERSION_POSITION_X - width2), VERSION_POSITION_Y+((float) (rect.getHeight()*1.3)));
		g2.drawGlyphVector(gv3, (float) (VERSION_POSITION_X - width3), VERSION_POSITION_Y+((float) (rect.getHeight()*1.3 + rect2.getHeight()*1.15)));

	}
	
	
	/**
	 * Return the current splash screen or <code>null</code> if not available or already closed.
	 * This method catches the possible exceptions and returns null if they occur.
	 * 
	 * @return	the current (visible) splash screen, or <code>null</code>.
	 */
	public static SplashScreen getSplashScreen() {
		try {
			SplashScreen splash = SplashScreen.getSplashScreen();
			if (splash != null && splash.isVisible()) {
				return splash;
			} else {
				return null;
			}
		} catch (RuntimeException e) {
			return null;
		}
	}
	
	
	
}
