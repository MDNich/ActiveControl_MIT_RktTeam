package info.openrocket.swing.gui.configdialog;

import info.openrocket.core.rocketcomponent.FinSet;
import info.openrocket.swing.gui.rocketfigure.FinProfileGeometry;
import javax.swing.*;
import java.awt.*;
import java.awt.geom.Path2D;

/** Enlarged cross-section sketch; geometry in the rocket view retains the true thickness. */
final class FinSectionPreview extends JPanel {
    private final FinSet fin;
    FinSectionPreview(FinSet fin) {
        this.fin = fin;
        setName("fin.sectionPreview");
        setPreferredSize(new Dimension(260, 100));
        setToolTipText("Illustrative cross-section, enlarged for clarity. Airfoil is a generic symmetric section, not a specified airfoil series.");
    }
    @Override protected void paintComponent(Graphics graphics) {
        super.paintComponent(graphics);
        var g = (Graphics2D) graphics.create();
        try {
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            double chord = Math.max(1, getWidth() - 30), thickness = Math.min(38, getHeight() - 45);
            double middle = getHeight() / 2.0 + 6;
            var path = new Path2D.Double();
            for (int side : new int[]{1, -1}) {
                for (int i = 0; i <= 256; i++) {
                    double x = chord * (side == 1 ? i : 256 - i) / 256;
                    double half = FinProfileGeometry.halfThickness(fin.getCrossSection(), x, chord, thickness,
                            fin.getLeadingEdgeDistance());
                    if (side == 1 && i == 0) path.moveTo(15 + x, middle - half);
                    else path.lineTo(15 + x, middle - side * half);
                }
            }
            path.closePath();
            g.setColor(new Color(180, 205, 230));
            g.fill(path);
            g.setColor(getForeground());
            g.draw(path);
            g.drawString(fin.getCrossSection() + " section — enlarged", 15, 15);
        } finally { g.dispose(); }
    }
}
