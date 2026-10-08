package info.openrocket.swing.gui.flightcomputer;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.*;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/** Small vector component illustrations, sharp at the tree's current display scale. */
final class FlightComputerIcons implements Icon {
    private static final Map<String, Icon> ICONS = new ConcurrentHashMap<>();
    private final String type;
    static Icon of(String type) { return ICONS.computeIfAbsent(type, FlightComputerIcons::new); }
    private FlightComputerIcons(String type) { this.type = type; }
    public int getIconWidth() { return 30; }
    public int getIconHeight() { return 22; }
    public void paintIcon(Component c, Graphics raw, int x, int y) {
        var g = (Graphics2D) raw.create();
        g.translate(x, y); g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g.setStroke(new BasicStroke(1.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        Color ink = c.getForeground();
        g.setColor(new Color(66, 124, 151));
        switch (type) {
            case "board", "java_board", "design" -> {
                g.fillRoundRect(2, 3, 25, 16, 3, 3); g.setColor(new Color(196, 226, 230));
                g.drawRect(11, 7, 7, 8); g.drawLine(5, 7, 11, 7); g.drawLine(18, 12, 24, 12);
                g.drawLine(7, 16, 11, 12); g.fillOval(4, 6, 3, 3); g.fillOval(23, 11, 3, 3);
                if (type.equals("java_board")) { g.setColor(new Color(255, 208, 96)); g.drawString("J", 20, 10); }
            }
            case "processor", "storage" -> {
                g.setColor(ink); for (int i=5;i<20;i+=5) { g.drawLine(i,2,i,5);g.drawLine(i,17,i,20); }
                g.setColor(new Color(81,95,112));g.fillRoundRect(3,5,21,12,3,3);g.setColor(new Color(220,229,237));
                if(type.equals("storage")){g.drawLine(7,8,20,8);g.drawLine(7,11,20,11);g.drawLine(7,14,20,14);}
                else g.drawRect(9,8,9,6);
            }
            case "barometer" -> { g.setColor(ink);g.drawOval(6,3,17,17);g.drawArc(9,6,11,11,0,180);g.drawLine(14,13,19,8);g.fillOval(13,12,3,3); }
            case "accelerometer" -> { g.setColor(ink);g.drawLine(7,17,24,17);g.drawLine(7,17,7,2);g.drawLine(7,17,19,6);arrow(g,24,17,20,14);arrow(g,7,2,4,6);g.setColor(new Color(240,179,82));g.fillOval(4,14,6,6); }
            case "gyroscope" -> { g.setColor(ink);g.drawOval(3,7,23,9);g.drawOval(9,2,11,19);g.setColor(new Color(183,158,239));g.fillOval(12,9,5,5); }
            case "gps" -> { g.setColor(new Color(109,174,225));g.fillRect(2,7,7,9);g.fillRect(21,7,7,9);g.setColor(ink);g.drawLine(7,11,22,11);g.fillRoundRect(11,6,7,11,3,3);g.drawArc(9,0,11,11,35,110); }
            case "radio" -> { g.setColor(ink);g.drawLine(14,8,14,20);g.drawLine(14,13,9,20);g.drawLine(14,13,19,20);g.fillOval(12,6,4,4);g.drawArc(7,2,14,12,35,110);g.drawArc(3,0,22,17,30,120); }
            case "power" -> { g.setColor(new Color(112,185,134));g.fillRoundRect(3,5,22,14,3,3);g.fillRect(25,9,3,6);g.setColor(Color.WHITE);g.drawLine(7,12,13,12);g.drawLine(10,9,10,15);g.drawLine(18,12,22,12); }
            case "airbrakes" -> { g.setColor(ink);g.drawLine(14,3,14,20);g.setColor(new Color(241,162,103));g.fillPolygon(new int[]{13,3,8,13},new int[]{12,5,18,20},4);g.fillPolygon(new int[]{16,26,21,16},new int[]{12,5,18,20},4); }
            case "roll" -> { g.setColor(ink);g.drawArc(4,2,22,18,40,275);arrow(g,24,5,20,4);g.setColor(new Color(120,197,210));g.fillPolygon(new int[]{14,10,18},new int[]{3,18,18},3); }
            case "recovery" -> { g.setColor(new Color(214,149,214));g.fillArc(3,2,24,18,0,180);g.setColor(ink);g.drawLine(3,11,14,20);g.drawLine(27,11,14,20);g.drawLine(14,11,14,20);g.drawArc(9,2,11,18,0,180); }
            default -> {g.setColor(ink);g.drawRoundRect(4,5,21,12,3,3);for(int i=8;i<24;i+=5)g.fillOval(i,9,2,3);}
        }
        g.dispose();
    }
    private static void arrow(Graphics2D g,int x,int y,int a,int b){g.drawLine(x,y,a,b);g.drawLine(x,y,a,y+(y-b));}
}
