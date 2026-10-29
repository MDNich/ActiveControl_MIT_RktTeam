package info.openrocket.swing.gui.mission;

import jakarta.json.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.net.URI;
import java.net.http.*;
import java.time.Duration;
import java.util.*;
import java.util.concurrent.*;
import java.util.function.*;
import javax.imageio.ImageIO;
import javax.swing.*;

/** Native slippy map: bounded background tile requests, drag pins, pan and wheel zoom. */
final class MissionMapPanel extends JPanel implements AutoCloseable {
  private JsonArray sites = JsonValue.EMPTY_JSON_ARRAY;
  private String selected = "launch-site", layer = "hybrid";
  private double latitude = 42.70416776971639, longitude = -77.19029501648315;
  private int zoom = 15;
  private Point pressed;
  private String dragged;
  private boolean placing;
  private final BiConsumer<String, double[]> moved;
  private final Consumer<String> chosen;
  private final ExecutorService fetch =
      Executors.newFixedThreadPool(
          3,
          r -> {
            Thread t = new Thread(r, "mission-map");
            t.setDaemon(true);
            return t;
          });
  private final HttpClient http =
      HttpClient.newBuilder()
          .connectTimeout(Duration.ofSeconds(5))
          .followRedirects(HttpClient.Redirect.NORMAL)
          .build();
  private final Map<String, BufferedImage> images =
      Collections.synchronizedMap(
          new LinkedHashMap<>(256, .75f, true) {
            protected boolean removeEldestEntry(Map.Entry<String, BufferedImage> e) {
              return size() > 192;
            }
          });
  private final Set<String> pending = ConcurrentHashMap.newKeySet();
  private final Map<String, Long> failures = new ConcurrentHashMap<>();

  MissionMapPanel(BiConsumer<String, double[]> moved, Consumer<String> chosen) {
    this.moved = moved;
    this.chosen = chosen;
    setName("mission-map");
    setPreferredSize(new Dimension(650, 520));
    setBackground(new Color(224, 232, 231));
    setToolTipText("Drag a station to move it. Drag the map to pan; use the wheel to zoom.");
    MouseAdapter mouse =
        new MouseAdapter() {
          public void mousePressed(MouseEvent e) {
            pressed = e.getPoint();
            dragged = hit(pressed);
            if (dragged != null) {
              selected = dragged;
              chosen.accept(selected);
            }
          }

          public void mouseDragged(MouseEvent e) {
            if (pressed == null) return;
            if (dragged != null) {
              double[] ll = coordinates(e.getPoint());
              preview(dragged, ll);
            } else {
              double[] p = pixel(latitude, longitude);
              double[] ll = inverse(p[0] - (e.getX() - pressed.x), p[1] - (e.getY() - pressed.y));
              latitude = ll[0];
              longitude = ll[1];
              pressed = e.getPoint();
            }
            repaint();
          }

          public void mouseReleased(MouseEvent e) {
            if (dragged != null) moved.accept(dragged, coordinates(e.getPoint()));
            else if (placing) {
              moved.accept(selected, coordinates(e.getPoint()));
              placing = false;
              setCursor(Cursor.getDefaultCursor());
            }
            pressed = null;
            dragged = null;
          }

          public void mouseWheelMoved(MouseWheelEvent e) {
            zoom = Math.max(2, Math.min(19, zoom - e.getWheelRotation()));
            repaint();
          }
        };
    addMouseListener(mouse);
    addMouseMotionListener(mouse);
    addMouseWheelListener(mouse);
  }

  void update(JsonArray sites, String selected) {
    this.sites = sites;
    this.selected = selected;
    repaint();
  }

  void layer(String layer) {
    this.layer = layer;
    repaint();
  }

  void place() {
    placing = true;
    setCursor(Cursor.getPredefinedCursor(Cursor.CROSSHAIR_CURSOR));
  }

  void zoom(int delta) {
    zoom = Math.max(2, Math.min(19, zoom + delta));
    repaint();
  }

  void fit() {
    if (sites.isEmpty()) return;
    double minLat = 90, maxLat = -90, minLon = 180, maxLon = -180;
    for (var v : sites) {
      var s = v.asJsonObject();
      double la = s.getJsonNumber("latitudeDeg").doubleValue(),
          lo = s.getJsonNumber("longitudeDeg").doubleValue();
      minLat = Math.min(minLat, la);
      maxLat = Math.max(maxLat, la);
      minLon = Math.min(minLon, lo);
      maxLon = Math.max(maxLon, lo);
    }
    latitude = (minLat + maxLat) / 2;
    longitude = (minLon + maxLon) / 2;
    zoom = 18;
    while (zoom > 2) {
      double[] a = pixel(minLat, minLon), b = pixel(maxLat, maxLon);
      if (Math.abs(a[0] - b[0]) < Math.max(200, getWidth() - 100)
          && Math.abs(a[1] - b[1]) < Math.max(200, getHeight() - 100)) break;
      zoom--;
    }
    repaint();
  }

  private double[] pixel(double lat, double lon) {
    double n = 256 * Math.pow(2, zoom),
        s = Math.sin(Math.toRadians(Math.max(-85.0511, Math.min(85.0511, lat))));
    return new double[] {
      (lon + 180) / 360 * n, (.5 - Math.log((1 + s) / (1 - s)) / (4 * Math.PI)) * n
    };
  }

  private double[] inverse(double x, double y) {
    double n = 256 * Math.pow(2, zoom);
    return new double[] {
      Math.max(
          -85.0511,
          Math.min(85.0511, Math.toDegrees(Math.atan(Math.sinh(Math.PI * (1 - 2 * y / n)))))),
      ((x / n * 360) % 360 + 360) % 360 - 180
    };
  }

  private double[] coordinates(Point p) {
    double[] c = pixel(latitude, longitude);
    return inverse(c[0] + p.x - getWidth() / 2., c[1] + p.y - getHeight() / 2.);
  }

  private Point point(JsonObject s) {
    double[]
        p =
            pixel(
                s.getJsonNumber("latitudeDeg").doubleValue(),
                s.getJsonNumber("longitudeDeg").doubleValue()),
        c = pixel(latitude, longitude);
    return new Point((int) (p[0] - c[0] + getWidth() / 2.), (int) (p[1] - c[1] + getHeight() / 2.));
  }

  private String hit(Point p) {
    for (var value : sites) {
      var s = value.asJsonObject();
      if (point(s).distance(p) < 20) return s.getString("id");
    }
    return null;
  }

  private void preview(String id, double[] ll) {
    var next = Json.createArrayBuilder();
    for (var v : sites)
      next.add(
          v.asJsonObject().getString("id").equals(id)
              ? Json.createObjectBuilder(v.asJsonObject())
                  .add("latitudeDeg", ll[0])
                  .add("longitudeDeg", ll[1])
                  .build()
              : v);
    sites = next.build();
  }

  protected void paintComponent(Graphics graphics) {
    super.paintComponent(graphics);
    Graphics2D g = (Graphics2D) graphics.create();
    g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
    double[] c = pixel(latitude, longitude);
    double left = c[0] - getWidth() / 2., top = c[1] - getHeight() / 2.;
    if (!layer.equals("offline"))
      for (int x = (int) Math.floor(left / 256); x * 256 < left + getWidth(); x++)
        for (int y = (int) Math.floor(top / 256); y * 256 < top + getHeight(); y++) {
          int n = 1 << zoom;
          if (y < 0 || y >= n) continue;
          int wrapped = Math.floorMod(x, n);
          String key = layer + "/" + zoom + "/" + wrapped + "/" + y;
          var tile = images.get(key);
          int px = (int) (x * 256 - left), py = (int) (y * 256 - top);
          if (tile != null) g.drawImage(tile, px, py, 256, 256, null);
          else {
            g.setColor(new Color(204, 216, 215));
            g.drawRect(px, py, 256, 256);
            request(key, wrapped, y, zoom, layer);
          }
        }
    for (var v : sites) {
      var s = v.asJsonObject();
      Point p = point(s);
      String id = s.getString("id");
      g.setColor(
          !s.getBoolean("enabled")
              ? Color.GRAY
              : id.equals("launch-site") ? new Color(175, 56, 44) : new Color(21, 105, 99));
      g.fillOval(p.x - 15, p.y - 15, 30, 30);
      g.setStroke(new BasicStroke(id.equals(selected) ? 3 : 1));
      g.setColor(Color.WHITE);
      g.drawOval(p.x - 16, p.y - 16, 32, 32);
      String text = id.equals("launch-site") ? "L" : id.equals("launch") ? "B" : id.substring(2);
      g.setFont(getFont().deriveFont(Font.BOLD, 13));
      g.drawString(text, p.x - g.getFontMetrics().stringWidth(text) / 2, p.y + 5);
    }
    g.setColor(new Color(255, 255, 255, 220));
    g.fillRect(0, getHeight() - 25, getWidth(), 25);
    g.setColor(Color.DARK_GRAY);
    g.setFont(getFont().deriveFont(11f));
    g.drawString(
        layer.equals("street")
            ? "© OpenStreetMap contributors · Drag pins / pan / scroll to zoom"
            : layer.equals("offline")
                ? "Offline map · Coordinates and placement remain available"
                : "USGS · The National Map · Drag pins / pan / scroll to zoom",
        8,
        getHeight() - 8);
    g.dispose();
  }

  private void request(String key, int x, int y, int z, String layer) {
    if (pending.size() >= 48
        || pending.contains(key)
        || System.currentTimeMillis() - failures.getOrDefault(key, 0L) < 30_000) return;
    if (!pending.add(key)) return;
    fetch.execute(
        () -> {
          try {
            String url =
                layer.equals("street")
                    ? "https://tile.openstreetmap.org/" + z + "/" + x + "/" + y + ".png"
                    : "https://basemap.nationalmap.gov/arcgis/rest/services/"
                        + (layer.equals("hybrid") ? "USGSImageryTopo" : "USGSImageryOnly")
                        + "/MapServer/tile/"
                        + z
                        + "/"
                        + y
                        + "/"
                        + x;
            var response =
                http.send(
                    HttpRequest.newBuilder(URI.create(url))
                        .header("User-Agent", "OpenRocket-MIT/6.5 Mission map")
                        .timeout(Duration.ofSeconds(8))
                        .build(),
                    HttpResponse.BodyHandlers.ofByteArray());
            if (response.statusCode() != 200 || response.body().length > 2_000_000)
              throw new java.io.IOException("Tile unavailable");
            var image = ImageIO.read(new java.io.ByteArrayInputStream(response.body()));
            if (image == null) throw new java.io.IOException("No image");
            images.put(key, image);
          } catch (Exception e) {
            if (failures.size() > 512) failures.clear();
            failures.put(key, System.currentTimeMillis());
          } finally {
            pending.remove(key);
            SwingUtilities.invokeLater(this::repaint);
          }
        });
  }

  public void close() {
    fetch.shutdownNow();
    images.clear();
    failures.clear();
  }
}
