package info.openrocket.swing.gui.mission;

import info.openrocket.core.document.*;
import info.openrocket.core.simulation.mission.*;
import jakarta.json.*;
import java.awt.*;
import java.awt.event.*;
import java.util.*;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.function.Consumer;
import javax.swing.*;
import javax.swing.table.DefaultTableModel;

/**
 * Native mission configuration and run window. All simulator/file/network work stays off the EDT.
 */
public final class MissionWindow extends JFrame {
  private final MissionService service;
  private OpenRocketDocument currentDocument;
  private JsonObject mission, job, opened;
  private final List<MissionForm> forms = new ArrayList<>();
  private boolean changed, rendering, busy, polling;
  private String selectedSite = "launch-site", selectedLink = "launch";
  private final JTabbedPane tabs = new JTabbedPane();
  private final JTextField title = new JTextField(28);
  private final JLabel status = new JLabel(" "), runStatus = new JLabel("No run started");
  private final JTextArea findings = new JTextArea(4, 70),
      runDetails = new JTextArea(5, 70),
      networkDetails = new JTextArea(10, 60);
  private final JProgressBar progress = new JProgressBar();
  private final JPanel resultsPanel = new JPanel(new BorderLayout());
  private final JPanel runPanel = new JPanel(new BorderLayout(8, 8));
  private final JComboBox<MissionForm.Choice> simulations = new JComboBox<>();
  private final MissionMapPanel map;
  private final javax.swing.Timer timer;
  private String resolvedHash;

  public MissionWindow(MissionService service, OpenRocketDocument document, JsonObject draft) {
    super("Mission Simulation — OpenRocket MIT");
    this.service = service;
    currentDocument = document;
    mission = defaults(draft);
    setName("mission-window");
    setDefaultCloseOperation(HIDE_ON_CLOSE);
    setMinimumSize(new Dimension(980, 680));
    setSize(1240, 850);
    map =
        new MissionMapPanel(
            this::moveSite,
            id -> {
              if (rendering) return;
              try {
                commitEditors();
                selectedSite = id;
                render();
              } catch (Exception e) {
                error(e);
              }
            });
    JPanel toolbar = new JPanel(new FlowLayout(FlowLayout.LEFT, 8, 8));
    toolbar.add(new JLabel("Mission"));
    title.setName("mission-title");
    toolbar.add(title);
    toolbar.add(
        button(
            "New",
            "mission-new",
            () ->
                perform(
                    "Creating mission",
                    () -> {
                      saveDraft();
                      return service.request(
                          "POST",
                          "drafts",
                          Json.createObjectBuilder().add("title", "URRG mission").build());
                    },
                    this::use)));
    toolbar.add(
        button(
            "Load…",
            "mission-load",
            () ->
                perform(
                    "Loading configuration",
                    () -> {
                      saveDraft();
                      return service.request("POST", "drafts/load", JsonValue.EMPTY_JSON_OBJECT);
                    },
                    v -> {
                      if (!v.containsKey("cancelled")) use(v);
                    })));
    toolbar.add(
        button(
            "Save…",
            "mission-save",
            () ->
                draftAction(
                    "save",
                    JsonValue.EMPTY_JSON_OBJECT,
                    v ->
                        status.setText(
                            v.containsKey("saved")
                                ? "Saved " + v.getString("saved")
                                : "Save cancelled"))));
    toolbar.add(
        button(
            "Portable setup…",
            "mission-export",
            () ->
                draftAction(
                    "export",
                    JsonValue.EMPTY_JSON_OBJECT,
                    v ->
                        status.setText(
                            v.containsKey("saved")
                                ? "Exported " + v.getString("saved")
                                : "Export cancelled"))));
    title.addActionListener(
        e -> {
          set("/identity/title", Json.createValue(title.getText().trim()));
          setTitle(title.getText() + " — Mission Simulation");
        });
    add(toolbar, BorderLayout.NORTH);
    add(tabs, BorderLayout.CENTER);
    status.setBorder(BorderFactory.createEmptyBorder(7, 12, 7, 12));
    add(status, BorderLayout.SOUTH);
    tabs.addChangeListener(
        e -> {
          if (!rendering && !busy)
            try {
              commitEditors();
            } catch (Exception ex) {
              error(ex);
            }
        });
    findings.setEditable(false);
    findings.setLineWrap(true);
    findings.setWrapStyleWord(true);
    findings.setName("mission-findings");
    runDetails.setEditable(false);
    runDetails.setLineWrap(true);
    runDetails.setWrapStyleWord(true);
    runDetails.setName("mission-run-details");
    networkDetails.setEditable(false);
    networkDetails.setName("mission-network-endpoints");
    timer = new javax.swing.Timer(500, e -> poll());
    timer.start();
    render();
    setLocationByPlatform(true);
    SwingUtilities.invokeLater(map::fit);
  }

  public void currentDocument(OpenRocketDocument value) {
    currentDocument = value;
  }

  public OpenRocketDocument documentCopy() {
    var copy = currentDocument.copy();
    copy.setFile(currentDocument.getFile());
    return copy;
  }

  private JsonObject defaults(JsonObject value) {
    var v = Json.createObjectBuilder(value.getJsonObject("video"));
    for (var entry :
        Map.of(
                "frameRateHz",
                30,
                "width",
                960,
                "height",
                540,
                "lossFraction",
                0,
                "delayMs",
                0,
                "analogNoiseFraction",
                0)
            .entrySet())
      if (!value.getJsonObject("video").containsKey(entry.getKey()))
        v.add(entry.getKey(), entry.getValue());
    if (!value.getJsonObject("video").containsKey("outages"))
      v.add("outages", JsonValue.EMPTY_JSON_ARRAY);
    var telemetry = Json.createObjectBuilder(value.getJsonObject("telemetry"));
    if (!value.getJsonObject("telemetry").containsKey("noise"))
      telemetry.add(
          "noise",
          Json.createObjectBuilder()
              .add("gpsHorizontalSigmaM", 0)
              .add("gpsVerticalSigmaM", 0)
              .add("barometerSigmaM", 0));
    if (!value.getJsonObject("telemetry").containsKey("gpsRateHz")) telemetry.add("gpsRateHz", 10);
    if (!value.getJsonObject("telemetry").containsKey("barometerRateHz"))
      telemetry.add("barometerRateHz", 20);
    var links = Json.createArrayBuilder(value.getJsonObject("telemetry").getJsonArray("links"));
    for (var item : value.getJsonArray("sites")) {
      String id = item.asJsonObject().getString("id");
      if (id.equals("launch-site")
          || value.getJsonObject("telemetry").getJsonArray("links").stream()
              .anyMatch(l -> l.asJsonObject().getString("receiverId").equals(id))) continue;
      links.add(
          Json.createObjectBuilder()
              .add("receiverId", id)
              .add("packetLossFraction", 0)
              .add("delayMs", 0)
              .add("jitterMs", 0)
              .add("bitErrorProbability", 0)
              .add("uartBaud", 0)
              .add("outages", JsonValue.EMPTY_JSON_ARRAY)
              .add("directional", false)
              .add("beamWidthDeg", 60));
    }
    telemetry.add("links", links);
    var execution = Json.createObjectBuilder(value.getJsonObject("execution"));
    if (!value.getJsonObject("execution").containsKey("clockMode"))
      execution.add("clockMode", "paced");
    if (!value.getJsonObject("execution").containsKey("applicationMember"))
      execution.add("applicationMember", 0);
    if (!value.getJsonObject("execution").containsKey("actions"))
      execution.add("actions", JsonValue.EMPTY_JSON_ARRAY);
    var outputs = Json.createObjectBuilder(value.getJsonObject("outputs"));
    if (!value.getJsonObject("outputs").containsKey("filename"))
      outputs.add("filename", "mission.rktmission");
    var bindings = Json.createArrayBuilder();
    for (var item : value.getJsonArray("stationBindings")) {
      var b = item.asJsonObject();
      if (!b.getString("role").equals("telem")) continue;
      var builder = Json.createObjectBuilder(b).add("owner", "headless-policy");
      if (!b.containsKey("targetVehicleId"))
        builder.add(
            "targetVehicleId",
            value.getJsonArray("vehicleBindings").getJsonObject(0).getString("id"));
      bindings.add(builder);
    }
    var next =
        Json.createObjectBuilder(value)
            .add("video", v)
            .add("telemetry", telemetry)
            .add("execution", execution)
            .add("outputs", outputs)
            .add("stationBindings", bindings);
    if (value.getJsonArray("stationBindings").stream()
        .anyMatch(b -> !b.asJsonObject().getString("owner").equals("headless-policy")))
      next.add("legacyStationBindings", value.getJsonArray("stationBindings"));
    if (!value.containsKey("network"))
      next.add(
          "network",
          Json.createObjectBuilder()
              .add("enabled", true)
              .add("video", true)
              .add("bindHost", "0.0.0.0")
              .add(
                  "vehicleMode",
                  value.getJsonObject("video").getString("preset", "Balius").equals("Iris")
                      ? "iris"
                      : "balius"));
    return next.build();
  }

  private void use(JsonObject value) {
    mission = defaults(value);
    resolvedHash = null;
    changed = false;
    render();
    status.setText("Configuration loaded");
  }

  private String id() {
    return MissionStore.id(mission);
  }

  private int revision() {
    return mission.getJsonObject("identity").getInt("revision");
  }

  private void dirty() {
    if (!rendering) {
      changed = true;
      resolvedHash = null;
      status.setText("Configuration has unsaved changes");
    }
  }

  private void set(String path, JsonValue value) {
    if (Objects.equals(MissionForm.value(mission, path), value)) return;
    mission = Json.createPointer(path).add(mission, value);
    dirty();
  }

  private MissionForm form() {
    var form = new MissionForm(() -> mission, this::set, this::dirty);
    forms.add(form);
    return form;
  }

  private static JButton button(String label, String name, Runnable action) {
    var b = new JButton(label);
    b.setName(name);
    b.addActionListener(e -> action.run());
    return b;
  }

  private static JPanel row(Component... values) {
    var p = new JPanel(new FlowLayout(FlowLayout.LEFT, 8, 5));
    for (var v : values) p.add(v);
    return p;
  }

  private static JScrollPane scroll(Component component) {
    var pane = new JScrollPane(component);
    pane.setBorder(BorderFactory.createEmptyBorder());
    pane.getVerticalScrollBar().setUnitIncrement(18);
    return pane;
  }

  private static JPanel page(Component component) {
    var p = new JPanel(new BorderLayout(8, 8));
    p.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
    p.add(component, BorderLayout.CENTER);
    return p;
  }

  private void commitEditors() {
    for (var f : forms) f.commit();
    set("/identity/title", Json.createValue(title.getText().trim()));
  }

  private void saveDraft() throws Exception {
    mission =
        service.request(
            "PUT",
            "drafts/" + id(),
            Json.createObjectBuilder()
                .add("expectedRevision", revision())
                .add("mission", mission)
                .build());
    changed = false;
  }

  private <T> void perform(String message, Callable<T> operation, Consumer<T> done) {
    if (busy) return;
    try {
      commitEditors();
    } catch (Exception e) {
      error(e);
      return;
    }
    busy = true;
    status.setText(message + "…");
    var glass = new JPanel();
    glass.setOpaque(false);
    glass.addMouseListener(new MouseAdapter() {});
    glass.setCursor(Cursor.getPredefinedCursor(Cursor.WAIT_CURSOR));
    setGlassPane(glass);
    glass.setVisible(true);
    glass.setFocusable(true);
    glass.requestFocusInWindow();
    new SwingWorker<T, Void>() {
      protected T doInBackground() throws Exception {
        return operation.call();
      }

      protected void done() {
        busy = false;
        glass.setVisible(false);
        try {
          done.accept(get());
        } catch (Exception e) {
          error(e.getCause() == null ? e : e.getCause());
        }
      }
    }.execute();
  }

  private void error(Throwable e) {
    status.setText(e.getMessage() == null ? e.toString() : e.getMessage());
    JOptionPane.showMessageDialog(
        this, status.getText(), "Mission Simulation", JOptionPane.ERROR_MESSAGE);
  }

  private void draftAction(String action, JsonObject body, Consumer<JsonObject> done) {
    perform(
        "Working",
        () -> {
          saveDraft();
          return service.request(
              "POST",
              "drafts/" + id() + "/" + action,
              Json.createObjectBuilder(body).add("expectedRevision", revision()).build());
        },
        done);
  }

  private void render() {
    rendering = true;
    int selected = Math.max(0, tabs.getSelectedIndex());
    forms.clear();
    tabs.removeAll();
    title.setText(mission.getJsonObject("identity").getString("title"));
    tabs.addTab("Site & pointers", sitesPage());
    tabs.addTab("Rocket & flight", flightPage());
    tabs.addTab("Telemetry", telemetryPage());
    tabs.addTab("Video", videoPage());
    tabs.addTab("Network", networkPage());
    tabs.addTab("Run & results", runPage());
    tabs.setSelectedIndex(Math.min(selected, tabs.getTabCount() - 1));
    rendering = false;
  }

  private JPanel sitesPage() {
    JPanel page = page(map);
    var list = new JComboBox<MissionForm.Choice>();
    list.setName("mission-site");
    for (var v : mission.getJsonArray("sites")) {
      var s = v.asJsonObject();
      var choice = new MissionForm.Choice(s.getString("id"), siteName(s.getString("id")));
      list.addItem(choice);
      if (choice.value().equals(selectedSite)) list.setSelectedItem(choice);
    }
    list.addActionListener(
        e -> {
          if (!rendering)
            try {
              commitEditors();
              selectedSite = ((MissionForm.Choice) list.getSelectedItem()).value();
              render();
            } catch (Exception ex) {
              error(ex);
            }
        });
    var count =
        new JSpinner(
            new SpinnerNumberModel(
                (int)
                    mission.getJsonArray("pointers").stream()
                        .filter(p -> p.asJsonObject().getBoolean("enabled"))
                        .count(),
                0,
                4,
                1));
    count.setName("mission-pointer-count");
    count.addChangeListener(
        e -> {
          if (!rendering)
            try {
              commitEditors();
              int n = (Integer) count.getValue();
              var pointers = Json.createArrayBuilder();
              for (int i = 1; i <= 4; i++) {
                final String site = "gs" + i;
                var previous =
                    mission.getJsonArray("pointers").stream()
                        .map(JsonValue::asJsonObject)
                        .filter(p -> p.getString("siteId").equals(site))
                        .findFirst()
                        .orElse(
                            Json.createObjectBuilder()
                                .add("id", "pointer" + i)
                                .add("siteId", site)
                                .add(
                                    "targetVehicleId",
                                    mission
                                        .getJsonArray("vehicleBindings")
                                        .getJsonObject(0)
                                        .getString("id"))
                                .add("trackingPolicy", "geodetic.v1")
                                .add("initialAzimuthDeg", 0)
                                .add("initialElevationDeg", 0)
                                .add("azimuthRateDegS", 90)
                                .add("elevationRateDegS", 60)
                                .add("mountHeightM", 2)
                                .build());
                pointers.add(Json.createObjectBuilder(previous).add("enabled", i <= n));
                if (i <= n) set("/sites/" + siteIndex(site) + "/enabled", JsonValue.TRUE);
              }
              set("/pointers", pointers.build());
              render();
            } catch (Exception ex) {
              error(ex);
            }
        });
    var layers = new JComboBox<>(new String[] {"Hybrid", "Satellite", "Street", "Offline"});
    layers.setName("mission-map-layer");
    layers.addActionListener(
        e ->
            map.layer(
                new String[] {"hybrid", "satellite", "street", "offline"}
                    [layers.getSelectedIndex()]));
    page.add(
        row(
            new JLabel("Antenna pointers"),
            count,
            layers,
            button("Fit sites", "mission-map-fit", map::fit),
            button("+", "mission-map-plus", () -> map.zoom(1)),
            button("−", "mission-map-minus", () -> map.zoom(-1))),
        BorderLayout.NORTH);
    var f = form();
    f.addRow("Selected site", list);
    int i = siteIndex(selectedSite);
    String p = "/sites/" + i;
    var site = mission.getJsonArray("sites").getJsonObject(i);
    if (selectedSite.startsWith("gs")) f.toggle(p + "/enabled", "Receive telemetry at this site");
    f.number(p + "/latitudeDeg", "Latitude (°)");
    f.number(p + "/longitudeDeg", "Longitude (°)");
    var mgrs =
        new JTextField(
            site.getJsonObject("coordinateInput") != null
                    && site.getJsonObject("coordinateInput").getString("format", "").equals("mgrs")
                ? site.getJsonObject("coordinateInput").getString("value", "")
                : "",
            18);
    mgrs.setName("mission-mgrs");
    f.addRow("MGRS", mgrs);
    f.addRow(
        "",
        button(
            "Apply MGRS",
            "mission-apply-mgrs",
            () -> {
              try {
                commitEditors();
                set(
                    p,
                    MissionCoordinates.reapply(
                        mission.getJsonArray("sites").getJsonObject(i), mgrs.getText()));
                render();
              } catch (Exception e) {
                error(e);
              }
            }));
    f.number(p + "/heightM", "Ground height (m)", 1, true);
    f.choice(
        p + "/heightDatum",
        "Height reference",
        "MSL",
        "Mean sea level",
        "ellipsoidal",
        "WGS84 ellipsoid");
    f.addRow("", button("Place selected site on map", "mission-place-site", map::place));
    f.note("Enter measured or chosen ground heights. Map imagery does not determine elevation.");
    for (int j = 0; j < mission.getJsonArray("pointers").size(); j++) {
      var pointer = mission.getJsonArray("pointers").getJsonObject(j);
      if (!pointer.getString("siteId").equals(selectedSite)) continue;
      String pp = "/pointers/" + j;
      f.section("Antenna pointer");
      f.toggle(pp + "/enabled", "Enable pointer");
      f.choice(
          pp + "/trackingPolicy",
          "Tracking policy",
          "geodetic.v1",
          "Absolute coordinates",
          "legacy-ui.v1",
          "Legacy Telemetry UI");
      f.choice(pp + "/targetVehicleId", "Target vehicle", vehicleChoices());
      f.number(pp + "/mountHeightM", "Antenna above ground (m)");
      f.number(pp + "/initialAzimuthDeg", "Initial azimuth (°)");
      f.number(pp + "/initialElevationDeg", "Initial elevation (°)");
      f.number(pp + "/azimuthRateDegS", "Azimuth slew rate (°/s)");
      f.number(pp + "/elevationRateDegS", "Elevation slew rate (°/s)");
    }
    f.finish();
    var split = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, map, scroll(f));
    split.setResizeWeight(.65);
    split.setDividerLocation(740);
    page.add(split, BorderLayout.CENTER);
    map.update(mission.getJsonArray("sites"), selectedSite);
    return page;
  }

  private static String siteName(String id) {
    return id.equals("launch-site")
        ? "Launch site"
        : id.equals("launch") ? "Base station" : "Away " + id.substring(2);
  }

  private int siteIndex(String id) {
    for (int i = 0; i < mission.getJsonArray("sites").size(); i++)
      if (mission.getJsonArray("sites").getJsonObject(i).getString("id").equals(id)) return i;
    throw new IllegalArgumentException("Unknown site");
  }

  private String[] vehicleChoices() {
    List<String> values = new ArrayList<>();
    mission
        .getJsonArray("vehicleBindings")
        .forEach(
            v -> {
              values.add(v.asJsonObject().getString("id"));
              values.add(v.asJsonObject().getString("id"));
            });
    return values.toArray(String[]::new);
  }

  private void moveSite(String id, double[] ll) {
    try {
      commitEditors();
      if (mission.containsKey("recordedFlight") && id.equals("launch-site"))
        throw new IllegalArgumentException("The recorded launch location is fixed");
      String p = "/sites/" + siteIndex(id);
      set(p + "/latitudeDeg", Json.createValue(ll[0]));
      set(p + "/longitudeDeg", Json.createValue(ll[1]));
      set(
          p + "/coordinateInput",
          Json.createObjectBuilder()
              .add("format", "latlon")
              .add("value", ll[0] + "," + ll[1])
              .add("reference", "point")
              .add("source", "map")
              .build());
      selectedSite = id;
      render();
    } catch (Exception e) {
      error(e);
      map.update(mission.getJsonArray("sites"), selectedSite);
    }
  }

  private JPanel flightPage() {
    var f = form();
    f.section("Rocket and flight configuration");
    f.note(
        "Use the current design or load a rocket. Mission edits use a frozen copy of its flight"
            + " configuration.");
    f.addRow(
        "",
        row(
            button("Use current rocket", "mission-current-rocket", () -> chooseRocket(true)),
            button("Choose .ork…", "mission-choose-rocket", () -> chooseRocket(false))));
    simulations.setName("mission-simulation-choice");
    f.addRow("Saved simulation", simulations);
    f.addRow(
        "",
        button(
            "Use selected simulation",
            "mission-select-flight",
            () -> {
              var selected = (MissionForm.Choice) simulations.getSelectedItem();
              if (selected != null)
                draftAction(
                    "flight",
                    Json.createObjectBuilder()
                        .add("simulation", Integer.parseInt(selected.value()))
                        .build(),
                    this::use);
            }));
    JsonObject flight = mission.getJsonObject("flight");
    f.note(
        "Rocket: "
            + flight.getString("filename", "Not selected")
            + "\nFlight: "
            + flight.getString("simulationName", "Not selected"));
    if (flight.containsKey("editor")) {
      f.section("Vehicle / stage assignments");
      var stages = flight.getJsonObject("editor").getJsonArray("stages");
      List<String> choices = new ArrayList<>(List.of("", "Choose stage"));
      for (var s : stages) {
        choices.add(s.asJsonObject().getString("componentId"));
        choices.add(s.asJsonObject().getString("name"));
      }
      for (int i = 0; i < mission.getJsonArray("vehicleBindings").size(); i++)
        f.choice(
            "/vehicleBindings/" + i + "/componentId",
            mission.getJsonArray("vehicleBindings").getJsonObject(i).getString("id"),
            choices.toArray(String[]::new));
      if (mission.containsKey("recordedFlight"))
        f.note(
            "This experiment uses a recorded trajectory. Choose a rocket to return to a new"
                + " flight.");
    } else
      f.note(
          "Choose a rocket to configure launch, atmosphere, motors, recovery and the flight"
              + " computer.");
    f.finish();
    if (!flight.containsKey("editor")) return page(scroll(f));
    JPanel panel = new JPanel(new BorderLayout(8, 8));
    JScrollPane selection = scroll(f);
    selection.setPreferredSize(new Dimension(900, 245));
    panel.add(selection, BorderLayout.NORTH);
    var details =
        new MissionFlightForms(
                this::form,
                () -> mission,
                this::set,
                this::chooseMotor,
                () -> draftAction("fc", JsonValue.EMPTY_JSON_OBJECT, this::use),
                this)
            .create();
    if (mission.containsKey("recordedFlight")) disableInputs(details);
    panel.add(details, BorderLayout.CENTER);
    return page(panel);
  }

  private void chooseRocket(boolean current) {
    draftAction(
        "rocket",
        Json.createObjectBuilder().add("current", current).build(),
        result -> {
          if (result.containsKey("cancelled")) return;
          simulations.removeAllItems();
          for (var v : result.getJsonArray("simulations")) {
            var s = v.asJsonObject();
            simulations.addItem(
                new MissionForm.Choice(Integer.toString(s.getInt("index")), s.getString("name")));
          }
          status.setText("Choose the saved simulation, then Use selected simulation");
        });
  }

  private void chooseMotor(String path) {
    var search = new JTextField(24);
    var choices = new JComboBox<MissionForm.Choice>();
    var panel = new JPanel(new BorderLayout(8, 8));
    var top = row(new JLabel("Search motor"), search);
    var find = new JButton("Search");
    top.add(find);
    panel.add(top, BorderLayout.NORTH);
    panel.add(choices, BorderLayout.CENTER);
    find.addActionListener(
        e -> {
          find.setEnabled(false);
          new SwingWorker<JsonObject, Void>() {
            protected JsonObject doInBackground() throws Exception {
              return service.request(
                  "POST",
                  "motors",
                  Json.createObjectBuilder().add("query", search.getText()).build());
            }

            protected void done() {
              find.setEnabled(true);
              try {
                choices.removeAllItems();
                choices.addItem(new MissionForm.Choice("", "No motor"));
                for (var v : get().getJsonArray("motors")) {
                  var m = v.asJsonObject();
                  choices.addItem(
                      new MissionForm.Choice(m.getString("digest"), m.getString("name")));
                }
              } catch (Exception error) {
                MissionWindow.this.error(error);
              }
            }
          }.execute();
        });
    find.doClick();
    if (JOptionPane.showConfirmDialog(this, panel, "Motor selection", JOptionPane.OK_CANCEL_OPTION)
            == JOptionPane.OK_OPTION
        && choices.getSelectedItem() instanceof MissionForm.Choice selected) {
      try {
        commitEditors();
        set(path + "/motorDigest", Json.createValue(selected.value()));
        set(path + "/motorName", Json.createValue(selected.label()));
        render();
      } catch (Exception e) {
        error(e);
      }
    }
  }

  private static void disableInputs(Container c) {
    for (var child : c.getComponents()) {
      if (child instanceof JTextField
          || child instanceof JComboBox<?>
          || child instanceof JCheckBox
          || child instanceof JButton) child.setEnabled(false);
      if (child instanceof Container container) disableInputs(container);
    }
  }

  private JPanel telemetryPage() {
    var f = form();
    f.section("Flight computer and packets");
    f.choice(
        "/telemetry/mode",
        "Telemetry source",
        "standard",
        "Standard legacy Zephyrus",
        "custom-fc",
        "Custom flight computer");
    f.choice(
        "/telemetry/fallback",
        "When FC has no format",
        "legacy",
        "Legacy Zephyrus fallback",
        "require-format",
        "Require FC format definition");
    f.number("/telemetry/rateHz", "Standard transmit rate (Hz)");
    f.section("Standard source measurements");
    f.number("/telemetry/gpsRateHz", "GPS acquisition (Hz)");
    f.number("/telemetry/barometerRateHz", "Barometer acquisition (Hz)");
    f.number("/telemetry/noise/gpsHorizontalSigmaM", "GPS horizontal noise (m)");
    f.number("/telemetry/noise/gpsVerticalSigmaM", "GPS vertical noise (m)");
    f.number("/telemetry/noise/barometerSigmaM", "Barometer noise (m)");
    var links = mission.getJsonObject("telemetry").getJsonArray("links");
    var receiver = new JComboBox<MissionForm.Choice>();
    receiver.setName("mission-link-station");
    for (var v : links) {
      String site = v.asJsonObject().getString("receiverId");
      var choice = new MissionForm.Choice(site, siteName(site));
      receiver.addItem(choice);
      if (site.equals(selectedLink)) receiver.setSelectedItem(choice);
    }
    receiver.addActionListener(
        e -> {
          if (!rendering)
            try {
              commitEditors();
              selectedLink = ((MissionForm.Choice) receiver.getSelectedItem()).value();
              render();
            } catch (Exception ex) {
              error(ex);
            }
        });
    f.section("Receiver link");
    f.addRow("Station", receiver);
    for (int i = 0; i < links.size(); i++)
      if (links.getJsonObject(i).getString("receiverId").equals(selectedLink)) {
        String p = "/telemetry/links/" + i;
        f.number(p + "/packetLossFraction", "Packet loss (%)", 100, false);
        f.number(p + "/bitErrorProbability", "Bit error rate", 1, false);
        f.number(p + "/delayMs", "Latency (ms)");
        f.number(p + "/jitterMs", "Jitter (ms)");
        f.number(p + "/uartBaud", "Serial baud (0 = software)");
        f.toggle(p + "/directional", "Qualify directional reception");
        f.number(p + "/beamWidthDeg", "Illustrative antenna beam width (°)");
        f.addRow("Outages", outages(p + "/outages"));
      }
    f.note(
        "Loss and corruption are applied before the ground receiver decodes each packet. Pointer"
            + " tracking uses received telemetry.");
    f.finish();
    return page(scroll(f));
  }

  private JButton outages(String path) {
    return outages(path, () -> MissionForm.value(mission, path), value -> set(path, value));
  }

  private JButton outages(
      String name, java.util.function.Supplier<JsonValue> source, Consumer<JsonArray> write) {
    return button(
        "Edit outage intervals…",
        name,
        () -> {
          try {
            commitEditors();
            var array = source.get();
            var model = new DefaultTableModel(new String[] {"Start (s)", "End (s)"}, 0);
            if (array instanceof JsonArray a)
              for (var v : a) {
                var o = v.asJsonObject();
                model.addRow(
                    new Object[] {
                      Long.parseLong(o.getString("startUs")) / 1e6,
                      Long.parseLong(o.getString("endUs")) / 1e6
                    });
              }
            var table = new JTable(model);
            JPanel panel = new JPanel(new BorderLayout());
            panel.add(new JScrollPane(table));
            panel.add(
                row(
                    button("Add", "outage-add", () -> model.addRow(new Object[] {0, 1})),
                    button(
                        "Remove",
                        "outage-remove",
                        () -> {
                          if (table.getSelectedRow() >= 0) model.removeRow(table.getSelectedRow());
                        })),
                BorderLayout.SOUTH);
            if (JOptionPane.showConfirmDialog(
                    this, panel, "Outage intervals", JOptionPane.OK_CANCEL_OPTION)
                == JOptionPane.OK_OPTION) {
              if (table.isEditing()) table.getCellEditor().stopCellEditing();
              var next = Json.createArrayBuilder();
              for (int i = 0; i < model.getRowCount(); i++) {
                double start = Double.parseDouble(model.getValueAt(i, 0).toString()),
                    end = Double.parseDouble(model.getValueAt(i, 1).toString());
                if (!Double.isFinite(start) || !Double.isFinite(end) || end <= start)
                  throw new IllegalArgumentException("Outage end must follow its start");
                next.add(
                    Json.createObjectBuilder()
                        .add("id", "outage-" + i)
                        .add("startUs", Long.toString(Math.round(start * 1e6)))
                        .add("endUs", Long.toString(Math.round(end * 1e6))));
              }
              write.accept(next.build());
            }
          } catch (Exception e) {
            error(e);
          }
        });
  }

  private JPanel videoPage() {
    var f = form();
    f.section("Identification streams");
    var preset = new JComboBox<>(new String[] {"Balius", "Iris", "Custom"});
    preset.setName("mission-video-preset");
    preset.setSelectedItem(mission.getJsonObject("video").getString("preset", "Balius"));
    preset.addActionListener(
        e -> {
          if (!rendering)
            try {
              commitEditors();
              applyPreset((String) preset.getSelectedItem());
              render();
            } catch (Exception ex) {
              error(ex);
            }
        });
    f.addRow("Vehicle layout", preset);
    for (String kind : List.of("digital", "analog")) {
      int n =
          (int)
              mission.getJsonObject("video").getJsonArray("streams").stream()
                  .filter(v -> v.asJsonObject().getString("kind").equals(kind))
                  .count();
      var count = new JSpinner(new SpinnerNumberModel(n, 0, 32, 1));
      count.setName("mission-" + kind + "-count");
      count.addChangeListener(
          e -> {
            if (!rendering)
              try {
                commitEditors();
                resizeStreams(kind, (Integer) count.getValue());
                render();
              } catch (Exception ex) {
                error(ex);
              }
          });
      f.addRow(kind.equals("digital") ? "Digital streams" : "Analog streams", count);
    }
    f.number("/video/frameRateHz", "Frame rate (Hz)");
    f.number("/video/lossFraction", "Frame loss (%)", 100, false);
    f.number("/video/delayMs", "Video delay (ms)");
    f.number("/video/analogNoiseFraction", "Analog noise (%)", 100, false);
    f.addRow("Outages", outages("/video/outages"));
    f.addRow(
        "Per-station conditions",
        button("Edit receiver overrides…", "mission-video-overrides", this::videoOverrides));
    f.toggle("/outputs/recordEncodedVideo", "Also include encoded video in the result bundle");
    f.note(
        "Network video supports two main and two preview sessions per station at once. Route at"
            + " most two simultaneously displayed streams to each station.");
    f.note(
        "Each stream is an identification slate showing its type, mission, run and rocket. Network"
            + " sharing uses the normal Away video protocol.");
    int i = 0;
    for (var v : mission.getJsonObject("video").getJsonArray("streams")) {
      var s = v.asJsonObject();
      String p = "/video/streams/" + i++;
      f.section(s.getString("id"));
      f.choice(p + "/vehicleId", "Vehicle", vehicleChoices());
      JPanel routes = row();
      List<JCheckBox> checks = new ArrayList<>();
      for (int n = 1; n <= 4; n++) {
        String id = "gs" + n;
        var c =
            new JCheckBox("Away " + n, s.getJsonArray("receivers").contains(Json.createValue(id)));
        c.addActionListener(
            e -> {
              var a = Json.createArrayBuilder();
              for (int q = 0; q < checks.size(); q++)
                if (checks.get(q).isSelected()) a.add("gs" + (q + 1));
              set(p + "/receivers", a.build());
            });
        checks.add(c);
        routes.add(c);
      }
      f.addRow("Receivers", routes);
    }
    f.finish();
    return page(scroll(f));
  }

  private void videoOverrides() {
    try {
      commitEditors();
      var profiles = mission.getJsonObject("video").getJsonArray("receiverProfiles");
      JsonObject[] draft = {
        Json.createObjectBuilder()
            .add("profiles", profiles == null ? JsonValue.EMPTY_JSON_ARRAY : profiles)
            .build()
      };
      var full = Json.createArrayBuilder();
      for (int n = 1; n <= 4; n++) {
        String id = "gs" + n;
        JsonObject saved = null;
        for (var v : draft[0].getJsonArray("profiles"))
          if (v.asJsonObject().getString("receiverId").equals(id)) saved = v.asJsonObject();
        full.add(
            saved == null
                ? Json.createObjectBuilder()
                    .add("receiverId", id)
                    .add("enabled", false)
                    .add("lossFraction", 0)
                    .add("delayMs", 0)
                    .add("analogNoiseFraction", 0)
                    .add("outages", JsonValue.EMPTY_JSON_ARRAY)
                    .build()
                : saved);
      }
      draft[0] = Json.createObjectBuilder().add("profiles", full).build();
      var editor =
          new MissionForm(
              () -> draft[0],
              (p, v) -> draft[0] = Json.createPointer(p).add(draft[0], v),
              () -> {});
      for (int n = 0; n < 4; n++) {
        String p = "/profiles/" + n;
        editor.section("Away " + (n + 1));
        editor.toggle(p + "/enabled", "Override common video conditions");
        editor.number(p + "/lossFraction", "Frame loss (%)", 100, false);
        editor.number(p + "/delayMs", "Delay (ms)");
        editor.number(p + "/analogNoiseFraction", "Analog noise (%)", 100, false);
        editor.addRow(
            "Outages",
            outages(
                p + "/outages",
                () -> MissionForm.value(draft[0], p + "/outages"),
                value -> draft[0] = Json.createPointer(p + "/outages").add(draft[0], value)));
      }
      editor.finish();
      var scroller = scroll(editor);
      scroller.setPreferredSize(new Dimension(600, 540));
      if (JOptionPane.showConfirmDialog(
              this, scroller, "Per-station video", JOptionPane.OK_CANCEL_OPTION)
          == JOptionPane.OK_OPTION) {
        editor.commit();
        set("/video/receiverProfiles", draft[0].getJsonArray("profiles"));
      }
    } catch (Exception e) {
      error(e);
    }
  }

  private void resizeStreams(String kind, int count) {
    var streams = mission.getJsonObject("video").getJsonArray("streams");
    var same = new ArrayList<JsonObject>();
    var next = Json.createArrayBuilder();
    streams.forEach(
        v -> {
          if (v.asJsonObject().getString("kind").equals(kind)) same.add(v.asJsonObject());
          else next.add(v);
        });
    for (int i = 0; i < count; i++)
      next.add(
          i < same.size()
              ? same.get(i)
              : Json.createObjectBuilder()
                  .add("id", kind + "-" + (i + 1))
                  .add("kind", kind)
                  .add(
                      "vehicleId",
                      mission.getJsonArray("vehicleBindings").getJsonObject(0).getString("id"))
                  .add(
                      "receivers",
                      Json.createArrayBuilder().add("gs1").add("gs2").add("gs3").add("gs4"))
                  .build());
    set("/video/streams", next.build());
    set("/video/preset", Json.createValue("Custom"));
  }

  private void applyPreset(String preset) {
    set("/video/preset", Json.createValue(preset));
    if (preset.equals("Custom")) return;
    boolean iris = preset.equals("Iris");
    JsonArray stages =
        mission.getJsonObject("flight").containsKey("editor")
            ? mission.getJsonObject("flight").getJsonObject("editor").getJsonArray("stages")
            : JsonValue.EMPTY_JSON_ARRAY;
    var vehicles = Json.createArrayBuilder();
    String first = iris ? "sustainer" : "balius";
    vehicles.add(
        Json.createObjectBuilder()
            .add("id", first)
            .add(
                "componentId",
                stages.isEmpty() ? "" : stages.getJsonObject(0).getString("componentId")));
    if (iris)
      vehicles.add(
          Json.createObjectBuilder()
              .add("id", "booster")
              .add(
                  "componentId",
                  stages.size() > 1 ? stages.getJsonObject(1).getString("componentId") : ""));
    set("/vehicleBindings", vehicles.build());
    var streams = Json.createArrayBuilder();
    streams.add(stream(first + ".digital", first, "digital", 1, 4));
    streams.add(stream(first + ".analog", first, "analog", 1, iris ? 3 : 4));
    if (iris) streams.add(stream("booster.analog", "booster", "analog", 4, 4));
    set("/video/streams", streams.build());
    for (String group : List.of("pointers", "stationBindings")) {
      var a = Json.createArrayBuilder();
      for (var v : mission.getJsonArray(group)) {
        var o = v.asJsonObject();
        a.add(
            Json.createObjectBuilder(o)
                .add(
                    "targetVehicleId",
                    iris && o.getString("siteId").equals("gs4") ? "booster" : first));
      }
      set("/" + group, a.build());
    }
    set("/network/vehicleMode", Json.createValue(iris ? "iris" : "balius"));
  }

  private static JsonObject stream(String id, String vehicle, String kind, int first, int last) {
    var a = Json.createArrayBuilder();
    for (int i = first; i <= last; i++) a.add("gs" + i);
    return Json.createObjectBuilder()
        .add("id", id)
        .add("vehicleId", vehicle)
        .add("kind", kind)
        .add("receivers", a)
        .build();
  }

  private JPanel networkPage() {
    var f = form();
    f.section("Share with Telemetry UI on the local network");
    if (mission.containsKey("legacyStationBindings"))
      f.note(
          "This imported setup now uses OpenRocket station controls and network discovery. Its"
              + " earlier application bindings are retained in the configuration for reference.");
    f.toggle("/network/enabled", "Advertise simulated Away stations");
    f.toggle("/network/video", "Publish video feeds");
    f.choice("/network/vehicleMode", "Base vehicle layout", "balius", "Balius", "iris", "Iris");
    f.choice(
        "/network/bindHost",
        "Network interface",
        "0.0.0.0",
        "Local network (all interfaces)",
        "127.0.0.1",
        "This computer only");
    f.note(
        "Open Telemetry UI normally in Base mode. In Station network, select a SIMULATION Away"
            + " station and join its mission. Telemetry and video use the same network connections"
            + " as an ordinary Away station. No application path is needed.");
    var uuid = new JTextField(id());
    uuid.setEditable(false);
    f.addRow("Mission UUID", uuid);
    f.addRow(
        "",
        row(
            button(
                "Advertise now",
                "mission-network-start",
                () ->
                    perform(
                        "Starting station services",
                        () -> {
                          saveDraft();
                          return service.request(
                              "POST",
                              "network/start",
                              Json.createObjectBuilder().add("draftId", id()).build());
                        },
                        this::showNetwork)),
            button(
                "Stop sharing",
                "mission-network-stop",
                () ->
                    perform(
                        "Stopping station services",
                        () -> service.request("POST", "network/stop", JsonValue.EMPTY_JSON_OBJECT),
                        this::showNetwork))));
    f.note(
        "Network rehearsals run in real time. Other ensemble members remain headless; only the"
            + " selected member is advertised. Pointer and flight-computer commands remain in this"
            + " OpenRocket window.");
    f.finish();
    JPanel page = page(scroll(f));
    page.add(scroll(networkDetails), BorderLayout.SOUTH);
    return page;
  }

  private void showNetwork(JsonObject value) {
    StringBuilder text = new StringBuilder("Sharing: " + value.getString("state", "off") + "\n\n");
    for (var entry : value.getJsonArray("services")) {
      var station = entry.asJsonObject();
      text.append(siteName(station.getString("station")))
          .append(" · ")
          .append(station.getString("role"))
          .append(" · ")
          .append(station.getString("state"))
          .append("\n");
      var addresses = station.getJsonArray("addresses");
      if (addresses != null && !addresses.isEmpty())
        for (var address : addresses)
          text.append("  ").append(((JsonString) address).getString()).append("\n");
      else text.append("  ").append(station.getString("endpoint", "")).append("\n");
    }
    networkDetails.setText(text.toString());
    networkDetails.setCaretPosition(0);
    status.setText("Network service updated");
  }

  private JPanel runPage() {
    var f = form();
    f.section("Execution");
    f.choice(
        "/execution/mode",
        "Mission type",
        "ground-link",
        "Standard flight / ground link analysis",
        "coupled",
        "Coupled custom flight computer",
        "interactive",
        "Real-time rehearsal");
    f.choice(
        "/execution/clockMode", "Clock", "paced", "Real time", "lockstep", "As fast as possible");
    f.text("/execution/seed", "Mission random seed");
    f.text("/outputs/filename", "Result filename (.rktmission)");
    f.number("/execution/applicationMember", "Member shown on network (zero-based)");
    f.note(
        "Configure the number of flights and their uncertainties under Rocket & flight →"
            + " Ensemble.");
    f.addRow(
        "Scenario", button("Timed station actions…", "mission-timed-actions", this::editActions));
    f.finish();
    JPanel execution = new JPanel(new BorderLayout());
    execution.add(scroll(f));
    execution.add(
        row(
            button("Validate", "mission-validate", this::validateMission),
            button("Start mission", "mission-start", this::start),
            button("Pause", "mission-pause", () -> jobAction("pause")),
            button("Resume", "mission-resume", () -> jobAction("resume")),
            button("Cancel run", "mission-cancel", () -> jobAction("cancel")),
            button("Pointer / FC command…", "mission-command", () -> command(false))),
        BorderLayout.SOUTH);
    JPanel live = new JPanel(new BorderLayout(4, 4));
    live.add(runStatus, BorderLayout.NORTH);
    live.add(progress, BorderLayout.CENTER);
    live.add(scroll(runDetails), BorderLayout.SOUTH);
    JPanel lower = new JPanel(new BorderLayout(8, 8));
    lower.add(scroll(findings), BorderLayout.NORTH);
    lower.add(live, BorderLayout.CENTER);
    lower.add(resultsPanel, BorderLayout.SOUTH);
    runPanel.removeAll();
    runPanel.add(new JSplitPane(JSplitPane.VERTICAL_SPLIT, execution, lower), BorderLayout.CENTER);
    if (opened == null) {
      resultsPanel.removeAll();
      resultsPanel.add(
          row(
              button(
                  "Open this run’s result",
                  "mission-result-current",
                  () -> {
                    if (job != null)
                      perform(
                          "Opening results",
                          () ->
                              service.request(
                                  "POST",
                                  "jobs/" + job.getString("id") + "/open-result",
                                  JsonValue.EMPTY_JSON_OBJECT),
                          v -> {
                            opened = v;
                            renderResults();
                          });
                  }),
              button("Open result…", "mission-result-open", () -> openResult("open")),
              button(
                  "Recover interrupted run…",
                  "mission-result-recover",
                  () -> openResult("recover"))));
    }
    return runPanel;
  }

  private static String human(String key) {
    return key.replaceAll("([a-z])([A-Z])", "$1 $2").replace("Sigma", "uncertainty");
  }

  private JsonObject validateInputs() throws Exception {
    saveDraft();
    var issues =
        service.request("POST", "drafts/" + id() + "/validate", JsonValue.EMPTY_JSON_OBJECT);
    if (!issues.getBoolean("configurationValid")) return issues;
    var r =
        service.request(
            "POST",
            "drafts/" + id() + "/resolve",
            Json.createObjectBuilder().add("expectedRevision", revision()).build());
    resolvedHash = r.getString("hash");
    return issues;
  }

  private void showValidation(JsonObject result) {
    StringBuilder text = new StringBuilder();
    for (var v : result.getJsonArray("issues"))
      text.append(v.asJsonObject().getString("message")).append('\n');
    findings.setText(text.isEmpty() ? "Configuration valid. Ready to run." : text.toString());
    status.setText(
        text.isEmpty() ? "Configuration valid" : "Resolve the listed configuration inputs");
  }

  private void validateMission() {
    perform("Validating mission", this::validateInputs, this::showValidation);
  }

  private void start() {
    try {
      commitEditors();
      if (mission.getJsonObject("network").getBoolean("enabled")) {
        set("/execution/clockMode", Json.createValue("paced"));
        render();
      }
    } catch (Exception e) {
      error(e);
      return;
    }
    perform(
        "Preparing mission",
        () -> {
          JsonObject validation = validateInputs();
          if (!validation.getBoolean("configurationValid")) return validation;
          return service.request(
              "POST",
              "jobs",
              Json.createObjectBuilder()
                  .add("draftId", id())
                  .add("expectedRevision", revision())
                  .add("resolvedHash", resolvedHash)
                  .add("idempotencyKey", UUID.randomUUID().toString())
                  .build());
        },
        result -> {
          if (result.containsKey("issues")) {
            showValidation(result);
            return;
          }
          job = result;
          updateJob();
          render();
          tabs.setSelectedIndex(5);
        });
  }

  private void jobAction(String action) {
    if (job == null) return;
    perform(
        "Updating run",
        () ->
            service.request(
                "POST", "jobs/" + job.getString("id") + "/" + action, JsonValue.EMPTY_JSON_OBJECT),
        v -> {
          job = v;
          updateJob();
        });
  }

  private void poll() {
    if (job == null || busy || polling || !isVisible()) return;
    polling = true;
    String jobId = job.getString("id");
    new SwingWorker<JsonObject, Void>() {
      protected JsonObject doInBackground() throws Exception {
        return service.request("GET", "jobs/" + jobId, JsonValue.EMPTY_JSON_OBJECT);
      }

      protected void done() {
        polling = false;
        try {
          var next = get();
          if (job != null && job.getString("id").equals(jobId)) {
            job = next;
            updateJob();
          }
        } catch (Exception e) {
          status.setText("Run status unavailable: " + e.getMessage());
        }
      }
    }.execute();
  }

  private void updateJob() {
    String state = job.getString("status");
    runStatus.setText(job.getString("phase") + " · " + state);
    long time = Long.parseLong(job.getString("timeUs")),
        end = Long.parseLong(job.getString("endUs"));
    progress.setIndeterminate(end <= 0 && !state.equals("complete") && !state.equals("failed"));
    progress.setValue(end <= 0 ? 0 : (int) Math.max(0, Math.min(100, 100. * time / end)));
    progress.setStringPainted(true);
    progress.setString(String.format(Locale.ROOT, "%.2f / %.2f s", time / 1e6, end / 1e6));
    runDetails.setText(
        (job.getString("failure", "").isEmpty() ? "" : job.getString("failure") + "\n")
            + "Result: "
            + job.getString("result", "")
            + "\n"
            + metrics(job.getJsonObject("metrics")));
    runDetails.setCaretPosition(0);
    status.setText(job.getString("phase"));
  }

  private String metrics(JsonObject value) {
    if (value == null || !value.containsKey("receivers")) return "";
    StringBuilder b = new StringBuilder("Station    Accepted packets    Pointer commands\n");
    for (String id : value.getJsonObject("receivers").keySet()) {
      var r = value.getJsonObject("receivers").getJsonObject(id);
      b.append(siteName(id))
          .append("    ")
          .append(r.getInt("acceptedFrames", 0))
          .append("    ")
          .append(r.getInt("pointerCommands", 0))
          .append('\n');
    }
    return b.toString();
  }

  private void editActions() {
    try {
      commitEditors();
      JList<String> list =
          new JList<>(
              mission.getJsonObject("execution").getJsonArray("actions").stream()
                  .map(
                      v ->
                          v.asJsonObject().getString("timeUs")
                              + " µs · "
                              + v.asJsonObject().getString("siteId")
                              + " · "
                              + v.asJsonObject().getString("kind"))
                  .toArray(String[]::new));
      JPanel panel = page(new JScrollPane(list));
      panel.add(
          row(
              button("Add action", "mission-action-add", () -> command(true)),
              button(
                  "Remove selected",
                  "mission-action-remove",
                  () -> {
                    int index = list.getSelectedIndex();
                    if (index >= 0) {
                      var array =
                          Json.createArrayBuilder(
                              mission.getJsonObject("execution").getJsonArray("actions"));
                      array.remove(index);
                      set("/execution/actions", array.build());
                      list.setListData(
                          mission.getJsonObject("execution").getJsonArray("actions").stream()
                              .map(Object::toString)
                              .toArray(String[]::new));
                    }
                  })),
          BorderLayout.SOUTH);
      JOptionPane.showMessageDialog(
          this, panel, "Timed station actions", JOptionPane.PLAIN_MESSAGE);
    } catch (Exception e) {
      error(e);
    }
  }

  private void command(boolean timed) {
    var fields = new JPanel(new GridLayout(0, 2, 8, 6));
    var kind = new JComboBox<>(MissionActions.KINDS.stream().sorted().toArray(String[]::new));
    var site = new JComboBox<>(new String[] {"gs1", "gs2", "gs3", "gs4", "launch"});
    JTextField time = new JTextField("0"),
        az = new JTextField("0"),
        el = new JTextField("0"),
        target =
            new JTextField(
                mission.getJsonArray("vehicleBindings").getJsonObject(0).getString("id")),
        opcode = new JTextField("1"),
        argument = new JTextField("0");
    for (var item :
        new Object[][] {
          {"Action", kind},
          {"Station", site},
          {"Time after launch (s)", time},
          {"Azimuth / manual angle (°)", az},
          {"Elevation (°)", el},
          {"Target vehicle", target},
          {"FC opcode (decimal)", opcode},
          {"FC argument", argument}
        }) {
      fields.add(new JLabel((String) item[0]));
      fields.add((Component) item[1]);
    }
    time.setEnabled(timed);
    if (JOptionPane.showConfirmDialog(
            this,
            fields,
            timed ? "Add timed action" : "Simulated pointer / FC command",
            JOptionPane.OK_CANCEL_OPTION)
        != JOptionPane.OK_OPTION) return;
    try {
      String k = (String) kind.getSelectedItem();
      var b =
          Json.createObjectBuilder()
              .add("id", UUID.randomUUID().toString())
              .add("kind", k)
              .add("siteId", (String) site.getSelectedItem());
      if (timed)
        b.add("timeUs", Long.toString(Math.round(Double.parseDouble(time.getText()) * 1e6)));
      if (k.equals("point"))
        b.add("azimuthDeg", Double.parseDouble(az.getText()))
            .add("elevationDeg", Double.parseDouble(el.getText()));
      if (k.equals("set-target")) b.add("targetVehicleId", target.getText());
      if (k.equals("uplink"))
        b.add("opcode", Integer.parseInt(opcode.getText()))
            .add("value", Integer.parseInt(argument.getText()))
            .add("angleDeg", Double.parseDouble(az.getText()));
      var action = b.build();
      MissionActions.validate(action);
      if (timed)
        set(
            "/execution/actions",
            Json.createArrayBuilder(mission.getJsonObject("execution").getJsonArray("actions"))
                .add(action)
                .build());
      else if (job != null)
        perform(
            "Sending command",
            () -> service.request("POST", "jobs/" + job.getString("id") + "/action", action),
            v -> status.setText("Command queued"));
    } catch (Exception e) {
      error(e);
    }
  }

  private void openResult(String operation) {
    perform(
        "Opening results",
        () -> service.request("POST", "results/" + operation, JsonValue.EMPTY_JSON_OBJECT),
        v -> {
          if (v.containsKey("cancelled")) return;
          opened = v;
          renderResults();
        });
  }

  private void renderResults() {
    resultsPanel.removeAll();
    JPanel panel = new JPanel(new BorderLayout(6, 6));
    var member =
        new JComboBox<>(
            opened.getJsonArray("members").stream()
                .map(v -> ((JsonString) v).getString())
                .toArray(String[]::new));
    var stream = new JComboBox<String>();
    var station = new JComboBox<>(new String[] {"launch", "gs1", "gs2", "gs3", "gs4"});
    Runnable channels =
        () -> {
          stream.removeAllItems();
          for (var v : opened.getJsonArray("streams")) {
            String name = v.asJsonObject().getString("stream");
            if (name.startsWith("runs/" + member.getSelectedItem() + "/")) stream.addItem(name);
          }
        };
    channels.run();
    member.addActionListener(
        e ->
            perform(
                "Reading member",
                () ->
                    service.request(
                        "POST",
                        "results/" + opened.getString("id") + "/query",
                        Json.createObjectBuilder()
                            .add("member", member.getSelectedItem().toString())
                            .build()),
                v -> {
                  opened =
                      Json.createObjectBuilder(opened)
                          .add("streams", v.getJsonArray("streams"))
                          .build();
                  channels.run();
                }));
    var start = new JTextField("-1", 5);
    var width = new JTextField("10", 5);
    var model =
        new DefaultTableModel(new String[] {"Time (s)", "Record"}, 0) {
          public boolean isCellEditable(int r, int c) {
            return false;
          }
        };
    var table = new JTable(model);
    table.setName("mission-result-records");
    var details = new JTextArea();
    details.setEditable(false);
    final List<JsonObject> records = new ArrayList<>();
    table
        .getSelectionModel()
        .addListSelectionListener(
            e -> {
              int i = table.getSelectedRow();
              if (i >= 0 && i < records.size()) details.setText(records.get(i).toString());
            });
    JButton read =
        button(
            "Read time window",
            "mission-result-read",
            () ->
                perform(
                    "Reading records",
                    () ->
                        service.request(
                            "POST",
                            "results/" + opened.getString("id") + "/query",
                            Json.createObjectBuilder()
                                .add("member", member.getSelectedItem().toString())
                                .add("stream", stream.getSelectedItem().toString())
                                .add(
                                    "startUs",
                                    Long.toString(
                                        Math.round(Double.parseDouble(start.getText()) * 1e6)))
                                .add(
                                    "endUs",
                                    Long.toString(
                                        Math.round(
                                            (Double.parseDouble(start.getText())
                                                    + Double.parseDouble(width.getText()))
                                                * 1e6)))
                                .add("limit", 250)
                                .build()),
                    v -> {
                      model.setRowCount(0);
                      records.clear();
                      for (var entry : v.getJsonArray("records")) {
                        var r = entry.asJsonObject();
                        records.add(r);
                        model.addRow(
                            new Object[] {
                              Long.parseLong(r.getString("timeUs")) / 1e6,
                              r.getString(
                                  "kind",
                                  r.containsKey("altitude")
                                      ? "Telemetry · height " + r.get("altitude")
                                      : "Sample")
                            });
                      }
                      status.setText(records.size() + " records loaded");
                    }));
    var source = new JComboBox<>(new String[] {"recorded-tx", "standard-generator"});
    panel.add(
        row(
            new JLabel(opened.getString("filename")),
            member,
            station,
            button(
                "Export station…",
                "mission-result-export",
                () ->
                    perform(
                        "Exporting recording",
                        () ->
                            service.request(
                                "POST",
                                "results/" + opened.getString("id") + "/export",
                                Json.createObjectBuilder()
                                    .add("member", member.getSelectedItem().toString())
                                    .add("station", station.getSelectedItem().toString())
                                    .add("session", "")
                                    .build()),
                        v -> status.setText("Exported " + v.getString("saved", "")))),
            source,
            button(
                "New experiment",
                "mission-result-derive",
                () ->
                    perform(
                        "Creating experiment",
                        () ->
                            service.request(
                                "POST",
                                "results/" + opened.getString("id") + "/derive",
                                Json.createObjectBuilder()
                                    .add("member", member.getSelectedItem().toString())
                                    .add("telemetrySource", source.getSelectedItem().toString())
                                    .build()),
                        this::use))),
        BorderLayout.NORTH);
    JPanel recordsPanel = new JPanel(new BorderLayout());
    recordsPanel.add(
        row(stream, new JLabel("Start (s)"), start, new JLabel("Window (s)"), width, read),
        BorderLayout.NORTH);
    var split = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, scroll(table), scroll(details));
    split.setPreferredSize(new Dimension(1000, 220));
    split.setResizeWeight(.5);
    recordsPanel.add(split);
    panel.add(recordsPanel);
    resultsPanel.add(panel);
    resultsPanel.revalidate();
    resultsPanel.repaint();
  }
}
