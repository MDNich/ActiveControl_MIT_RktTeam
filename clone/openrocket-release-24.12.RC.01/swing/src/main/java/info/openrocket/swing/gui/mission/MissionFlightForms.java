package info.openrocket.swing.gui.mission;

import info.openrocket.core.motor.IgnitionEvent;
import info.openrocket.core.rocketcomponent.DeploymentConfiguration;
import info.openrocket.core.rocketcomponent.StageSeparationConfiguration;
import jakarta.json.*;
import java.awt.*;
import java.util.ArrayList;
import java.util.List;
import java.util.function.*;
import javax.swing.*;
import javax.swing.table.DefaultTableModel;

/** Mission-specific native forms for the complete flight snapshot. */
final class MissionFlightForms {
  private final Supplier<MissionForm> forms;
  private final Supplier<JsonObject> mission;
  private final BiConsumer<String, JsonValue> write;
  private final Consumer<String> chooseMotor;
  private final Runnable chooseComputer;
  private final Component parent;

  MissionFlightForms(
      Supplier<MissionForm> forms,
      Supplier<JsonObject> mission,
      BiConsumer<String, JsonValue> write,
      Consumer<String> chooseMotor,
      Runnable chooseComputer,
      Component parent) {
    this.forms = forms;
    this.mission = mission;
    this.write = write;
    this.chooseMotor = chooseMotor;
    this.chooseComputer = chooseComputer;
    this.parent = parent;
  }

  JTabbedPane create() {
    JTabbedPane tabs = new JTabbedPane();
    tabs.setName("mission-flight-tabs");
    tabs.addTab("Launch & atmosphere", scroll(environment()));
    tabs.addTab("Motors & recovery", scroll(components()));
    tabs.addTab("Flight computer", scroll(computer()));
    tabs.addTab("Ensemble", scroll(ensemble()));
    return tabs;
  }

  private static JScrollPane scroll(Component c) {
    var p = new JScrollPane(c);
    p.setBorder(BorderFactory.createEmptyBorder());
    p.getVerticalScrollBar().setUnitIncrement(18);
    return p;
  }

  private static String[] choices(Enum<?>[] values) {
    var result = new ArrayList<String>();
    for (var v : values) {
      result.add(v.name());
      result.add(v.toString());
    }
    return result.toArray(String[]::new);
  }

  private MissionForm environment() {
    var f = forms.get();
    String p = "/flight/editor/options";
    f.section("Launch guide");
    f.number(p + "/guideLengthM", "Length (m)");
    f.number(p + "/guideAngleRad", "Tilt from vertical (°)", 180 / Math.PI, false);
    f.toggle(p + "/intoWind", "Point the guide into the wind");
    f.number(p + "/manualBearingRad", "Manual bearing (°)", 180 / Math.PI, false);
    f.number(p + "/initialRollRateRadS", "Initial roll rate (°/s)", 180 / Math.PI, false);
    f.note("Launch latitude, longitude and altitude come from Site & pointers.");
    f.section("Atmosphere");
    f.toggle(p + "/isa", "Use standard atmosphere");
    f.number(p + "/temperatureK", "Launch temperature (K)");
    f.number(p + "/pressurePa", "Launch pressure (Pa)");
    f.section("Wind");
    f.choice(
        p + "/windModel",
        "Wind model",
        "AVERAGE",
        "Average wind",
        "MULTI_LEVEL",
        "Altitude profile");
    f.number(p + "/averageWind/speedMS", "Average speed (m/s)");
    f.number(p + "/averageWind/directionRad", "Wind bearing (°)", 180 / Math.PI, false);
    f.number(p + "/averageWind/sigmaMS", "Speed standard deviation (m/s)");
    f.choice(
        p + "/windAltitudeReference",
        "Profile altitude reference",
        "MSL",
        "Mean sea level",
        "AGL",
        "Above launch site");
    JButton levels = new JButton("Edit altitude profile…");
    levels.setName("mission-wind-levels");
    levels.addActionListener(e -> windLevels());
    f.addRow("Wind levels", levels);
    f.section("Numerical simulation");
    f.choice(
        p + "/geodeticMethod",
        "Earth model",
        "FLAT",
        "Flat Earth",
        "SPHERICAL",
        "Spherical Earth",
        "WGS84",
        "WGS84 ellipsoid");
    f.number(p + "/timeStepS", "Maximum time step (s)");
    f.number(p + "/maximumStepAngleRad", "Maximum rotation per step (°)", 180 / Math.PI, false);
    f.number(p + "/maximumTimeS", "Maximum flight duration (s)");
    f.number(p + "/randomSeed", "Flight random seed");
    f.finish();
    return f;
  }

  private MissionForm components() {
    var f = forms.get();
    var editor = mission.get().getJsonObject("flight").getJsonObject("editor");
    int index = 0;
    for (var entry : editor.getJsonArray("stages")) {
      var stage = entry.asJsonObject();
      String p = "/flight/editor/stages/" + index++;
      f.section(stage.getString("name"));
      f.toggle(p + "/active", "Include this stage");
      f.choice(
          p + "/event",
          "Separation event",
          choices(StageSeparationConfiguration.SeparationEvent.values()));
      f.number(p + "/altitudeM", "Separation altitude (m)");
      f.number(p + "/delayS", "Separation delay (s)");
    }
    index = 0;
    for (var entry : editor.getJsonArray("motors")) {
      var motor = entry.asJsonObject();
      String p = "/flight/editor/motors/" + index++;
      f.section(motor.getString("name"));
      JButton choose = new JButton(motor.getString("motorName") + " · Choose…");
      choose.setName(p + "/choose");
      choose.addActionListener(e -> chooseMotor.accept(p));
      f.addRow("Motor", choose);
      f.number(p + "/ejectionDelayS", "Ejection delay (s; blank = plugged)", 1, true);
      f.toggle(p + "/ignitionOverride", "Override default ignition");
      f.choice(p + "/event", "Ignition event", choices(IgnitionEvent.values()));
      f.number(p + "/delayS", "Ignition delay (s)");
    }
    index = 0;
    for (var entry : editor.getJsonArray("recovery")) {
      var recovery = entry.asJsonObject();
      String p = "/flight/editor/recovery/" + index++;
      f.section(recovery.getString("name"));
      f.choice(
          p + "/event", "Deployment event", choices(DeploymentConfiguration.DeployEvent.values()));
      f.number(p + "/altitudeM", "Deployment altitude (m)");
      f.number(p + "/delayS", "Deployment delay (s)");
    }
    f.finish();
    return f;
  }

  private MissionForm computer() {
    var f = forms.get();
    f.section("Custom flight computer");
    JButton choose = new JButton("Choose flight computer .fc…");
    choose.setName("mission-choose-fc");
    choose.addActionListener(e -> chooseComputer.run());
    f.addRow("Design", choose);
    f.note(
        "Select Standard or Custom flight computer in Telemetry. A chosen design is copied into the"
            + " mission setup. The telemetry format is taken from the design when available.");
    var extensions =
        mission.get().getJsonObject("flight").getJsonObject("editor").getJsonArray("extensions");
    for (int i = 0; i < extensions.size(); i++) {
      var extension = extensions.getJsonObject(i);
      f.section(extension.getString("name"));
      var config = extension.getJsonObject("config");
      for (String key : config.keySet()) {
        if (List.of("designFile", "designId", "computer").contains(key)) continue;
        String p =
            "/flight/editor/extensions/"
                + i
                + "/config/"
                + key.replace("~", "~0").replace("/", "~1");
        var value = config.get(key);
        String label = key.replaceAll("([a-z])([A-Z])", "$1 $2");
        if (value == JsonValue.TRUE || value == JsonValue.FALSE) f.toggle(p, label);
        else if (value instanceof JsonNumber) f.number(p, label);
        else if (value instanceof JsonString) f.text(p, label);
      }
    }
    f.finish();
    return f;
  }

  private MissionForm ensemble() {
    var f = forms.get();
    String p = "/flight/editor/options/ensemble";
    f.section("Flight ensemble");
    f.choice(p + "/enabled", "Flights", "false", "Single flight", "true", "Ensemble");
    for (String[] item :
        new String[][] {
          {"runs", "Number of members"},
          {"threads", "Worker threads"},
          {"seed", "Sampling seed"},
          {"motorsigma", "Thrust standard deviation (N)"},
          {"noiseinterval", "Thrust noise interval (s)"},
          {"temperaturesigma", "Temperature standard deviation (K)"},
          {"pressuresigma", "Pressure standard deviation (fraction)"},
          {"windsigma", "Wind speed standard deviation (m/s)"},
          {"winddirectionsigma", "Wind direction standard deviation (rad)"},
          {"masssigma", "Mass standard deviation (fraction)"},
          {"dragsigma", "Drag standard deviation (fraction)"},
          {"rollforcesigma", "Roll forcing standard deviation (fraction)"},
          {"rolldampingsigma", "Roll damping standard deviation (fraction)"}
        }) f.text(p + "/" + item[0], item[1]);
    f.note(
        "A fraction of 0.1 means 10%. Network sharing publishes only the member selected in Run &"
            + " results; other members are calculated without network traffic.");
    f.finish();
    return f;
  }

  private void windLevels() {
    try {
      var model =
          new DefaultTableModel(
              new String[] {"Altitude (m)", "Speed (m/s)", "Direction (°)", "Speed σ (m/s)"}, 0);
      for (var entry :
          mission
              .get()
              .getJsonObject("flight")
              .getJsonObject("editor")
              .getJsonObject("options")
              .getJsonArray("windLevels")) {
        var v = entry.asJsonObject();
        model.addRow(
            new Object[] {
              v.get("altitudeM"),
              v.get("speedMS"),
              Math.toDegrees(v.getJsonNumber("directionRad").doubleValue()),
              v.get("sigmaMS")
            });
      }
      var table = new JTable(model);
      var panel = new JPanel(new BorderLayout());
      panel.add(new JScrollPane(table));
      var buttons = new JPanel();
      var add = new JButton("Add level");
      add.addActionListener(e -> model.addRow(new Object[] {0, 0, 0, 0}));
      var remove = new JButton("Remove selected");
      remove.addActionListener(
          e -> {
            if (table.getSelectedRow() >= 0) model.removeRow(table.getSelectedRow());
          });
      buttons.add(add);
      buttons.add(remove);
      panel.add(buttons, BorderLayout.SOUTH);
      if (JOptionPane.showConfirmDialog(
              parent, panel, "Wind altitude profile", JOptionPane.OK_CANCEL_OPTION)
          != JOptionPane.OK_OPTION) return;
      if (table.isEditing()) table.getCellEditor().stopCellEditing();
      var array = Json.createArrayBuilder();
      for (int row = 0; row < model.getRowCount(); row++)
        array.add(
            Json.createObjectBuilder()
                .add("altitudeM", number(model, row, 0))
                .add("speedMS", number(model, row, 1))
                .add("directionRad", Math.toRadians(number(model, row, 2)))
                .add("sigmaMS", number(model, row, 3)));
      write.accept("/flight/editor/options/windLevels", array.build());
    } catch (Exception e) {
      JOptionPane.showMessageDialog(
          parent, e.getMessage(), "Wind profile", JOptionPane.ERROR_MESSAGE);
    }
  }

  private static double number(DefaultTableModel table, int row, int column) {
    double value = Double.parseDouble(table.getValueAt(row, column).toString());
    if (!Double.isFinite(value))
      throw new IllegalArgumentException("Wind levels need finite numbers");
    return value;
  }
}
