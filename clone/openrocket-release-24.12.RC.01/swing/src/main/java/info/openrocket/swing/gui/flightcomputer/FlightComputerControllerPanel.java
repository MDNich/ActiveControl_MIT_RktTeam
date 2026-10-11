package info.openrocket.swing.gui.flightcomputer;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.simulation.flightcomputer.*;
import info.openrocket.swing.gui.components.BasicTree;
import jakarta.json.*;
import net.miginfocom.swing.MigLayout;
import javax.swing.*;
import javax.swing.event.*;
import javax.swing.tree.*;
import java.awt.*;
import java.awt.event.*;
import java.util.*;
import java.util.List;
import java.util.function.*;

/** Signal-flow controller editor sharing the designer's immutable history and file lifecycle. */
public final class FlightComputerControllerPanel extends JPanel {
    private final Supplier<FlightComputerDesign> design;
    private final Supplier<FlightComputerControllerSettings> settings;
    private final BiConsumer<FlightComputerDesign, FlightComputerControllerSettings> apply;
    private final Simulation simulation;
    private final JTree tree = new BasicTree();
    private final JPanel inspector = new JPanel(new BorderLayout());
    private final Flow flow = new Flow();
    private final JLabel status = new JLabel();
    private final JButton applyButton = new JButton("Apply controller");
    private final JTabbedPane propertyTabs = new JTabbedPane();
    private String selected = "";
    private JsonObject draft;
    private boolean refreshing, pending;
    private FlightComputerActuatorMappingPanel mapping;
    private FlightComputerControllerSettings draftSettings;

    private record Item(String id, String label) {
        /**
         * Returns the display representation used by this value in logs or UI controls.
         *
         * @return the display representation used by this value in logs or UI controls
         */
        public String toString() {
            return label;
        }
    }

    /**
     * Builds the flight computer controller panel and connects its controls to the supplied model.
     *
     * @param design external flight-computer design snapshot
     * @param settings validated configuration for this operation
     * @param apply initial apply
     * @param simulation simulation being configured or executed
     */
    public FlightComputerControllerPanel(
            Supplier<FlightComputerDesign> design,
            Supplier<FlightComputerControllerSettings> settings,
            BiConsumer<FlightComputerDesign, FlightComputerControllerSettings> apply,
            Simulation simulation) {
        super(new BorderLayout(8, 8));
        this.design = design;
        this.settings = settings;
        this.apply = apply;
        this.simulation = simulation;
        setName("fc.controllers");
        tree.setName("fc.controllers.tree");
        tree.setRowHeight(0);
        tree.setCellRenderer(
                new DefaultTreeCellRenderer() {
                    /**
                     * Configures the tree cell appearance for the supplied node, selection and
                     * expansion state.
                     *
                     * @param t t
                     * @param v v
                     * @param s s
                     * @param x X component or horizontal coordinate
                     * @param l l
                     * @param row zero-based row index
                     * @param f f
                     * @return the resulting Component
                     */
                    public Component getTreeCellRendererComponent(
                            JTree t,
                            Object v,
                            boolean s,
                            boolean x,
                            boolean l,
                            int row,
                            boolean f) {
                        super.getTreeCellRendererComponent(t, v, s, x, l, row, f);
                        setIcon(FlightComputerIcons.of("controller"));
                        return this;
                    }
                });
        tree.addTreeSelectionListener(
                e -> {
                    if (!refreshing
                            && tree.getLastSelectedPathComponent()
                                    instanceof DefaultMutableTreeNode n
                            && n.getUserObject() instanceof Item i
                            && !i.id.isBlank()) select(i.id);
                });
        var left = new JPanel(new BorderLayout(4, 4));
        left.add(new JScrollPane(tree));
        var buttons = new JPanel(new GridLayout(0, 1, 4, 4));
        button(buttons, "Add controller", this::addController);
        button(buttons, "Duplicate", this::duplicate);
        button(buttons, "Delete", this::delete);
        left.add(buttons, BorderLayout.SOUTH);
        var center = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, flow, inspector);
        center.setResizeWeight(.38);
        center.setDividerLocation(330);
        var split = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, left, center);
        split.setResizeWeight(0);
        split.setDividerLocation(215);
        add(split);
        var bottom = new JPanel(new BorderLayout(8, 8));
        bottom.add(status);
        bottom.add(applyButton, BorderLayout.EAST);
        add(bottom, BorderLayout.SOUTH);
        applyButton.setName("fc.controller.apply");
        applyButton.addActionListener(e -> safe(this::applyPending));
        for (String modifier : List.of("control", "meta")) {
            bind(modifier + " ENTER", "apply-" + modifier, this::applyPending);
            bind(
                    modifier + " D",
                    "duplicate-" + modifier,
                    () -> {
                        if (!(KeyboardFocusManager.getCurrentKeyboardFocusManager().getFocusOwner()
                                instanceof javax.swing.text.JTextComponent)) duplicate();
                    });
        }
        tree.getInputMap().put(KeyStroke.getKeyStroke("DELETE"), "delete");
        tree.getActionMap()
                .put(
                        "delete",
                        new AbstractAction() {
                            /**
                             * Handles the registered Swing action using the current editor or view
                             * state.
                             *
                             * @param e event dispatched by the UI
                             */
                            public void actionPerformed(ActionEvent e) {
                                safe(FlightComputerControllerPanel.this::delete);
                            }
                        });
        refresh();
    }

    /**
     * Installs a controller-editor keyboard shortcut.
     *
     * @param stroke stroke
     * @param name display name or lookup key
     * @param r r
     */
    private void bind(String stroke, String name, Runnable r) {
        getInputMap(WHEN_ANCESTOR_OF_FOCUSED_COMPONENT).put(KeyStroke.getKeyStroke(stroke), name);
        getActionMap()
                .put(
                        name,
                        new AbstractAction() {
                            /**
                             * Handles the registered Swing action using the current editor or view
                             * state.
                             *
                             * @param e event dispatched by the UI
                             */
                            public void actionPerformed(ActionEvent e) {
                                safe(r);
                            }
                        });
    }

    /**
     * Runs an editor action and reports validation errors without losing pending edits.
     *
     * @param r r
     */
    private void safe(Runnable r) {
        try {
            r.run();
        } catch (RuntimeException ex) {
            status.setText(ex.getMessage());
            JOptionPane.showMessageDialog(
                    this, ex.getMessage(), "Controller", JOptionPane.ERROR_MESSAGE);
        }
    }

    /**
     * Creates a controller-editor action button.
     *
     * @param p p
     * @param text text
     * @param r r
     */
    private void button(JPanel p, String text, Runnable r) {
        var b = new JButton(text);
        b.addActionListener(e -> safe(r));
        p.add(b);
    }

    /**
     * Reports whether the inspector contains unapplied edits.
     *
     * @return true when the condition described above holds
     */
    public boolean hasPending() {
        return pending;
    }

    /**
     * Commits pending edits and selects a controller block.
     *
     * @param id stable identifier
     */
    public void select(String id) {
        if (!id.equals(selected) && pending) {
            try {
                applyPending();
            } catch (RuntimeException e) {
                safe(
                        () -> {
                            throw e;
                        });
                return;
            }
        }
        selected = id;
        refresh();
    }

    /** Creates a new controller block with algorithm defaults. */
    public void addController() {
        applyPending();
        var d = design.get();
        var list = new ArrayList<>(d.controllers());
        var c = FlightComputerController.create(d);
        list.add(c);
        selected = c.id();
        apply.accept(FlightComputerController.replace(d, list), settings.get());
    }

    /** Copies the selected controller with a fresh stable ID. */
    public void duplicate() {
        if (draft == null) return;
        applyPending();
        var d = design.get();
        var old =
                d.controllers().stream()
                        .filter(c -> c.id().equals(selected))
                        .findFirst()
                        .orElseThrow();
        var c =
                new FlightComputerController(
                        Json.createObjectBuilder(old.json())
                                .add("id", UUID.randomUUID().toString())
                                .add("name", old.name() + " copy")
                                .add("enabled", false)
                                .build());
        var list = new ArrayList<>(d.controllers());
        list.add(c);
        selected = c.id();
        apply.accept(FlightComputerController.replace(d, list), settings.get());
    }

    /** Removes the selected controller and its physical assignments. */
    public void delete() {
        if (draft == null) return;
        var d = design.get();
        var list = d.controllers().stream().filter(c -> !c.id().equals(selected)).toList();
        var map =
                new FlightComputerControllerSettings(
                        d.id(),
                        settings.get().bindings().stream()
                                .filter(b -> !b.controllerId().equals(selected))
                                .toList());
        selected = "";
        pending = false;
        apply.accept(FlightComputerController.replace(d, list), map);
    }

    /** Refreshes the controller list, signal-flow preview and inspector. */
    public void refresh() {
        refreshing = true;
        try {
            var d = design.get();
            var root = new DefaultMutableTreeNode(new Item("", "Controllers"));
            DefaultMutableTreeNode selectedNode = null;
            if (d.controllers().stream().noneMatch(c -> c.id().equals(selected)))
                selected = d.controllers().isEmpty() ? "" : d.controllers().get(0).id();
            for (var c : d.controllers()) {
                var n =
                        new DefaultMutableTreeNode(
                                new Item(c.id(), c.name() + (c.enabled() ? "" : " (disabled)")));
                root.add(n);
                if (c.id().equals(selected)) selectedNode = n;
            }
            tree.setModel(new DefaultTreeModel(root));
            tree.expandRow(0);
            if (selectedNode != null) tree.setSelectionPath(new TreePath(selectedNode.getPath()));
            draft =
                    d.controllers().stream()
                            .filter(c -> c.id().equals(selected))
                            .map(FlightComputerController::json)
                            .findFirst()
                            .orElse(null);
            draftSettings = settings.get();
            pending = false;
            buildInspector();
            updateStatus();
            flow.repaint();
        } finally {
            refreshing = false;
        }
    }

    /**
     * Returns the currently selected controller definition.
     *
     * @return the currently selected controller definition
     */
    private FlightComputerController current() {
        return new FlightComputerController(draft);
    }

    /** Marks the inspector as changed and updates its status. */
    private void changed() {
        if (!refreshing) {
            pending = true;
            updateStatus();
            flow.repaint();
        }
    }

    /**
     * Applies one immutable JSON property change to the selected controller.
     *
     * @param key configuration or lookup key
     * @param v v
     */
    private void put(String key, JsonValue v) {
        if (refreshing) return;
        draft = Json.createObjectBuilder(draft).add(key, v).build();
        changed();
    }

    /**
     * Creates a property form with consistent label/editor spacing.
     *
     * @return the resulting JPanel
     */
    private JPanel form() {
        return new JPanel(new MigLayout("fillx,insets 12,wrap 2", "[grow,fill][grow,fill]"));
    }

    /**
     * Adds a section title to a controller property form.
     *
     * @param p p
     * @param text text
     */
    private void title(JPanel p, String text) {
        var label = new JLabel(text);
        label.setFont(label.getFont().deriveFont(Font.BOLD));
        p.add(label, "newline,span,growx,gapbottom 5");
    }

    /**
     * Adds a labeled editor to the current form.
     *
     * @param p p
     * @param name display name or lookup key
     * @param c selected channel or controller
     * @param full full
     */
    private void field(JPanel p, String name, JComponent c, boolean full) {
        var box = new JPanel(new BorderLayout(3, 3));
        var label = new JLabel(name);
        label.setLabelFor(c);
        box.add(label, BorderLayout.NORTH);
        box.add(c);
        p.add(box, full ? "newline,span,growx,wmin 0" : "growx,wmin 0");
    }

    /**
     * Creates a text editor that tracks pending changes.
     *
     * @param key configuration or lookup key
     * @return the resulting JTextField
     */
    private JTextField text(String key) {
        var f = new JTextField(current().s(key));
        f.setName("fc.controller." + key);
        f.getDocument()
                .addDocumentListener(
                        new DocumentListener() {
                            /**
                             * Refreshes pending-edit state after text is inserted into the
                             * document.
                             *
                             * @param e event dispatched by the UI
                             */
                            public void insertUpdate(DocumentEvent e) {
                                update();
                            }

                            /**
                             * Refreshes pending-edit state after text is removed from the document.
                             *
                             * @param e event dispatched by the UI
                             */
                            public void removeUpdate(DocumentEvent e) {
                                update();
                            }

                            /**
                             * Refreshes editor state after document attributes change.
                             *
                             * @param e event dispatched by the UI
                             */
                            public void changedUpdate(DocumentEvent e) {
                                update();
                            }

                            /** Applies a controller property change to the current draft. */
                            void update() {
                                put(key, Json.createValue(f.getText()));
                            }
                        });
        f.addActionListener(e -> safe(this::applyPending));
        return f;
    }

    /**
     * Creates a bounded numeric controller-property editor.
     *
     * @param p p
     * @param label label
     * @param key configuration or lookup key
     * @param divisor divisor
     * @param suffix suffix
     * @throws IllegalArgumentException when rejected: Invalid number
     */
    private void number(JPanel p, String label, String key, double divisor, String suffix) {
        double v = current().n(key) / divisor;
        var spinner =
                new JSpinner(new SpinnerNumberModel(v, -1e12, 1e12, divisor == 1000 ? .1 : .01));
        spinner.setEditor(new JSpinner.NumberEditor(spinner, "0.########"));
        spinner.setName("fc.controller." + key);
        spinner.addChangeListener(
                e -> {
                    double value = ((Number) spinner.getValue()).doubleValue() * divisor;
                    put(
                            key,
                            key.endsWith("Us")
                                    ? Json.createValue(Math.round(value))
                                    : Json.createValue(value));
                });
        ((JSpinner.NumberEditor) spinner.getEditor())
                .getTextField()
                .addActionListener(
                        e ->
                                safe(
                                        () -> {
                                            try {
                                                spinner.commitEdit();
                                            } catch (java.text.ParseException ex) {
                                                throw new IllegalArgumentException(
                                                        "Invalid number");
                                            }
                                            applyPending();
                                        }));
        field(p, label + (suffix.isEmpty() ? "" : " (" + suffix + ")"), spinner, false);
    }

    /**
     * Creates a boolean controller-property editor.
     *
     * @param p p
     * @param label label
     * @param key configuration or lookup key
     */
    private void check(JPanel p, String label, String key) {
        var box = new JCheckBox(label, current().flag(key));
        box.setName("fc.controller." + key);
        box.addActionListener(e -> put(key, box.isSelected() ? JsonValue.TRUE : JsonValue.FALSE));
        p.add(box, "span,growx");
    }

    /**
     * Creates a dropdown for a finite set of controller options.
     *
     * @param p p
     * @param label label
     * @param key configuration or lookup key
     * @param items ite in milliseconds
     * @param rebuild rebuild
     * @return the resulting JComboBox&lt;Item&gt;
     */
    private JComboBox<Item> choices(
            JPanel p, String label, String key, List<Item> items, boolean rebuild) {
        var combo = new JComboBox<>(items.toArray(Item[]::new));
        combo.setName("fc.controller." + key);
        for (var item : items) if (item.id.equals(current().s(key))) combo.setSelectedItem(item);
        combo.addActionListener(
                e ->
                        safe(
                                () -> {
                                    if (refreshing || !(combo.getSelectedItem() instanceof Item i))
                                        return;
                                    String oldUnit = inputUnit();
                                    stashMappings();
                                    put(key, Json.createValue(i.id));
                                    if (key.equals("source")) {
                                        var node = design.get().node(i.id);
                                        if (node != null)
                                            draft =
                                                    Json.createObjectBuilder(draft)
                                                            .add(
                                                                    "channel",
                                                                    FlightComputerSignals.channels(
                                                                                    node.getString(
                                                                                            "type"))
                                                                            .get(0)
                                                                            .id())
                                                            .build();
                                    }
                                    if (key.equals("source") || key.equals("channel")) {
                                        if (!oldUnit.equals(inputUnit()))
                                            draft =
                                                    Json.createObjectBuilder(draft)
                                                            .add("kp", 0)
                                                            .add("ki", 0)
                                                            .add("kd", 0)
                                                            .add("setpoint", 0)
                                                            .add("wrapAngle", false)
                                                            .add("derivativeSource", "")
                                                            .add("derivativeChannel", "")
                                                            .build();
                                    }
                                    if (key.equals("derivativeSource")) {
                                        var channels = compatibleDerivativeChannels(i.id);
                                        draft =
                                                Json.createObjectBuilder(draft)
                                                        .add(
                                                                "derivativeChannel",
                                                                channels.isEmpty()
                                                                        ? ""
                                                                        : channels.get(0).id)
                                                        .build();
                                    }
                                    if (key.equals("mode")) {
                                        boolean brakes = i.id.equals("AIRBRAKES");
                                        draft =
                                                Json.createObjectBuilder(draft)
                                                        .add("minimum", brakes ? 0 : -15)
                                                        .add("maximum", brakes ? 1 : 15)
                                                        .add("trim", 0)
                                                        .build();
                                        draftSettings =
                                                new FlightComputerControllerSettings(
                                                        design.get().id(),
                                                        draftSettings.bindings().stream()
                                                                .filter(
                                                                        b ->
                                                                                !b.controllerId()
                                                                                        .equals(
                                                                                                selected))
                                                                .toList());
                                    }
                                    if (key.equals("type")
                                            && (i.id.equals("LEGACY_ROLL")
                                                    || i.id.equals("LEGACY_APOGEE"))) {
                                        boolean roll = i.id.equals("LEGACY_ROLL");
                                        draft =
                                                Json.createObjectBuilder(draft)
                                                        .add(
                                                                "source",
                                                                FlightComputerController.source(
                                                                        design.get(),
                                                                        roll
                                                                                ? "gyroscope"
                                                                                : "barometer"))
                                                        .add("channel", roll ? "roll" : "altitude")
                                                        .add("mode", roll ? "TABS" : "AIRBRAKES")
                                                        .add("baseline", "fixed")
                                                        .add("setpoint", roll ? 0 : 4550)
                                                        .add("derivativeSource", "")
                                                        .add("derivativeChannel", "")
                                                        .add("kp", roll ? .08444 : 1)
                                                        .add("ki", roll ? 0 : 2)
                                                        .add("kd", roll ? .02111 : 0)
                                                        .add("minimum", roll ? -90 : 0)
                                                        .add("maximum", roll ? 90 : 1)
                                                        .add("trim", 0)
                                                        .add("periodUs", 10000)
                                                        .add("executionUs", 0)
                                                        .build();
                                        draftSettings =
                                                new FlightComputerControllerSettings(
                                                        design.get().id(),
                                                        draftSettings.bindings().stream()
                                                                .filter(
                                                                        b ->
                                                                                !b.controllerId()
                                                                                        .equals(
                                                                                                selected))
                                                                .toList());
                                    }
                                    if (rebuild) {
                                        refreshing = true;
                                        try {
                                            buildInspector();
                                        } finally {
                                            refreshing = false;
                                        }
                                    }
                                    changed();
                                }));
        field(p, label, combo, true);
        return combo;
    }

    /** Retains target assignment edits while switching controller views. */
    private void stashMappings() {
        if (mapping != null) draftSettings = mapping.settings(design.get().id());
    }

    /**
     * Returns the selected measured input's engineering unit.
     *
     * @return the selected measured input's engineering unit
     */
    private String inputUnit() {
        try {
            return FlightComputerSignals.descriptor(
                            design.get(), current().s("source"), current().s("channel"))
                    .unit();
        } catch (RuntimeException e) {
            return "?";
        }
    }

    /**
     * Lists eligible measured sensor sources for the input dropdown.
     *
     * @return the resulting List&lt;Item&gt;
     */
    private List<Item> sensorItems() {
        return FlightComputerSignals.sources(design.get()).stream()
                .map(
                        n ->
                                new Item(
                                        n.getString("id"),
                                        n.getString("name")
                                                + " · "
                                                + (design.get().node(n.getString("board", ""))
                                                                == null
                                                        ? n.getString("type")
                                                        : design.get()
                                                                .node(n.getString("board"))
                                                                .getString("name"))
                                                + " ["
                                                + n.getString("id")
                                                        .substring(
                                                                0,
                                                                Math.min(
                                                                        8,
                                                                        n.getString("id").length()))
                                                + "]"))
                .toList();
    }

    /**
     * Lists design state IDs for controller activation gates.
     *
     * @return the resulting List&lt;Item&gt;
     */
    private List<Item> stateItems() {
        return FlightComputerStateMachine.states(design.get().stateMachine()).stream()
                .map(s -> new Item(s.getString("id"), s.getString("name")))
                .toList();
    }

    /**
     * Lists channels exposed by the selected sensor source.
     *
     * @param source source text, model or measured-channel selection
     * @return the resulting List&lt;Item&gt;
     */
    private List<Item> channels(String source) {
        var n = design.get().node(source);
        return n == null
                ? List.of()
                : FlightComputerSignals.channels(n.getString("type")).stream()
                        .map(c -> new Item(c.id(), c.toString()))
                        .toList();
    }

    /**
     * Lists measured derivative channels compatible with the selected controller input.
     *
     * @param source source text, model or measured-channel selection
     * @return the resulting List&lt;Item&gt;
     */
    private List<Item> compatibleDerivativeChannels(String source) {
        var n = design.get().node(source);
        if (n == null) return List.of();
        String unit;
        try {
            unit =
                    FlightComputerSignals.descriptor(
                                    design.get(), current().s("source"), current().s("channel"))
                            .derivativeUnit();
        } catch (RuntimeException e) {
            return List.of();
        }
        return FlightComputerSignals.channels(n.getString("type")).stream()
                .filter(c -> c.unit().equals(unit))
                .map(c -> new Item(c.id(), c.toString()))
                .toList();
    }

    /**
     * Adds wrapped explanatory text to the inspector.
     *
     * @param p p
     * @param text text
     */
    private void note(JPanel p, String text) {
        var area =
                new JTextArea(text) {
                    /**
                     * Returns the preferred size used by the surrounding Swing layout.
                     *
                     * @return the preferred size used by the surrounding Swing layout
                     */
                    public Dimension getPreferredSize() {
                        int width =
                                getParent() == null || getParent().getWidth() < 120
                                        ? 360
                                        : Math.max(100, getParent().getWidth() - 28);
                        setSize(width, Short.MAX_VALUE);
                        return super.getPreferredSize();
                    }
                };
        area.setEditable(false);
        area.setLineWrap(true);
        area.setWrapStyleWord(true);
        area.setOpaque(false);
        area.setFont(UIManager.getFont("Label.font"));
        p.add(area, "newline,span,growx,wmin 0");
    }

    /** Builds setup, tuning, timing and state controls for the selected block. */
    private void buildInspector() {
        int selectedTab = propertyTabs.getSelectedIndex();
        inspector.removeAll();
        propertyTabs.removeAll();
        mapping = null;
        if (draft == null) {
            inspector.add(
                    new JLabel(
                            "Add a controller to select its input, baseline and output.",
                            SwingConstants.CENTER));
            applyButton.setEnabled(false);
            inspector.revalidate();
            return;
        }
        applyButton.setEnabled(true);
        var c = current();
        var setup = form();
        title(setup, "Controller");
        field(setup, "Name", text("name"), true);
        check(setup, "Enabled", "enabled");
        choices(
                setup,
                "Type",
                "type",
                List.of(
                        new Item("PID", "PID / P / PI / PD"),
                        new Item("VELOCITY_PID", "Velocity-weighted PID"),
                        new Item("CONSTANT", "Constant output"),
                        new Item("LEGACY_ROLL", "Existing roll controller"),
                        new Item("LEGACY_APOGEE", "Existing apogee controller")),
                true);
        var boards = new ArrayList<Item>();
        boards.add(new Item("", "Outside a board"));
        design.get().nodes().getValuesAs(JsonObject.class).stream()
                .filter(n -> FlightComputerModels.isBoard(n.getString("type")))
                .forEach(n -> boards.add(new Item(n.getString("id"), n.getString("name"))));
        choices(setup, "Board grouping", "board", boards, false);
        if (c.type() != FlightComputerController.Type.CONSTANT) {
            title(setup, "Measured input & baseline");
            choices(setup, "Sensor / estimator", "source", sensorItems(), true);
            choices(setup, "Quantity", "channel", channels(c.s("source")), true);
            choices(
                    setup,
                    "Baseline",
                    "baseline",
                    List.of(
                            new Item("fixed", "Fixed value"),
                            new Item("ground", "Capture after ground calibration"),
                            new Item("entry", "Capture on state entry")),
                    true);
            number(
                    setup,
                    c.s("baseline").equals("fixed") ? "Maintain" : "Capture offset",
                    "setpoint",
                    1,
                    inputUnit());
            if (c.s("baseline").equals("entry"))
                choices(setup, "Capture state", "captureState", stateItems(), false);
            if (inputUnit().equals("deg")) check(setup, "Use shortest-angle error", "wrapAngle");
            note(
                    setup,
                    "Uses noisy, held FC measurements. Roll is −sensor X; other axes follow the configured sensor mounting.");
        }
        title(setup, "Output & actuator");
        choices(
                setup,
                "Actuation",
                "mode",
                List.of(
                        new Item("TABS", "Fin tabs"),
                        new Item("CANT", "Fin cant"),
                        new Item("AIRBRAKES", "Airbrakes")),
                true);
        mapping =
                new FlightComputerActuatorMappingPanel(simulation, c, draftSettings, this::changed);
        setup.add(mapping, "span,growx,wmin 0");
        note(
                setup,
                "Mapped enabled blocks reserve their targets, including inactive neutral output. Explicit Java writes to these targets are rejected; broadcasts skip them.");
        var tuning = form();
        title(
                tuning,
                c.type() == FlightComputerController.Type.CONSTANT
                        ? "Fixed command"
                        : "Controller gains");
        String outputUnit =
                c.mode() == FlightComputerController.Mode.AIRBRAKES ? "fraction" : "deg";
        boolean legacy =
                c.type() == FlightComputerController.Type.LEGACY_ROLL
                        || c.type() == FlightComputerController.Type.LEGACY_APOGEE;
        if (legacy)
            note(
                    tuning,
                    "Existing controller formulas and internal limits are retained. Apogee gains use the firmware's sensitivity scaling and per-update integral. Generic integral and derivative-filter settings do not apply to these adapters. Plotted P/I/D contributions precede internal limits.");
        if (c.type() != FlightComputerController.Type.CONSTANT) {
            number(
                    tuning,
                    "Kp",
                    "kp",
                    1,
                    legacy ? "firmware gain" : outputUnit + "/(" + inputUnit() + ")");
            number(
                    tuning,
                    "Ki",
                    "ki",
                    1,
                    legacy ? "firmware gain" : outputUnit + "/(" + inputUnit() + "·s)");
            number(
                    tuning,
                    "Kd",
                    "kd",
                    1,
                    legacy ? "firmware gain" : outputUnit + "·s/(" + inputUnit() + ")");
            if (!legacy) {
                number(tuning, "Integral limit", "integralLimit", 1, inputUnit() + "·s");
                number(tuning, "Derivative filter", "filterUs", 1000, "ms");
                var derivativeSources = new ArrayList<Item>();
                derivativeSources.add(new Item("", "Filtered difference of measurement"));
                sensorItems().stream()
                        .filter(i -> !compatibleDerivativeChannels(i.id).isEmpty())
                        .forEach(derivativeSources::add);
                choices(tuning, "Derivative input", "derivativeSource", derivativeSources, true);
                if (!c.s("derivativeSource").isEmpty())
                    choices(
                            tuning,
                            "Derivative quantity",
                            "derivativeChannel",
                            compatibleDerivativeChannels(c.s("derivativeSource")),
                            false);
            }
            if (legacy && c.type() == FlightComputerController.Type.LEGACY_ROLL)
                note(
                        tuning,
                        "The existing roll adapter uses the selected gyro’s roll rate for its derivative, and primary barometer height for aerodynamic scaling.");
        }
        if (c.type() == FlightComputerController.Type.VELOCITY_PID
                || c.type() == FlightComputerController.Type.LEGACY_ROLL
                || c.type() == FlightComputerController.Type.LEGACY_APOGEE) {
            title(tuning, "Velocity input");
            choices(
                    tuning,
                    "Source",
                    "velocitySource",
                    sensorItems().stream()
                            .filter(
                                    i ->
                                            channels(i.id).stream()
                                                    .anyMatch(x -> x.id.equals("velocity")))
                            .toList(),
                    false);
            note(tuning, "Integrated longitudinal velocity is an estimate, not measured airspeed.");
        }
        if (c.type() == FlightComputerController.Type.VELOCITY_PID) {
            number(tuning, "Reference speed", "referenceSpeed", 1, "m/s");
            number(tuning, "Speed floor", "speedFloor", 1, "m/s");
            number(tuning, "Inverse-speed exponent", "exponent", 1, "");
            number(tuning, "Minimum weight", "minimumWeight", 1, "×");
            number(tuning, "Maximum weight", "maximumWeight", 1, "×");
            number(tuning, "Activation speed", "activationSpeed", 1, "m/s");
            number(tuning, "Speed hysteresis", "speedHysteresis", 1, "m/s");
        }
        title(tuning, "Logical output limits");
        if (c.type() == FlightComputerController.Type.CONSTANT)
            number(tuning, "Fixed command", "constantValue", 1, outputUnit);
        number(tuning, "Neutral / trim", "trim", 1, outputUnit);
        number(tuning, "Minimum", "minimum", 1, outputUnit);
        number(tuning, "Maximum", "maximum", 1, outputUnit);
        number(tuning, "Direction (+1 or −1)", "direction", 1, "");
        var timing = form();
        title(timing, "Active states");
        for (var state : stateItems()) {
            var box = new JCheckBox(state.label, c.states().contains(state.id));
            box.addActionListener(
                    e -> {
                        var values = new LinkedHashSet<>(current().states());
                        if (box.isSelected()) values.add(state.id);
                        else values.remove(state.id);
                        var a = Json.createArrayBuilder();
                        values.forEach(a::add);
                        put("states", a.build());
                    });
            timing.add(box, "span");
        }
        title(timing, "Virtual timing");
        number(timing, "Update period", "periodUs", 1000, "ms");
        number(timing, "Modeled execution cost", "executionUs", 1, "µs");
        number(timing, "Stale input after", "staleUs", 1000, "ms");
        number(timing, "Hold invalid input for", "invalidHoldUs", 1000, "ms");
        number(timing, "Baseline averaging window", "captureAverageUs", 1000, "ms");
        check(timing, "Reset integral on state entry", "resetIntegral");
        check(timing, "Hold last output while inactive", "inactiveHold");
        note(
                timing,
                "Servo outputs latch every 20 ms. Each block has an independent virtual schedule; board grouping does not model processor contention or communication delays. Sensor acquisition follows the FC read cadence.");
        for (var entry :
                List.of(
                        Map.entry("Setup", setup),
                        Map.entry("Tuning", tuning),
                        Map.entry("Timing & states", timing))) {
            var scroll = new JScrollPane(entry.getValue());
            scroll.getVerticalScrollBar().setUnitIncrement(16);
            scroll.setMinimumSize(new Dimension(260, 150));
            propertyTabs.addTab(entry.getKey(), scroll);
        }
        propertyTabs.setSelectedIndex(Math.max(0, Math.min(2, selectedTab)));
        inspector.add(propertyTabs);
        inspector.revalidate();
        inspector.repaint();
    }

    /**
     * Commits pending spinner text before reading numeric values.
     *
     * @param parent parent window, model or output directory
     * @throws IllegalArgumentException when rejected: Invalid controller number
     */
    private void commitNumbers(Container parent) {
        for (var child : parent.getComponents()) {
            if (child instanceof JSpinner spinner) {
                try {
                    spinner.commitEdit();
                } catch (java.text.ParseException e) {
                    throw new IllegalArgumentException("Invalid controller number", e);
                }
            } else if (child instanceof Container container) commitNumbers(container);
        }
    }

    /** Validates and applies the controller definition and physical mappings together. */
    public void applyPending() {
        if (draft == null) return;
        commitNumbers(inspector);
        if (!pending) return;
        var d = design.get();
        var c = current();
        c.validate(d);
        stashMappings();
        var blocks = d.controllers().stream().map(x -> x.id().equals(c.id()) ? c : x).toList();
        var next = FlightComputerController.replace(d, blocks);
        if (simulation != null)
            draftSettings.validate(next, simulation.getActiveConfiguration().getActiveComponents());
        pending = false;
        apply.accept(next, draftSettings);
        System.out.println(
                "FC controller.apply id="
                        + c.id()
                        + " name="
                        + c.name()
                        + " type="
                        + c.type()
                        + " enabled="
                        + c.enabled());
    }

    /** Refreshes the applied/pending status and signal-flow preview. */
    private void updateStatus() {
        if (draft == null) {
            status.setText("No controller blocks");
            return;
        }
        long targets =
                (mapping == null ? draftSettings : readMappingQuietly())
                        .forController(selected)
                        .size();
        String error = "";
        try {
            current().validate(design.get());
        } catch (RuntimeException e) {
            error = e.getMessage();
        }
        status.setText(
                (pending ? "Unapplied changes" : "Applied")
                        + " · "
                        + (error.isEmpty() ? "Valid" : "Needs attention: " + error)
                        + " · "
                        + (targets == 0 ? "Recorded only" : targets + " target(s)"));
    }

    /**
     * Reads current target assignments while retaining actionable validation in the editor.
     *
     * @return the resulting FlightComputerControllerSettings
     */
    private FlightComputerControllerSettings readMappingQuietly() {
        try {
            return mapping.settings(design.get().id());
        } catch (RuntimeException e) {
            return draftSettings;
        }
    }

    private final class Flow extends JPanel {
        /** Creates a Flow value from the supplied . */
        Flow() {
            setPreferredSize(new Dimension(300, 540));
            setMinimumSize(new Dimension(180, 150));
            setToolTipText("Measured input → controller → selected physical actuator");
        }

        /**
         * Paints this view from the current model and selection state.
         *
         * @param raw g value
         */
        protected void paintComponent(Graphics raw) {
            super.paintComponent(raw);
            var g = (Graphics2D) raw.create();
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            boolean dark = getBackground().getRed() < 150;
            g.setColor(dark ? new Color(22, 38, 50) : new Color(232, 241, 248));
            g.fillRect(0, 0, getWidth(), getHeight());
            if (draft == null) {
                g.dispose();
                return;
            }
            var c = current();
            int x = 18, w = Math.max(120, getWidth() - 36), y = 48;
            box(
                    g,
                    x,
                    y,
                    w,
                    c.type() == FlightComputerController.Type.CONSTANT
                            ? "Fixed command"
                            : sensorName(),
                    c.type() == FlightComputerController.Type.CONSTANT
                            ? "No sensor input"
                            : channelName(),
                    false,
                    dark);
            arrow(g, y + 80, "Measured / held");
            y += 135;
            box(
                    g,
                    x,
                    y,
                    w,
                    c.name(),
                    c.type()
                            + " · "
                            + (c.s("baseline").equals("fixed")
                                    ? "baseline " + c.n("setpoint") + " " + inputUnit()
                                    : "capture " + c.s("baseline")),
                    true,
                    dark);
            arrow(g, y + 80, "PWM latch · 20 ms");
            y += 135;
            var bindings = readMappingQuietly().forController(c.id());
            String target =
                    bindings.isEmpty()
                            ? "Recorded only"
                            : bindings.size() + " assigned actuator(s)";
            if (simulation != null && bindings.size() == 1)
                for (var component : simulation.getRocket())
                    if (component.getID().equals(bindings.get(0).deviceId()))
                        target = component.getName();
            box(
                    g,
                    x,
                    y,
                    w,
                    target,
                    c.mode() + " · " + c.n("minimum") + " to " + c.n("maximum"),
                    false,
                    dark);
            g.dispose();
        }

        /**
         * Resolves a stable sensor ID to its display name.
         *
         * @return formatted or resolved text
         */
        private String sensorName() {
            var n = design.get().node(current().s("source"));
            return n == null ? "Select a sensor" : n.getString("name");
        }

        /**
         * Resolves a signal-channel key to its display label.
         *
         * @return formatted or resolved text
         */
        private String channelName() {
            try {
                return FlightComputerSignals.descriptor(
                                design.get(), current().s("source"), current().s("channel"))
                        .quantity()
                        .toString();
            } catch (RuntimeException e) {
                return "Select a quantity";
            }
        }

        /**
         * Draws one block of the controller signal-flow preview.
         *
         * @param g graphics context
         * @param x X component or horizontal coordinate
         * @param y Y component or vertical coordinate
         * @param w w
         * @param title title
         * @param text text
         * @param selected whether this item is selected
         * @param dark dark
         */
        private void box(
                Graphics2D g,
                int x,
                int y,
                int w,
                String title,
                String text,
                boolean selected,
                boolean dark) {
            g.setColor(dark ? new Color(39, 70, 89) : new Color(246, 251, 255));
            g.fillRoundRect(x, y, w, 78, 8, 8);
            g.setStroke(new BasicStroke(selected ? 2 : 1));
            g.setColor(new Color(89, 147, 180));
            g.drawRoundRect(x, y, w, 78, 8, 8);
            g.setColor(dark ? new Color(229, 241, 248) : new Color(31, 50, 62));
            g.setFont(getFont().deriveFont(Font.BOLD));
            drawWrapped(g, title, x + 10, y + 23, w - 20, 2);
            g.setFont(getFont().deriveFont(Font.PLAIN));
            drawWrapped(g, text, x + 10, y + 58, w - 20, 1);
        }

        /**
         * Draws a wrapped label within the signal-flow block.
         *
         * @param g graphics context
         * @param text rest value
         * @param x X component or horizontal coordinate
         * @param y Y component or vertical coordinate
         * @param width width
         * @param lines lines
         */
        private void drawWrapped(Graphics2D g, String text, int x, int y, int width, int lines) {
            var fm = g.getFontMetrics();
            String rest = text;
            for (int row = 0; row < lines && !rest.isEmpty(); row++) {
                int end = rest.length();
                while (end > 1
                        && fm.stringWidth(rest.substring(0, end) + (end < rest.length() ? "…" : ""))
                                > width) end--;
                g.drawString(
                        rest.substring(0, end) + (end < rest.length() ? "…" : ""),
                        x,
                        y + row * fm.getHeight());
                rest = rest.substring(end);
            }
        }

        /**
         * Draws a directed connection in the signal-flow preview.
         *
         * @param g graphics context
         * @param y Y component or vertical coordinate
         * @param label label
         */
        private void arrow(Graphics2D g, int y, String label) {
            int x = getWidth() / 2;
            g.setColor(new Color(74, 164, 188));
            g.setStroke(new BasicStroke(1.5f));
            g.drawLine(x, y + 5, x, y + 32);
            g.drawLine(x, y + 32, x - 5, y + 26);
            g.drawLine(x, y + 32, x + 5, y + 26);
            g.setFont(getFont());
            g.drawString(
                    label, Math.max(10, x - g.getFontMetrics().stringWidth(label) / 2), y + 49);
        }
    }
}
