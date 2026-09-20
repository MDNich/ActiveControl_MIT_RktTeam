package info.openrocket.swing.gui.plot;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.startup.Application;
import info.openrocket.core.unit.UnitGroup;
import info.openrocket.swing.gui.util.GUIUtil;
import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.awt.*;
import java.awt.event.*;
import java.nio.file.Path;
import java.util.concurrent.CancellationException;

/** Interactive trajectory viewer; it never mutates or reruns the simulation. */
public final class Trajectory3DDialog extends JDialog {
    private final Simulation simulation;
    private final Trajectory3DPanel scene;
    private final TrajectoryPlayback playback = new TrajectoryPlayback();
    private final Timer timer;
    private final JSlider timeline = new JSlider(0, 10000, 0);
    private final JLabel status = new JLabel(text("loading"));
    private final JLabel readout = new JLabel(" ");
    private final JButton play = new JButton(text("play"));
    private final JComboBox<EventChoice> events = new JComboBox<>();
    private TrajectoryData data;
    private boolean refreshing, closed;
    private SwingWorker<TrajectoryData, Void> loader;
    private long loadVersion;

    public static String text(String key) { return Application.getTranslator().get("Trajectory3D." + key); }
    private record EventChoice(TrajectoryData.Marker marker) {
        @Override public String toString() { return String.format(java.util.Locale.ROOT, "%.2f s · %s %s", marker.time(),
                Application.getTranslator().get("FlightEvent.Type." + marker.type().name()), marker.source()); }
    }

    public Trajectory3DDialog(Window parent, Simulation simulation, int initialBranch) {
        super(parent, text("title") + " — " + simulation.getName(), ModalityType.DOCUMENT_MODAL);
        this.simulation = simulation;
        setDefaultCloseOperation(DISPOSE_ON_CLOSE);
        scene = new Trajectory3DPanel(message -> { playback.setPlaying(false); status.setText(message); play.setEnabled(false); });
        JPanel content = new JPanel(new BorderLayout(6, 6));
        content.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));
        JPanel tools = new JPanel(); tools.setLayout(new BoxLayout(tools, BoxLayout.Y_AXIS));
        JPanel camera = new JPanel(new FlowLayout(FlowLayout.LEADING, 6, 2));
        String[] names = new String[simulation.getSimulatedData().getBranchCount()];
        for (int i=0;i<names.length;i++) names[i]=simulation.getSimulatedData().getBranch(i).getName();
        JComboBox<String> branches = new JComboBox<>(names);
        branches.setSelectedIndex(Math.max(0, Math.min(names.length-1, initialBranch)));
        camera.add(new JLabel(text("branch"))); camera.add(branches);
        addButton(camera, "fit", scene::fit); addButton(camera, "reset", () -> scene.view("perspective"));
        addButton(camera, "top", () -> scene.view("top")); addButton(camera, "side", () -> scene.view("side"));
        addButton(camera, "perspective", () -> scene.view("perspective"));
        JCheckBox follow = new JCheckBox(text("follow")); follow.addActionListener(e -> scene.setFollow(follow.isSelected())); camera.add(follow);
        addButton(camera, "snapshot", this::saveSnapshot);
        tools.add(camera);
        JPanel appearance = new JPanel(new FlowLayout(FlowLayout.LEADING, 6, 2));
        JCheckBox velocity = new JCheckBox(text("velocity"), true); velocity.addActionListener(e -> scene.setShowVelocity(velocity.isSelected())); appearance.add(velocity);
        JCheckBox ground = new JCheckBox(text("projection"), true); ground.addActionListener(e -> scene.setShowProjection(ground.isSelected())); appearance.add(ground);
        appearance.add(new JLabel(text("rocketSize")));
        JSlider size = new JSlider(5, 40, 10); size.setPreferredSize(new Dimension(110, 25));
        size.setToolTipText(text("rocketSize.tip")); size.addChangeListener(e -> scene.setRocketSize(size.getValue()/10.0)); appearance.add(size);
        appearance.add(new JLabel(text("arrowSize")));
        JSlider arrow = new JSlider(1, 50, 10); arrow.setPreferredSize(new Dimension(110, 25));
        arrow.setToolTipText(text("arrowSize.tip")); arrow.addChangeListener(e -> scene.setArrowSeconds(arrow.getValue()/10.0)); appearance.add(arrow);
        tools.add(appearance);
        content.add(tools, BorderLayout.NORTH); content.add(scene, BorderLayout.CENTER);
        JPanel bottom = new JPanel(); bottom.setLayout(new BoxLayout(bottom, BoxLayout.Y_AXIS));
        JPanel transport = new JPanel(new FlowLayout(FlowLayout.LEADING, 6, 2));
        play.setEnabled(false);
        play.addActionListener(e -> {
            if (data == null) return;
            if (playback.time() >= data.end()) playback.seek(data.start());
            playback.setPlaying(!playback.isPlaying()); refresh();
        });
        transport.add(play);
        addButton(transport, "restart", () -> { if (data != null) { playback.seek(data.start()); refresh(); } });
        transport.add(new JLabel(text("speed")));
        JComboBox<String> speed = new JComboBox<>(new String[]{"0.1", "0.25", "0.5", "1", "2", "5"});
        speed.setEditable(true); speed.setSelectedItem("0.25");
        speed.addActionListener(e -> {
            try { playback.setSpeed(Double.parseDouble(speed.getSelectedItem().toString())); refresh(); }
            catch (IllegalArgumentException error) { JOptionPane.showMessageDialog(this, text("speed.invalid")); speed.setSelectedItem("0.25"); }
        });
        transport.add(speed); transport.add(new JLabel("×"));
        transport.add(readout);
        bottom.add(transport);
        timeline.setEnabled(false); timeline.setName("trajectory.time");
        timeline.getAccessibleContext().setAccessibleName(text("time"));
        timeline.addChangeListener(e -> {
            if (!refreshing && data != null) { playback.seek(data.start() + timeline.getValue()/10000.0*(data.end()-data.start())); refresh(); }
        });
        bottom.add(timeline);
        JPanel eventRow = new JPanel(new FlowLayout(FlowLayout.LEADING, 6, 2));
        eventRow.add(new JLabel(text("events"))); events.setPreferredSize(new Dimension(460, 27)); eventRow.add(events);
        events.addActionListener(e -> {
            if (!refreshing && events.getSelectedItem() instanceof EventChoice choice) { playback.seek(choice.marker().time()); refresh(); }
        });
        bottom.add(eventRow);
        bottom.add(status); bottom.add(new JLabel(text("gestures")));
        content.add(bottom, BorderLayout.SOUTH); setContentPane(content);
        timer = new Timer(33, e -> { if (playback.isPlaying()) refresh(); }); timer.start();
        branches.addActionListener(e -> load(branches.getSelectedIndex()));
        addWindowListener(new WindowAdapter() { @Override public void windowClosed(WindowEvent e) { shutdown(); } });
        GUIUtil.setDisposableDialogOptions(this, null);
        setMinimumSize(new Dimension(850, 650)); pack(); setLocationRelativeTo(parent);
        load(branches.getSelectedIndex());
    }
    private static void addButton(JPanel panel, String key, Runnable action) {
        JButton button = new JButton(text(key)); button.addActionListener(e -> action.run()); panel.add(button);
    }
    private void load(int branch) {
        playback.setPlaying(false); data = null; play.setEnabled(false); timeline.setEnabled(false); status.setText(text("loading"));
        if (loader != null) loader.cancel(true);
        long version = ++loadVersion;
        loader = new SwingWorker<>() {
            @Override protected TrajectoryData doInBackground() { return new TrajectoryData(simulation.getSimulatedData().getBranch(branch)); }
            @Override protected void done() {
                if (closed || version != loadVersion) return;
                try {
                    data = get(); playback.range(data.start(), data.end());
                    refreshing = true;
                    events.removeAllItems();
                    for (TrajectoryData.Marker event : data.markers()) if (event.time() >= data.start() && event.time() <= data.end()) events.addItem(new EventChoice(event));
                    events.setSelectedIndex(-1); refreshing = false;
                    scene.setData(data); play.setEnabled(scene.isAvailable()); timeline.setEnabled(true); refresh();
                } catch (CancellationException ignored) {
                } catch (Exception error) { status.setText(text("unavailable") + " " + (error.getCause() == null ? error.getMessage() : error.getCause().getMessage())); }
            }
        };
        loader.execute();
    }
    private void refresh() {
        if (data == null || closed) return;
        double time = playback.time();
        refreshing = true;
        timeline.setValue(data.end() == data.start() ? 0 : (int)Math.round((time-data.start())/(data.end()-data.start())*10000));
        refreshing = false;
        play.setText(text(playback.isPlaying() ? "pause" : "play"));
        var frame = data.at(time);
        String speed = Double.isFinite(frame.speed()) ? UnitGroup.UNITS_VELOCITY.getDefaultUnit().toStringUnit(frame.speed()) : "—";
        readout.setText(String.format(java.util.Locale.ROOT, "%.2f s   ·   %s: %s", time, text("trueSpeed"), speed));
        if (scene.isAvailable()) {
            if (frame.position() == null) status.setText(text("missing"));
            else if (frame.undersampled()) status.setText(text("undersampled"));
            else if (frame.attitude() == null) status.setText(text("noAttitude"));
            else if (frame.held()) status.setText(text("held"));
            else status.setText(text("equalScale"));
        }
        scene.setTime(time);
    }
    private void saveSnapshot() {
        if (data == null || !scene.isAvailable()) return;
        JFileChooser chooser = new JFileChooser(); chooser.setFileFilter(new FileNameExtensionFilter("PNG", "png")); chooser.setSelectedFile(new java.io.File("trajectory.png"));
        if (chooser.showSaveDialog(this) != JFileChooser.APPROVE_OPTION) return;
        Path chosen = chooser.getSelectedFile().toPath();
        Path path = chosen.toString().toLowerCase(java.util.Locale.ROOT).endsWith(".png") ? chosen : Path.of(chosen + ".png");
        if (java.nio.file.Files.exists(path) && JOptionPane.showConfirmDialog(this, text("overwrite"), text("snapshot"), JOptionPane.YES_NO_OPTION) != JOptionPane.YES_OPTION) return;
        scene.snapshot(image -> new SwingWorker<Void, Void>() {
            @Override protected Void doInBackground() throws Exception { javax.imageio.ImageIO.write(image, "png", path.toFile()); return null; }
            @Override protected void done() {
                try { get(); status.setText(text("saved") + " " + path.toAbsolutePath()); }
                catch (Exception e) { JOptionPane.showMessageDialog(Trajectory3DDialog.this, e.getMessage()); }
            }
        }.execute());
    }
    private void shutdown() {
        if (closed) return;
        closed = true; timer.stop(); if (loader != null) loader.cancel(true);
        System.out.println("TRAJECTORY_VIEW frames=" + scene.getRenderedFrames() + " mean_draw_ms=" + scene.getMeanDrawMillis());
        scene.close();
    }
}
