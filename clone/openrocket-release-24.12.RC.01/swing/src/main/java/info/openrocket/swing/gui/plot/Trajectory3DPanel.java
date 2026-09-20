package info.openrocket.swing.gui.plot;

import com.jogamp.opengl.*;
import com.jogamp.opengl.awt.GLJPanel;
import com.jogamp.opengl.glu.GLU;
import com.jogamp.opengl.util.awt.TextRenderer;
import info.openrocket.core.unit.UnitGroup;
import info.openrocket.core.util.Coordinate;
import info.openrocket.core.util.Quaternion;
import info.openrocket.swing.gui.figure3d.RocketFigure3d;
import info.openrocket.swing.gui.util.GUIUtil;
import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

/** A world-space trajectory scene with a camera independent of the recorded body attitude. */
public final class Trajectory3DPanel extends JPanel implements GLEventListener {
    private GLJPanel canvas;
    private final GLU glu = new GLU();
    private TextRenderer text;
    private volatile TrajectoryData data;
    private TrajectoryData compiled;
    private int trajectoryList, projectionList;
    private volatile double time, yaw = -35, tilt = 65, zoom = 1, panX, panY;
    private volatile double rocketSize = 1, arrowSeconds = 1;
    private volatile boolean showVelocity = true, showProjection = true, follow;
    private volatile Consumer<BufferedImage> snapshot;
    private final Consumer<String> failure;
    private boolean failed;
    private final List<Label> labels = new ArrayList<>();
    private record Label(String text, Coordinate location) {}
    private final double[] model = new double[16], projection = new double[16];
    private final int[] viewport = new int[4];
    private long frames, drawNanos;

    public Trajectory3DPanel(Consumer<String> failure) {
        super(new BorderLayout());
        this.failure = failure;
        setPreferredSize(new Dimension(900, 610));
        try {
            if (!RocketFigure3d.is3dEnabled()) throw new IllegalStateException(Trajectory3DDialog.text("disabled"));
            GLProfile profile = GLProfile.get(GLProfile.GL2);
            GLCapabilities caps = new GLCapabilities(profile);
            caps.setDepthBits(24);
            canvas = new GLJPanel(caps);
            canvas.addGLEventListener(this);
            canvas.setFocusable(true);
            canvas.getAccessibleContext().setAccessibleName(Trajectory3DDialog.text("title"));
            add(canvas, BorderLayout.CENTER);
            MouseAdapter mouse = new MouseAdapter() {
                private Point last;
                @Override public void mousePressed(MouseEvent e) { last = e.getPoint(); canvas.requestFocusInWindow(); }
                @Override public void mouseDragged(MouseEvent e) {
                    if (last == null) return;
                    double dx = e.getX()-last.x, dy = e.getY()-last.y;
                    if (e.isShiftDown() || (e.getModifiersEx() & (InputEvent.BUTTON2_DOWN_MASK | InputEvent.BUTTON3_DOWN_MASK)) != 0) {
                        panX += dx / Math.max(1, canvas.getWidth()) * 4;
                        panY -= dy / Math.max(1, canvas.getHeight()) * 4;
                    } else { yaw += dx * 0.5; tilt = Math.max(0, Math.min(180, tilt + dy * 0.5)); }
                    last = e.getPoint(); render();
                }
                @Override public void mouseWheelMoved(MouseWheelEvent e) {
                    zoom = Math.max(0.05, Math.min(50, zoom * Math.exp(-e.getPreciseWheelRotation()*0.1))); render();
                }
            };
            canvas.addMouseListener(mouse); canvas.addMouseMotionListener(mouse); canvas.addMouseWheelListener(mouse);
            canvas.setToolTipText(Trajectory3DDialog.text("gestures"));
        } catch (RuntimeException | LinkageError e) { fail(e); }
    }
    private void fail(Throwable error) {
        if (failed) return;
        failed = true;
        String message = Trajectory3DDialog.text("unavailable") + " " + error.getMessage();
        SwingUtilities.invokeLater(() -> {
            removeAll(); add(new JLabel(message), BorderLayout.CENTER); revalidate(); repaint(); failure.accept(message);
        });
    }
    public boolean isAvailable() { return canvas != null && !failed; }
    public void setData(TrajectoryData data) { this.data = data; this.time = data.start(); fit(); }
    public void setTime(double time) { this.time = time; render(); }
    public void setRocketSize(double size) { rocketSize = size; render(); }
    public void setArrowSeconds(double scale) { arrowSeconds = scale; render(); }
    public void setShowVelocity(boolean value) { showVelocity = value; render(); }
    public void setShowProjection(boolean value) { showProjection = value; render(); }
    public void setFollow(boolean value) { follow = value; render(); }
    public void fit() { zoom = 1; panX = panY = 0; render(); }
    public void view(String view) {
        switch (view) {
            case "top" -> { tilt = 0; yaw = 0; }
            case "side" -> { tilt = 90; yaw = 0; }
            default -> { tilt = 65; yaw = -35; }
        }
        fit();
    }
    public void snapshot(Consumer<BufferedImage> consumer) { snapshot = consumer; render(); }
    public void render() {
        if (!isAvailable()) return;
        try { canvas.display(); } catch (RuntimeException | LinkageError e) { fail(e); }
    }
    public void close() { if (canvas != null) canvas.destroy(); }
    public long getRenderedFrames() { return frames; }
    public double getMeanDrawMillis() { return frames == 0 ? 0 : drawNanos/1e6/frames; }

    @Override public void init(GLAutoDrawable drawable) {
        GL2 gl = drawable.getGL().getGL2();
        gl.glEnable(GL.GL_DEPTH_TEST);
        text = new TextRenderer(new Font(Font.SANS_SERIF, Font.PLAIN, 14), true, true);
        compiled = null;
    }
    @Override public void reshape(GLAutoDrawable drawable, int x, int y, int width, int height) {}
    @Override public void dispose(GLAutoDrawable drawable) {
        GL2 gl = drawable.getGL().getGL2();
        if (trajectoryList != 0) gl.glDeleteLists(trajectoryList, 1);
        if (projectionList != 0) gl.glDeleteLists(projectionList, 1);
        trajectoryList = projectionList = 0;
        if (text != null) { text.dispose(); text = null; }
    }
    @Override public void display(GLAutoDrawable drawable) {
        long before = System.nanoTime();
        try { draw(drawable); }
        catch (RuntimeException | LinkageError error) { fail(error); }
        finally { drawNanos += System.nanoTime()-before; frames++; }
    }
    private void draw(GLAutoDrawable drawable) {
        GL2 gl = drawable.getGL().getGL2();
        Color background = GUIUtil.getUITheme().getBackgroundColor();
        gl.glClearColor(background.getRed()/255f, background.getGreen()/255f, background.getBlue()/255f, 1);
        gl.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);
        TrajectoryData track = data;
        if (track == null) return;
        if (compiled != track) compileLine(gl, track);
        int w = Math.max(1, drawable.getSurfaceWidth()), h = Math.max(1, drawable.getSurfaceHeight());
        gl.glViewport(0, 0, w, h);
        gl.glMatrixMode(GL2.GL_PROJECTION); gl.glLoadIdentity();
        glu.gluPerspective(45, (double)w/h, 0.05, 200);
        gl.glMatrixMode(GL2.GL_MODELVIEW); gl.glLoadIdentity();
        gl.glTranslated(panX, panY, -4.8);
        gl.glRotated(-tilt, 1, 0, 0); gl.glRotated(yaw, 0, 0, 1);
        gl.glScaled(2*zoom/track.span(), 2*zoom/track.span(), 2*zoom/track.span());
        TrajectoryData.Frame frame = track.at(time);
        Coordinate center = follow && frame.position() != null ? frame.position() : track.center();
        gl.glTranslated(-center.x, -center.y, -center.z);
        labels.clear();
        grid(gl, track);
        gl.glColor3d(0.08, 0.57, 0.9); gl.glLineWidth(2); gl.glCallList(trajectoryList);
        if (showProjection) { gl.glColor3d(0.45, 0.55, 0.64); gl.glLineWidth(1); gl.glCallList(projectionList); }
        gl.glPointSize(7);
        gl.glBegin(GL.GL_POINTS); gl.glColor3d(0.15, 0.7, 0.35); vertex(gl, Coordinate.ZERO); gl.glEnd();
        labels.add(new Label(Trajectory3DDialog.text("launch"), Coordinate.ZERO));
        for (TrajectoryData.Marker marker : track.markers()) {
            if (marker.time() < track.start() || marker.time() > track.end()) continue;
            Coordinate p = track.at(marker.time()).position();
            if (p == null) continue;
            gl.glColor3d(0.88, 0.4, 0.12); gl.glBegin(GL.GL_POINTS); vertex(gl, p); gl.glEnd();
        }
        if (frame.position() != null) {
            double length = track.span()*0.055*rocketSize/zoom;
            if (frame.attitude() != null) rocket(gl, frame.position(), frame.attitude(), length);
            else { gl.glColor3d(0.92, 0.35, 0.12); gl.glPointSize(12); gl.glBegin(GL.GL_POINTS); vertex(gl, frame.position()); gl.glEnd(); }
            if (frame.recovery()) canopy(gl, frame.position(), frame.attitude(), length);
            if (showVelocity && frame.velocity() != null && frame.velocity().length() > 1e-8)
                arrow(gl, frame.position(), frame.velocity().multiply(arrowSeconds), length*0.15);
        }
        drawLabels(gl, w, h);
        Consumer<BufferedImage> capture = snapshot;
        if (capture != null) {
            snapshot = null;
            ByteBuffer pixels = ByteBuffer.allocateDirect(w*h*4);
            gl.glReadPixels(0, 0, w, h, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE, pixels);
            BufferedImage image = new BufferedImage(w, h, BufferedImage.TYPE_INT_ARGB);
            for (int y=0; y<h; y++) for (int x=0; x<w; x++) {
                int i=(y*w+x)*4;
                image.setRGB(x, h-y-1, 0xff000000 | (pixels.get(i)&255)<<16 | (pixels.get(i+1)&255)<<8 | (pixels.get(i+2)&255));
            }
            SwingUtilities.invokeLater(() -> capture.accept(image));
        }
    }
    private void compileLine(GL2 gl, TrajectoryData track) {
        if (trajectoryList != 0) gl.glDeleteLists(trajectoryList, 1);
        if (projectionList != 0) gl.glDeleteLists(projectionList, 1);
        trajectoryList = gl.glGenLists(1); projectionList = gl.glGenLists(1);
        // Display lists remove per-frame traversal; all recorded attitude samples remain in the adapter.
        for (boolean ground : new boolean[]{false, true}) {
            gl.glNewList(ground ? projectionList : trajectoryList, GL2.GL_COMPILE);
            gl.glBegin(GL.GL_LINE_STRIP);
            for (TrajectoryData.Sample sample : track.samples()) {
                if (sample.time() > track.end()) break;
                if (sample.position() == null || sample.breakBefore()) { gl.glEnd(); gl.glBegin(GL.GL_LINE_STRIP); }
                if (sample.position() != null) {
                    Coordinate p = sample.position(); gl.glVertex3d(p.x, p.y, ground ? 0 : p.z);
                }
            }
            gl.glEnd(); gl.glEndList();
        }
        compiled = track;
    }
    private static double niceStep(double value) {
        double base = Math.pow(10, Math.floor(Math.log10(value)));
        double n = value/base;
        return base*(n <= 1 ? 1 : n <= 2 ? 2 : n <= 5 ? 5 : 10);
    }
    private void grid(GL2 gl, TrajectoryData track) {
        double step = niceStep(track.span()/5), pad = track.span()*0.06;
        Coordinate min = track.minimum(), max = track.maximum();
        double x0 = Math.floor((min.x-pad)/step)*step, x1 = Math.ceil((max.x+pad)/step)*step;
        double y0 = Math.floor((min.y-pad)/step)*step, y1 = Math.ceil((max.y+pad)/step)*step;
        double z0 = Math.floor(min.z/step)*step, z1 = Math.ceil((max.z+pad)/step)*step;
        gl.glLineWidth(1); gl.glColor3d(0.5, 0.53, 0.56);
        gl.glBegin(GL.GL_LINES);
        for (double x=x0; x<=x1+step/10; x+=step) { gl.glVertex3d(x,y0,0); gl.glVertex3d(x,y1,0); }
        for (double y=y0; y<=y1+step/10; y+=step) { gl.glVertex3d(x0,y,0); gl.glVertex3d(x1,y,0); }
        gl.glEnd();
        var unit = UnitGroup.UNITS_DISTANCE.getDefaultUnit();
        for (double x=x0; x<=x1+step/10; x+=step) labels.add(new Label(unit.toString(x), new Coordinate(x,y0,0)));
        for (double y=y0; y<=y1+step/10; y+=step) labels.add(new Label(unit.toString(y), new Coordinate(x0,y,0)));
        for (double z=z0; z<=z1+step/10; z+=step) labels.add(new Label(unit.toString(z), new Coordinate(x0,y0,z)));
        gl.glBegin(GL.GL_LINES);
        gl.glColor3d(0.86,0.23,0.2); gl.glVertex3d(x0,y0,0); gl.glVertex3d(x1,y0,0);
        gl.glColor3d(0.15,0.65,0.3); gl.glVertex3d(x0,y0,0); gl.glVertex3d(x0,y1,0);
        gl.glColor3d(0.13,0.47,0.91); gl.glVertex3d(x0,y0,z0); gl.glVertex3d(x0,y0,z1);
        gl.glEnd();
        labels.add(new Label(Trajectory3DDialog.text("east")+" ("+unit.getUnit()+")", new Coordinate(x1+step*.35,y0,0)));
        labels.add(new Label(Trajectory3DDialog.text("north")+" ("+unit.getUnit()+")", new Coordinate(x0,y1+step*.35,0)));
        labels.add(new Label(Trajectory3DDialog.text("height")+" ("+unit.getUnit()+")", new Coordinate(x0,y0,z1+step*.25)));
    }
    private void drawLabels(GL2 gl, int width, int height) {
        gl.glGetDoublev(GL2.GL_MODELVIEW_MATRIX, model, 0); gl.glGetDoublev(GL2.GL_PROJECTION_MATRIX, projection, 0);
        gl.glGetIntegerv(GL.GL_VIEWPORT, viewport, 0);
        text.beginRendering(width,height);
        Color color = GUIUtil.getUITheme().getTextColor(); text.setColor(color);
        List<Rectangle> used = new ArrayList<>();
        for (Label label : labels) {
            Coordinate p=label.location(); double[] screen=new double[3];
            if (!glu.gluProject(p.x,p.y,p.z,model,0,projection,0,viewport,0,screen,0) || screen[2]<0 || screen[2]>1) continue;
            int x=(int)screen[0]+5, y=(int)screen[1]+5;
            Rectangle box=new Rectangle(x,y,(int)text.getBounds(label.text()).getWidth()+6,18);
            if (x<0 || y<0 || x+box.width>width || y+18>height || used.stream().anyMatch(r -> r.intersects(box))) continue;
            text.draw(label.text(),x,y); used.add(box);
        }
        text.endRendering();
    }
    private static void vertex(GL2 gl, Coordinate p) { gl.glVertex3d(p.x,p.y,p.z); }
    private static void line(GL2 gl, Coordinate a, Coordinate b) { vertex(gl,a); vertex(gl,b); }
    private void rocket(GL2 gl, Coordinate p, Quaternion q, double length) {
        Coordinate x=q.rotate(new Coordinate(1,0,0)), y=q.rotate(new Coordinate(0,1,0)), z=q.rotateZ();
        gl.glPushMatrix(); gl.glTranslated(p.x,p.y,p.z);
        gl.glMultMatrixd(new double[]{x.x,x.y,x.z,0,y.x,y.y,y.z,0,z.x,z.y,z.z,0,0,0,0,1},0);
        gl.glScaled(length,length,length);
        double r=.065;
        gl.glBegin(GL2.GL_QUADS);
        for(int i=0;i<16;i++) {
            double a=2*Math.PI*i/16,b=2*Math.PI*(i+1)/16;
            if (i<3) gl.glColor3d(.93,.23,.12); else gl.glColor3d(.88-.2*Math.sin(a),.9-.2*Math.sin(a),.94-.2*Math.sin(a));
            gl.glVertex3d(r*Math.cos(a),r*Math.sin(a),-.4); gl.glVertex3d(r*Math.cos(b),r*Math.sin(b),-.4);
            gl.glVertex3d(r*Math.cos(b),r*Math.sin(b),.25); gl.glVertex3d(r*Math.cos(a),r*Math.sin(a),.25);
        }
        gl.glEnd();
        gl.glBegin(GL.GL_TRIANGLES);
        for(int i=0;i<16;i++) {
            double a=2*Math.PI*i/16,b=2*Math.PI*(i+1)/16;
            gl.glColor3d(.3,.36,.45);
            gl.glVertex3d(r*Math.cos(a),r*Math.sin(a),.25); gl.glVertex3d(r*Math.cos(b),r*Math.sin(b),.25); gl.glVertex3d(0,0,.55);
        }
        for(int i=0;i<3;i++) {
            double a=2*Math.PI*i/3;
            if(i==0) gl.glColor3d(.94,.24,.13); else gl.glColor3d(.18,.35,.55);
            gl.glVertex3d(r*Math.cos(a),r*Math.sin(a),-.14);
            gl.glVertex3d(.23*Math.cos(a),.23*Math.sin(a),-.45);
            gl.glVertex3d(r*Math.cos(a),r*Math.sin(a),-.4);
        }
        gl.glEnd(); gl.glPopMatrix();
    }
    private void canopy(GL2 gl, Coordinate p, Quaternion q, double size) {
        Coordinate center=p.add(0,0,size*1.05);
        Coordinate attachment=q==null?p:p.add(q.rotateZ().multiply(size*.2));
        gl.glColor3d(.52,.56,.62); gl.glBegin(GL.GL_LINES);
        for(int i=0;i<8;i++) { double a=i*Math.PI/4; line(gl,attachment,center.add(size*.4*Math.cos(a),size*.4*Math.sin(a),0)); }
        gl.glEnd();
        for(int ring=0;ring<5;ring++) {
            double a0=ring*Math.PI/10,a1=(ring+1)*Math.PI/10;
            gl.glBegin(GL2.GL_QUAD_STRIP);
            for(int i=0;i<=24;i++) {
                double a=i*Math.PI/12;
                if ((i/3)%2==0) gl.glColor3d(.95,.4,.15); else gl.glColor3d(.95,.85,.65);
                for(double latitude:new double[]{a0,a1}) vertex(gl,center.add(size*.4*Math.cos(latitude)*Math.cos(a),size*.4*Math.cos(latitude)*Math.sin(a),size*.25*Math.sin(latitude)));
            }
            gl.glEnd();
        }
    }
    private void arrow(GL2 gl, Coordinate p, Coordinate vector, double head) {
        Coordinate tip=p.add(vector), unit=vector.normalize();
        Coordinate side=new Coordinate(-unit.y,unit.x,0);
        if(side.length()<1e-8) side=new Coordinate(1,0,0); else side=side.normalize();
        double h=Math.min(head,vector.length()*.25);
        Coordinate base=tip.sub(unit.multiply(h));
        gl.glColor3d(.97,.62,.12); gl.glLineWidth(3); gl.glBegin(GL.GL_LINES);
        line(gl,p,tip); line(gl,tip,base.add(side.multiply(h*.5))); line(gl,tip,base.sub(side.multiply(h*.5))); gl.glEnd();
    }
}
