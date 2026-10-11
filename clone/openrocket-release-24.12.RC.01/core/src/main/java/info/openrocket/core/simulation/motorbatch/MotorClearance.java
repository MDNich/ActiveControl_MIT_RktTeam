package info.openrocket.core.simulation.motorbatch;

import java.util.*;
import java.awt.geom.*;
import info.openrocket.core.motor.ThrustCurveMotor;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.util.*;

/** Installed motor envelopes versus the active, instance-aware component geometry. */
public final class MotorClearance {
    private static final double EPS=1e-6; // Surface contact is permitted; penetration beyond 1 micrometre is not.
    private MotorClearance() { }
    private record Cylinder(double start,double end,double y,double z,double radius) { }
    public static List<String> conflicts(FlightConfiguration config,MotorMount mount,ThrustCurveMotor motor) {
        var instances=config.getActiveInstances();var targets=instances.get((RocketComponent)mount);
        if(targets==null||targets.isEmpty())return List.of("CLEARANCE_UNKNOWN: motor mount has no active instances");
        var motors=new java.util.ArrayList<Cylinder>();
        double start=mount.getLength()+mount.getMotorOverhang()-motor.getLength();
        for(var target:targets) {
            var p=target.transform.transform(new Coordinate(start,0,0));
            motors.add(new Cylinder(p.x,p.x+motor.getLength(),p.y,p.z,motor.getDiameter()/2));
        }
        var reasons=new TreeSet<String>();
        for(Cylinder envelope:motors) {
            for(var entry:instances.entrySet()) {
                RocketComponent component=entry.getKey();
                if(component instanceof ComponentAssembly)continue;
                for(var context:entry.getValue()) {
                    if(intersects(envelope,context)) {
                        boolean envelopeOnly=!(component instanceof SymmetricComponent || component instanceof Coaxial || component instanceof MassObject || component instanceof FinSet)
                                || component instanceof FinSet f && Math.abs(f.getCantAngle())>EPS;
                        reasons.add((envelopeOnly?"CLEARANCE_REVIEW_ENVELOPE:":"MOTOR_INTERSECTS:")+component.getName()+" ["+component.getID()+"]");
                    }
                }
            }
            // Other fixed motors are solids even though their enclosing tubes have open bores.
            for(var other:config.getAllMotors())if(other.getMotor()!=null && other.getMount()!=mount) {
                var contexts=instances.get((RocketComponent)other.getMount());if(contexts==null)continue;
                for(var c:contexts) {
                    var p=c.transform.transform(new Coordinate(other.getMount().getLength()+other.getMount().getMotorOverhang()-other.getMotor().getLength(),0,0));
                    if(annulus(envelope,p,0,other.getMotor().getLength(),0,other.getMotor().getDiameter()/2))
                        reasons.add("MOTOR_INTERSECTS:fixed motor in "+((RocketComponent)other.getMount()).getName());
                }
            }
        }
        for(int i=0;i<motors.size();i++)for(int j=0;j<i;j++) {
            var a=motors.get(i);var b=motors.get(j);
            if(overlap(a.start,a.end,b.start,b.end)&&Math.hypot(a.y-b.y,a.z-b.z)<a.radius+b.radius-EPS)
                reasons.add("MOTOR_INTERSECTS:another motor in the selected cluster");
        }
        return List.copyOf(reasons);
    }
    private static boolean overlap(double a,double b,double c,double d){return Math.min(b,d)-Math.max(a,c)>EPS;}
    private static boolean annulus(Cylinder m,Coordinate p,double start,double end,double inner,double outer) {
        if(!overlap(m.start,m.end,p.x+start,p.x+end) || outer<=inner+EPS)return false;
        double d=Math.hypot(m.y-p.y,m.z-p.z);
        return d+m.radius>inner+EPS && Math.max(0,d-m.radius)<outer-EPS;
    }
    private static boolean intersects(Cylinder m,InstanceContext c) {
        RocketComponent part=c.component;Coordinate p=c.getLocation();
        // MassObject's radial displacement is stored in its CG rather than instance offsets.
        if(part instanceof MassObject mass)p=c.transform.transform(new Coordinate(0,
                mass.getRadialPosition()*Math.cos(mass.getRadialDirection()),mass.getRadialPosition()*Math.sin(mass.getRadialDirection())));
        // InnerTube includes its displacement in cluster instances; the other ring types do not.
        else if(part instanceof RingComponent ring && !(part instanceof InnerTube))p=c.transform.transform(new Coordinate(0,
                ring.getRadialPosition()*Math.cos(ring.getRadialDirection()),ring.getRadialPosition()*Math.sin(ring.getRadialDirection())));
        if(part instanceof Transition t) {
            if(annulus(m,p,-t.getForeShoulderLength(),0,Math.max(0,t.getForeShoulderRadius()-t.getForeShoulderThickness()),t.getForeShoulderRadius()) ||
               annulus(m,p,t.getLength(),t.getLength()+t.getAftShoulderLength(),Math.max(0,t.getAftShoulderRadius()-t.getAftShoulderThickness()),t.getAftShoulderRadius()))return true;
            if(t.isForeShoulderCapped()&&annulus(m,p,-t.getForeShoulderLength(),-t.getForeShoulderLength()+t.getForeShoulderThickness(),0,t.getForeShoulderRadius()))return true;
            if(t.isAftShoulderCapped()&&annulus(m,p,t.getLength()+t.getAftShoulderLength()-t.getAftShoulderThickness(),t.getLength()+t.getAftShoulderLength(),0,t.getAftShoulderRadius()))return true;
        }
        if(part instanceof SymmetricComponent body) {
            double lo=Math.max(0,m.start-p.x),hi=Math.min(body.getLength(),m.end-p.x);
            if(hi-lo<=EPS)return false;
            // These supported OR profiles are monotonic in radius. Sweeping their radial interval
            // also covers offset motors crossing either side of a hollow shell.
            double outer=Math.max(body.getRadius(lo),body.getRadius(hi));
            double inner=body.isFilled()?0:Math.min(body.getInnerRadius(lo),body.getInnerRadius(hi));
            return annulus(m,p,lo,hi,inner,outer);
        }
        if(part instanceof Coaxial tube)return annulus(m,p,0,part.getLength(),tube.getInnerRadius(),tube.getOuterRadius());
        if(part instanceof MassObject mass)return annulus(m,p,0,mass.getLength(),0,mass.getRadius());
        if(part instanceof FinSet fins) {
            var points=new java.util.ArrayList<Coordinate>(Arrays.asList(fins.getFinPoints()));
            if(fins.getTabHeight()>0&&fins.getTabLength()>0)points.addAll(Arrays.asList(fins.getTabPoints()));
            double minX=points.stream().mapToDouble(v->v.x).min().orElse(0),maxX=points.stream().mapToDouble(v->v.x).max().orElse(0);
            if(!overlap(m.start,m.end,p.x+minX,p.x+maxX))return false;
            var ey=c.transform.transform(new Coordinate(0,1,0)).sub(p);var ez=c.transform.transform(new Coordinate(0,0,1)).sub(p);
            double y=(m.y-p.y)*ey.y+(m.z-p.z)*ey.z,z=(m.y-p.y)*ez.y+(m.z-p.z)*ez.z;
            // Conservative thickness envelope for a canted fin; root/tab geometry is included.
            double half=fins.getThickness()/2+Math.abs(Math.sin(fins.getCantAngle()))*(maxX-minX);
            double dz=Math.max(0,Math.abs(z)-half);
            if(dz>=m.radius-EPS)return false;
            double dy=Math.sqrt(m.radius*m.radius-dz*dz)-EPS;
            var rectangle=new Rectangle2D.Double(m.start-p.x+EPS,y-dy,m.end-m.start-2*EPS,2*dy);
            return polygon(fins.getFinPoints()).intersects(rectangle) ||
                    fins.getTabHeight()>0&&fins.getTabLength()>0&&polygon(fins.getTabPoints()).intersects(rectangle);
        }
        // Conservatively screen other modeled parts using all eight transformed box corners.
        // Do not use BoundingBox.transform(), which transforms only two opposite corners.
        BoundingBox local=new BoundingBox();
        Transformation transform=c.transform;
        if(part instanceof RailButton button) {
            // OR's general selection bounds extend inward as well as outward. The modeled
            // button (including its screw head) is entirely outside its mounting surface.
            double r=button.getOuterDiameter()/2;
            local.update(new Coordinate(-r,0,-r));
            local.update(new Coordinate(r,button.getTotalHeight()+button.getScrewHeight(),r));
            transform=transform.applyTransformation(Transformation.getAxialRotation(button.getAngleOffset()));
            Coordinate outward=transform.transform(new Coordinate(0,1,0)).sub(p);
            if((m.y-p.y)*outward.y+(m.z-p.z)*outward.z+m.radius<=EPS)return false;
        } else if(part instanceof BoxBounded box) {
            var b=box.getInstanceBoundingBox();local.update(b.min);local.update(b.max);
        } else for(var point:part.getComponentBounds())local.update(point);
        if(local.isEmpty())return false;
        var world=new BoundingBox();
        for(double x:new double[]{local.min.x,local.max.x})for(double y:new double[]{local.min.y,local.max.y})for(double z:new double[]{local.min.z,local.max.z})
            world.update(transform.transform(new Coordinate(x,y,z)));
        if(!overlap(m.start,m.end,world.min.x,world.max.x))return false;
        double dy=Math.max(0,Math.max(world.min.y-m.y,m.y-world.max.y)),dz=Math.max(0,Math.max(world.min.z-m.z,m.z-world.max.z));
        return Math.hypot(dy,dz)<m.radius-EPS;
    }
    private static Path2D polygon(Coordinate[] points) {
        var p=new Path2D.Double();if(points.length==0)return p;
        p.moveTo(points[0].x,points[0].y);for(int i=1;i<points.length;i++)p.lineTo(points[i].x,points[i].y);p.closePath();return p;
    }
}
