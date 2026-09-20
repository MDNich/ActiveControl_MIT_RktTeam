package info.openrocket.swing.gui.plot;

import info.openrocket.core.simulation.*;
import info.openrocket.core.util.*;
import info.openrocket.swing.util.BaseTestCase;
import org.junit.jupiter.api.Test;
import java.util.concurrent.atomic.AtomicLong;
import static org.junit.jupiter.api.Assertions.*;
import static info.openrocket.core.simulation.FlightDataType.*;

class TrajectoryDataTest extends BaseTestCase {
    private void point(FlightDataBranch b,double time,Quaternion q) {
        b.addPoint(); b.setValue(TYPE_TIME,time); b.setValue(TYPE_POSITION_X,2*time); b.setValue(TYPE_POSITION_Y,time); b.setValue(TYPE_ALTITUDE,10*time);
        b.setValue(TYPE_VELOCITY_X,2); b.setValue(TYPE_VELOCITY_Y,1); b.setValue(TYPE_VELOCITY_Z,10);
        if(q!=null) { b.setValue(TYPE_ORIENTATION_QW,q.getW()); b.setValue(TYPE_ORIENTATION_QX,q.getX()); b.setValue(TYPE_ORIENTATION_QY,q.getY()); b.setValue(TYPE_ORIENTATION_QZ,q.getZ()); }
    }
    @Test void fullRollIsIndependentOfVelocityAndSlerpSign() {
        FlightDataBranch branch=new FlightDataBranch("roll",TYPE_TIME);
        point(branch,0,new Quaternion()); point(branch,2,Quaternion.rotation(new Coordinate(0,0,Math.PI/2)));
        TrajectoryData track=new TrajectoryData(branch); var frame=track.at(1);
        Coordinate bodyX=frame.attitude().rotate(new Coordinate(1,0,0));
        assertEquals(Math.sqrt(.5),bodyX.x,1e-12); assertEquals(Math.sqrt(.5),bodyX.y,1e-12);
        assertEquals(1,frame.attitude().rotateZ().z,1e-12); assertEquals(2,frame.velocity().x); assertEquals(10,frame.position().z);
        Quaternion q=new Quaternion(-1,0,0,0); assertEquals(1,TrajectoryData.slerp(new Quaternion(),q,.5).getW(),1e-12);
        assertEquals(1,TrajectoryData.slerp(new Quaternion(),Quaternion.rotation(new Coordinate(Math.PI,0,0)),.5).rotate(new Coordinate(0,1,0)).z,1e-12);
    }
    @Test void gapsDuplicatesLegacyAndRecoverySeeking() {
        FlightDataBranch branch=new FlightDataBranch("legacy",TYPE_TIME);
        point(branch,0,null); point(branch,1,null); point(branch,1,null); point(branch,2,null); point(branch,3,null);
        branch.setValue(TYPE_POSITION_X,Double.NaN); point(branch,4,null);
        branch.addEvent(new FlightEvent(FlightEvent.Type.RECOVERY_DEVICE_DEPLOYMENT,2,null));
        TrajectoryData track=new TrajectoryData(branch);
        assertEquals(5,track.samples().size()); assertNull(track.at(.5).attitude());
        assertFalse(track.at(1).recovery()); assertTrue(track.at(2).recovery()); assertFalse(track.at(0).recovery());
        assertTrue(track.at(2).held()); assertNull(track.at(2.5).position()); assertNull(track.at(3.5).position()); assertNotNull(track.at(4).position());
    }
    @Test void undersampledRotationAndBackwardsTimeAreExplicit() {
        FlightDataBranch branch=new FlightDataBranch("fast",TYPE_TIME);
        for(int i=0;i<2;i++) { point(branch,i,new Quaternion()); branch.setValue(TYPE_ROLL_RATE,10); branch.setValue(TYPE_PITCH_RATE,0); branch.setValue(TYPE_YAW_RATE,0); }
        var frame=new TrajectoryData(branch).at(.5); assertTrue(frame.undersampled()); assertNull(frame.attitude());
        point(branch,.5,new Quaternion()); assertThrows(IllegalArgumentException.class,()->new TrajectoryData(branch));
    }
    @Test void playbackUsesElapsedTimeAndPreservesPauseAndSeek() {
        AtomicLong nanos=new AtomicLong(); TrajectoryPlayback clock=new TrajectoryPlayback(nanos::get); clock.range(0,10); clock.setPlaying(true);
        nanos.set(4_000_000_000L); assertEquals(1,clock.time()); clock.setPlaying(false);
        nanos.set(9_000_000_000L); assertEquals(1,clock.time()); clock.setSpeed(2); clock.seek(3); clock.setPlaying(true);
        nanos.set(10_000_000_000L); assertEquals(5,clock.time()); nanos.set(20_000_000_000L); assertEquals(10,clock.time()); assertFalse(clock.isPlaying());
        assertThrows(IllegalArgumentException.class,()->clock.setSpeed(Double.NaN));
    }
}
