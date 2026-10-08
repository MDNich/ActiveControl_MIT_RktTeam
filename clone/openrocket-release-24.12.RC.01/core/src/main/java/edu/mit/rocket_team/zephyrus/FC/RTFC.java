package edu.mit.rocket_team.zephyrus.FC;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Objects;
import info.openrocket.core.simulation.flightcomputer.*;
import jakarta.json.JsonObject;
import edu.mit.rocket_team.zephyrus.control.*;
import edu.mit.rocket_team.zephyrus.control.airbrakes.RTAirbrakesController;
import edu.mit.rocket_team.zephyrus.instrument.*;
import edu.mit.rocket_team.zephyrus.internal.*;
import edu.mit.rocket_team.zephyrus.telemetry.RTTelemetryEngine;
import edu.mit.rocket_team.zephyrus.telemetry.TelemetryLinkSettings;
import edu.mit.rocket_team.zephyrus.AV.*;
import edu.mit.rocket_team.zephyrus.util.*;
import edu.mit.rocket_team.zephyrus.util.data.*;
import static edu.mit.rocket_team.zephyrus.util.RTRocketState.*;
import static edu.mit.rocket_team.zephyrus.util.RTUtilLibrary.*;

/** FC/FC.ino translated in place. No simulator truth enters the control path. */
public class RTFC {
    public static final long LOOP_US=10_000, PWM_US=20_000;
    public static final float CLOSED_ANGLE=-67.0f, OPEN_ANGLE=-117.0f;
    public final Trace trace;
    public final RTAccel accel;
    public final RTBaro baro;
    public final RTGPS gps;
    public final RTGyro gyro;
    public final RTRollController rollController;
    public final RTAirbrakesController airbrakesController;
    public final RTPyroController pyroController;
    public final RTPowerBoard powerBoard;
    public final RTFlashDriver flashDriver;
    public final RTTelemetryEngine telemetry;
    private final RTCamera cam0,cam1;
    private final RTVTX vtx;
    private RTRocketState currentState=GROUND_TESTING,recState=GROUND_TESTING;
    private Inputs pendingInput;
    private FlightComputerStateMachine stateMachine;
    private JavaBoardRuntime boards;
    private long lastSensorAcquisitionUs,lastBoardCommandUs=-1,lastBoardSampleUs;
    public long getNextBoardDeadlineUs(){return boards==null?Long.MAX_VALUE:boards.nextDeadlineUs();}
    public long getLastBoardCommandUs(){return lastBoardCommandUs;}
    public long getLastBoardSampleUs(){return lastBoardSampleUs;}
    public void dispatchBoards(long nowUs){
        if(boards==null)return;
        boards.dispatch(nowUs,lastSensorAcquisitionUs,getDesignStateId(),key->switch(key){
            case "flight_ms"->elapsed32(trace.millis(),flightBeginTime);
            case "apogee_ms"->elapsed32(trace.millis(),apogeeTime);
            default->signal(key);
        },this::connected,(board,port,value,sampleUs)->{
            boardOutput(port,value);
            if(connected(port)&&!port.equals("recovery")){lastBoardCommandUs=trace.bootUs();lastBoardSampleUs=sampleUs;}
        },message->trace.log("java.board",message));
    }
    public String getDesignStateId(){return stateMachine==null?(currentState.ID<5?FlightComputerStateMachine.PHASES.get(currentState.ID):"end"):stateMachine.id();}
    public boolean connected(String port){return design==null||design.active(port)!=null;}
    private void boardOutput(String port,Double value){
        if(!connected(port)){trace.log("output.discard","port="+port+" value="+value+" reason=no_component");return;}
        switch(port){
            case "airbrakes":airbrakesEnabled=false;airbrakesSetAngle=dpToDeg((float)Math.min(value,setting("airbrakeLimit",1)));break;
            case "roll":rollControlEnabled=false;rollControlSetAngle=value.floatValue();rollSignalEnabled=true;break;
            case "recovery":fireChannels(value.intValue());break;
            default:throw new IllegalArgumentException("Unknown output "+port);
        }
        trace.log("output.command","port="+port+" value="+value);
    }
    private void entryAction(JsonObject action){
        trace.log("state.action",action.toString());
        switch(action.getString("type")){
            case "prepare":prepareFlight();break;case "launch":beginFlight();break;
            case "apogee":detectApogee();break;case "main":deployMain();break;case "finish":finishFlight();break;
            case "fire_recovery":fireChannels(action.getInt("channel"));break;
            case "close_controllers":closeControllers();break;
            case "airbrakes":case "roll":boardOutput(action.getString("type"),action.getJsonNumber("value").doubleValue());break;
            case "log":trace.log("state.message",action.getString("message"));break;
            default:throw new IllegalArgumentException("Unknown state action");
        }
    }
    private info.openrocket.core.simulation.flightcomputer.FlightComputerDesign design;
    private info.openrocket.core.simulation.flightcomputer.FlightComputerSensors sensors;
    public void configure(info.openrocket.core.simulation.flightcomputer.FlightComputerDesign value) {
        if(initialized) throw new IllegalStateException("Configure before FC setup");
        value.requireRunnable(); design=value; sensors=new info.openrocket.core.simulation.flightcomputer.FlightComputerSensors(value);
        baro.setFilterSamples((int)value.active("barometer").getJsonObject("properties").getJsonNumber("filterSamples").doubleValue());
        airbrakesController.setPidGains((float)value.parameter("airbrakeKp"),(float)value.parameter("airbrakeKi"),(float)value.parameter("airbrakeKd"));
        rollController.setPidGains((float)value.parameter("rollKp"),(float)value.parameter("rollKi"),(float)value.parameter("rollKd"));
        boards=new JavaBoardRuntime(value);
        if(value.hasStateMachine())stateMachine=new FlightComputerStateMachine(value.stateMachine(),new FlightComputerStateMachine.Host(){
            public void enter(String id,String name,String phase){
                currentState=switch(phase){case "ground"->GROUND_TESTING;case "preflight"->PRE_FLIGHT;case "flight"->FLIGHT;case "apogee"->APOGEE;case "main"->MAIN;default->throw new IllegalArgumentException(phase);};
                trace.log("state.enter","id="+id+" name="+name+" phase="+phase);
            }
            public void action(JsonObject action){entryAction(action);}
            public void transition(String from,String to){trace.log("fc.transition","from="+from+" to="+to+" phase="+currentState);}
        });
    }
    private double setting(String key,double fallback){return design==null?fallback:design.parameter(key);}
    private boolean rule(String id,boolean fallback){return design==null?fallback:design.test(id,this::signal);}
    private double signal(String name){return switch(name){
        case "command"->recState.ID; case "flight_ms"->elapsed32(FCtime,flightBeginTime);case "apogee_ms"->elapsed32(FCtime,apogeeTime);
        case "accel_vertical"->accel.getVerticalAccelMinusGravity();case "baro_drop"->baro.getMaxAlt()-baro.getFilteredAltitude();
        case "gps_drop"->gps.getMaxAlt()-gps.getHeight();case "gps_fix"->gps.getFixType();case "baro_altitude"->baro.getFilteredAltitude();
        case "gps_altitude"->gps.getHeight();case "state_ms"->stateMachine==null?0:stateMachine.stateAgeMs(trace.bootUs());case "time_ms"->trace.bootUs()/1000;
        case "velocity"->accel.getIntegratedVelo();case "roll_deg"->gyro.getRoll();case "pitch_deg"->gyro.getPitch();case "yaw_deg"->gyro.getYaw();default->throw new IllegalArgumentException(name);};}
    private int loopPhase;
    private boolean initialized,prepared,airbrakesEnabled,rollControlEnabled,loggingEnabled;
    private boolean bpFired1,bpFired2,baroMaxAltReset,rollSignalEnabled=true;
    private float airbrakesSetAngle=CLOSED_ANGLE,rollControlSetAngle;
    private long FCtime,flightBeginTime,apogeeTime,lastTelem,lastPowerPkt,lastLoopUs=-1;
    private long loopCount,airbrakeUpdateCount,rollUpdateCount;
    private int packetNum,badPackets;
    private Output output=new Output(0,CLOSED_ANGLE,degToUsAirbrakes(CLOSED_ANGLE),0,0,1495,1430);

    /** Null accel/GPS means no new sample. Barometer/gyro retain last delivered readings. */
    public record Inputs(long acquisitionUs, RTAccelData accel, RTBaroData baro, RTGPSData gps, RTGyroData gyro) {}
    public record Output(float requestedDeployment,float selectedAngle,int airbrakePulseUs,
                         float exposedFraction,float rollAngle,int servo2Us,int servo3Us) {}
    public RTFC() { this(new Trace()); }
    public RTFC(Trace trace) { this(trace, TelemetryLinkSettings.DEFAULT); }
    public RTFC(Trace trace, TelemetryLinkSettings link) {
        this.trace=Objects.requireNonNull(trace);
        accel=new RTAccel(trace); baro=new RTBaro(trace); gps=new RTGPS(trace); gyro=new RTGyro(trace);
        rollController=new RTRollController(trace); airbrakesController=new RTAirbrakesController(trace);
        pyroController=new RTPyroController(trace); powerBoard=new RTPowerBoard(trace); flashDriver=new RTFlashDriver(trace);
        telemetry=new RTTelemetryEngine(trace, link); cam0=new RTCamera(trace); cam1=new RTCamera(trace); vtx=new RTVTX(trace);
    }
    /** FC.ino setup; fresh objects reproduce the firmware globals' startup initialization. */
    public void init() {
        if(initialized) throw new IllegalStateException("Create a fresh RTFC for a new boot");
        initialized=true;
        trace.log("fc.setup", "source=FC.ino sketch=4cd2660 library=1db1f22 airbrake_time=integer_seconds pyro=recorded_only");
        if(getNextBoardDeadlineUs()!=Long.MAX_VALUE)trace.log("java.board_model",JavaBoardProgram.MODEL_NOTICE);
        pyroController.setup(); latchPwm(); telemetry.setup(); accel.setup(); baro.setup(); gyro.setup(); flashDriver.setup();
        cam0.setup(); cam1.setup(); vtx.setup(); gps.setup(); powerBoard.setup(); airbrakesController.setup(); rollController.setup();
        if(stateMachine!=null)stateMachine.start(trace.bootUs());
    }
    public void pre_loop(long bootUs,Inputs input) {
        if(!initialized) throw new IllegalStateException("Call init first");
        if(bootUs<=lastLoopUs) throw new IllegalArgumentException("Duplicate/backwards FC tick");
        if(input.acquisitionUs()>bootUs) throw new IllegalArgumentException("Cannot deliver a future sample");
        if (prepared || loopPhase != 0) throw new IllegalStateException("Previous FC iteration still active");
        trace.time(bootUs);
        pendingInput=sensors==null?input:sensors.sample(bootUs,input);
        prepared=true;
    }
    /** Immediate execution remains available for firmware semantic/replay tests. */
    public void loop() { beginLoop(); readSensors(); completeLoop(); }

    /** The existing listener schedules these three phases in virtual time. */
    public void beginLoop() {
        if(!prepared) throw new IllegalStateException("Call pre_loop once per tick");
        prepared=false; loopPhase=1; lastLoopUs=trace.bootUs(); loopCount++;
        trace.log("fc.loop_begin", "count="+loopCount+" state="+currentState);
    }
    public void readSensors() {
        if(loopPhase != 1) throw new IllegalStateException("FC sensors out of order");
        Inputs input=pendingInput;
        if(input.accel()!=null) accel.backdoorFudge(input.accel());
        if(input.baro()!=null) baro.backdoorFudge(input.baro());
        if(input.gps()!=null) gps.backdoorFudge(input.gps());
        if(input.gyro()!=null) gyro.backdoorFudge(input.gyro());
        trace.log("sensors.deliver", "acquisition_us="+input.acquisitionUs()+" age_us="+(trace.bootUs()-input.acquisitionUs())+" accel_new="+(input.accel()!=null)+" gps_new="+(input.gps()!=null));
        baro.updateAll(); accel.update(currentState); gyro.update(); gps.updateAndParse(); pyroController.pyroMonitor(currentState); powerBoard.update();
        lastSensorAcquisitionUs=input.acquisitionUs();
        loopPhase=2;
    }
    public void completeLoop() {
        if(loopPhase != 2) throw new IllegalStateException("FC completion out of order");
        FCtime=trace.millis();
        if(stateMachine==null)handleState();else{
            if(currentState==FLIGHT&&elapsed32(FCtime,flightBeginTime)>setting("apogeeLockoutMs",26000)&&!baroMaxAltReset){baro.resetMaxAlt();baroMaxAltReset=true;}
            if(currentState==APOGEE&&design.stateMachine().getBoolean("automaticRecovery",true))delayedRecovery();
            stateMachine.step(trace.bootUs(),this::signal,setting("apogeeLockoutMs",26000));
        }
        if(rollControlEnabled) updateRollControl();
        if(airbrakesEnabled) updateAirbrakes();
        if(elapsed32(trace.millis(),lastTelem)>setting("telemetryIntervalMs",50)) {
            lastTelem=trace.millis(); byte[] packet=constructTelemetryPacket(); telemetry.send(packet);
            if(loggingEnabled) flashDriver.append(packet);
        }
        readTelem();
        // FC.ino advances the timer before sending; retain its strict >100 ms comparison.
        if(elapsed32(trace.millis(),lastPowerPkt)>setting("powerIntervalMs",100)) {
            lastPowerPkt=trace.millis();
            powerBoard.sendCommand();
        }
        telemetry.deliverDue(trace.bootUs());
        loopPhase=0; pendingInput=null;
        trace.log("fc.loop_end", "state="+currentState+" flight_ms="+elapsed32(FCtime,flightBeginTime)+" airbrakes_enabled="+airbrakesEnabled+" roll_enabled="+rollControlEnabled);
    }

    /** FC.ino handleState; repeated preflight is deliberately not a complete reset. */
    private void handleState() {
        RTRocketState previous=currentState;
        trace.log("fc.handle_state", "state="+currentState+" received="+recState);
        switch(currentState) {
            case GROUND_TESTING:
                if(rule("preflight",recState==PRE_FLIGHT)) {
                    currentState=PRE_FLIGHT; prepareFlight();
                } break;
            case PRE_FLIGHT:
                if(rule("launch",recState==FLIGHT || accel.getVerticalAccelMinusGravity()>30)) {
                    currentState=FLIGHT; beginFlight();
                } break;
            case FLIGHT:
                if(elapsed32(FCtime,flightBeginTime)>setting("apogeeLockoutMs",26000)) {
                    if(!baroMaxAltReset) { baro.resetMaxAlt(); baroMaxAltReset=true; }
                    if(rule("apogee",recState==APOGEE || (gps.getFixType()==3 && gps.getMaxAlt()>gps.getHeight()+20)
                            || baro.getMaxAlt()>baro.getFilteredAltitude()+20)) currentState=APOGEE;
                }
                if(rule("apogeeOverride",elapsed32(FCtime,flightBeginTime)>35000)) currentState=APOGEE;
                if(currentState==APOGEE) detectApogee();
                break;
            case APOGEE:
                delayedRecovery();
                if(rule("main",elapsed32(FCtime,apogeeTime)>55000 && (recState==MAIN || (gps.getFixType()==3 && gps.getHeight()<457) || baro.getFilteredAltitude()<457))) {
                    currentState=MAIN; deployMain();
                } break;
            case MAIN:
                if(rule("end",recState==END)) { currentState=GROUND_TESTING; finishFlight(); }
                break;
            default: break;
        }
        if(previous!=currentState) trace.log("fc.transition", "from="+previous+" to="+currentState+" flight_begin_ms="+flightBeginTime+" apogee_ms="+apogeeTime);
    }
    private void prepareFlight(){
        flashDriver.allocate();loggingEnabled=true;
        gyro.update();gyro.zeroRollPitchYaw();accel.update(currentState);accel.zeroIntegratedVelo();gps.zeroAlt();baro.zeroAlt();
        closeControllers();powerBoard.enableAll();trace.log("logging.enable","enabled=true");
    }
    private void beginFlight(){
        flightBeginTime=trace.millis();FCtime=trace.millis();gyro.zeroRollPitchYaw();
        airbrakesController.resetPidHistory();rollController.resetPidHistory();
        updateAirbrakes();updateRollControl();airbrakesEnabled=rollControlEnabled=true;vtx.setPower(3);
        trace.log("controllers.enable","airbrakes=true roll=true");
    }
    private void detectApogee(){apogeeTime=trace.millis();FCtime=trace.millis();fireChannels(0,1);closeControllers();}
    private void delayedRecovery(){
        if(elapsed32(FCtime,apogeeTime)>setting("recovery1Ms",3000)&&!bpFired1){fireChannels(3,4);bpFired1=true;}
        if(elapsed32(FCtime,apogeeTime)>setting("recovery2Ms",5000)&&!bpFired2){fireChannels(5);bpFired2=true;}
    }
    private void deployMain(){fireChannels(2);rollSignalEnabled=false;powerBoard.setConverter(4,false);trace.log("roll.disable_signal","servo2=false servo3=false");}
    private void finishFlight(){loggingEnabled=false;trace.log("logging.enable","enabled=false");}
    private void closeControllers() {
        rollControlSetAngle=0; airbrakesSetAngle=CLOSED_ANGLE; airbrakesEnabled=rollControlEnabled=false;
        trace.log("controllers.close", "airbrakes_angle="+airbrakesSetAngle+" roll_angle=0");
    }
    private void fireChannels(int... channels) { if(!connected("recovery")){trace.log("output.discard","port=recovery reason=no_component");return;} for(int c:channels) { pyroController.armPyro(c); pyroController.firePyro(c); } }
    private void updateAirbrakes() {
        airbrakeUpdateCount++;
        float t=elapsed32(FCtime,flightBeginTime)/1000; // Preserve the FC's integer division.
        airbrakesController.update(t,new RTFudgedAirbrakesData(baro.getFilteredAltitude(),accel.getIntegratedVelo(),accel.getAccelZ(),currentState.ID>FLIGHT.ID));
    }
    private void updateRollControl() {
        rollUpdateCount++;
        rollController.update((float)(elapsed32(FCtime,flightBeginTime)/1000.0),baro.getFilteredAltitude(),accel.getIntegratedVelo(),gyro.getRoll(),gyro.getRollRate());
    }
    public static float dpToDeg(float dp) { return (OPEN_ANGLE-CLOSED_ANGLE)*dp+CLOSED_ANGLE; }
    public static int degToUsAirbrakes(float degrees) { return ((int)(1500.0+(degrees/60.0)*500.0))&0xffff; }
    public static int degToUsRollControl(float degrees) { return ((int)(1500.0+(degrees/50.0)*500.0))&0xffff; }
    /** FC Update_IT_callback; source integer PWM with an ideal, endpoint-calibrated linear linkage. */
    public void latchPwm() {
        float requested=airbrakesController.getDeployment();
        if(design!=null)requested=(float)Math.min(requested,setting("airbrakeLimit",1));
        float angle=airbrakesEnabled?dpToDeg(requested):airbrakesSetAngle;
        float roll=rollControlEnabled?rollController.getAngle():rollControlSetAngle;
        if(!Float.isFinite(angle) || !Float.isFinite(roll)) { trace.log("fc.error", "reason=nonfinite_actuator"); throw new IllegalStateException("Nonfinite FC actuator output at boot_us="+trace.bootUs()); }
        if(!connected("airbrakes")){trace.log("output.discard","port=airbrakes reason=no_component requested="+requested+" angle="+angle);requested=0;angle=CLOSED_ANGLE;}
        if(!connected("roll")){trace.log("output.discard","port=roll reason=no_component angle="+roll);roll=0;}
        int pulse=degToUsAirbrakes(angle);
        float fraction=(float)(degToUsAirbrakes(CLOSED_ANGLE)-pulse)/(degToUsAirbrakes(CLOSED_ANGLE)-degToUsAirbrakes(OPEN_ANGLE));
        fraction=Math.max(0,Math.min(1,fraction));
        output=new Output(requested,angle,connected("airbrakes")?pulse:0,fraction,roll,rollSignalEnabled&&connected("roll")?degToUsRollControl(roll-0.5f):0,rollSignalEnabled&&connected("roll")?degToUsRollControl(roll-7.0f):0);
        trace.log("pwm.latch", "requested="+requested+" enabled="+airbrakesEnabled+" angle_deg="+angle+" pulse_us="+pulse+" exposed="+fraction+" roll_deg="+roll+" servo2_us="+output.servo2Us()+" servo3_us="+output.servo3Us());
    }
    public Output getOutput() { return output; }
    public RTRocketState getState() { return currentState; }
    public long getFlightBeginTime() { return flightBeginTime; }
    public long getApogeeTime() { return apogeeTime; }
    public long getLoopCount() { return loopCount; }
    public long getAirbrakeUpdateCount() { return airbrakeUpdateCount; }
    public long getRollUpdateCount() { return rollUpdateCount; }
    public boolean isAirbrakesEnabled() { return airbrakesEnabled; }
    public int getBadPackets() { return badPackets; }
    public void enqueueCommand(byte[] packet) { telemetry.enqueue(packet); }
    /** Test/scenario convenience: still traverses the FC's real byte decoder and loop ordering. */
    public static byte[] stateCommand(RTRocketState state) { return command(2,state.ID); }
    public static byte[] command(int opcode,int value) {
        byte[] p=new byte[16]; p[0]=(byte)0xaa; p[12]=(byte)value; p[13]=(byte)opcode; checksumCommand(p); return p;
    }
    public static byte[] angleCommand(int opcode,float value) {
        byte[] p=command(opcode,0); ByteBuffer.wrap(p).order(ByteOrder.LITTLE_ENDIAN).putFloat(9,value); checksumCommand(p); return p;
    }
    public static void checksumCommand(byte[] p) { int sum=0; for(int i=1;i<14;i++) sum+=p[i]&255; p[14]=(byte)(sum>>8); p[15]=(byte)sum; }
    /** FC readTelem: process one received packet at the end of a loop. */
    private void readTelem() {
        byte[] p=telemetry.receive(); if(p==null) return;
        if(p.length!=16 || (p[0]&255)!=0xaa) { reject("length_or_header"); return; }
        int sum=0; for(int i=1;i<14;i++) sum+=p[i]&255;
        if(sum!=((p[14]&255)<<8)+(p[15]&255)) { reject("checksum"); return; }
        int op=p[13]&255,value=p[12]&255;
        int lastReserved=(op==3 || op==5)?8:((op==1 || op==2 || op==6 || op==0x14 || op==0x15 || op==0x16)?11:12);
        for(int i=1;i<=lastReserved;i++) if(p[i]!=0) { trace.log("command.ignore", "reason=reserved_bytes opcode="+op); return; }
        boolean ground=currentState==GROUND_TESTING;
        if((op==1 || op==6) && (value&0xc0)!=0) { reject("pyro_mask_out_of_range"); return; }
        if(op==2 && value>=RTRocketState.values().length) { reject("invalid_state"); return; }
        trace.log("command.decode", "opcode="+op+" value="+value+" ground="+ground);
        if((op==1||op==6)&&!connected("recovery")){trace.log("output.discard","port=recovery reason=no_component opcode="+op);return;}
        switch(op) {
            case 1: for(int i=0;i<6;i++) if((value&(1<<i))!=0) pyroController.armPyro(i); break;
            case 2: recState=RTRocketState.values()[value]; trace.log("command.state", "received="+recState); break;
            case 6: for(int i=0;i<6;i++) if((value&(1<<i))!=0) pyroController.firePyro(i); break;
            case 3: case 5:
                if(ground) {
                    float a=ByteBuffer.wrap(p).order(ByteOrder.LITTLE_ENDIAN).getFloat(9);
                    if(!Float.isFinite(a) || a < -180 || a > 180) { reject("invalid_manual_angle"); break; }
                    if(op==3) { airbrakesSetAngle=a; airbrakesEnabled=false; } else { rollControlSetAngle=a; rollControlEnabled=false; }
                    trace.log("command.manual_servo", "opcode="+op+" degrees="+a);
                } break;
            case 4: if(ground) { updateRollControl(); rollControlEnabled=true; } break;
            case 8: if(ground) closeControllers(); break;
            case 9: if(ground) gyro.zeroRollPitchYaw(); break;
            case 10: if(ground) { baro.zeroAlt(); gps.zeroAlt(); } break;
            case 11: if(ground) accel.zeroIntegratedVelo(); break;
            case 0x10: fireChannels(0,1); break;
            case 0x11: fireChannels(3,4); break;
            case 0x12: fireChannels(2); break;
            case 0x13: fireChannels(0,1,2,3,4); break; // Source excludes channel 5 here.
            case 0x14: if(ground) vtx.setPower(Math.min(3,value)); break;
            case 0x15: if(ground) for(int i=0;i<6;i++) powerBoard.setConverter(i,(value&(1<<i))!=0); break;
            case 0x16: if(ground) powerBoard.setProtections(value!=0); break;
            default: trace.log("command.ignore", "reason=unknown_opcode opcode="+op); break;
        }
        if(!ground && ((op>=3 && op<=11 && op!=6) || op>=0x14)) trace.log("command.ignore", "reason=requires_ground opcode="+op);
    }
    private void reject(String reason) { badPackets=(badPackets+1)&255; trace.log("command.reject", "reason="+reason+" bad_packets="+badPackets); }
    private static void put24(byte[] p,int offset,int value) { p[offset]=(byte)value; p[offset+1]=(byte)(value>>8); p[offset+2]=(byte)(value>>16); }
    private static int clampCount(double value,int low,int high) { return (int)Math.max(low,Math.min(high,Math.round(value))); }
    /** FC constructTelemetryPacket: raw counts synthesized at the declared engineering boundary. */
    private byte[] constructTelemetryPacket() {
        packetNum=(packetNum+1)&0xffff; byte[] p=new byte[128]; ByteBuffer out=ByteBuffer.wrap(p).order(ByteOrder.LITTLE_ENDIAN);
        out.putShort(0,(short)pyroController.getPyrosStatus());
        for(int i=0;i<6;i++) { if(pyroController.isArmed(i)) p[2]|=1<<i; if(pyroController.isFired(i)) p[3]|=1<<i; }
        int s0=output.airbrakePulseUs(),s1=1500,s2=output.servo2Us(),s3=output.servo3Us();
        p[10]=(byte)s0; p[11]=(byte)((s0>>8)|((s1&15)<<4)); p[12]=(byte)(s1>>4);
        p[13]=(byte)s2; p[14]=(byte)((s2>>8)|((s3&15)<<4)); p[15]=(byte)(s3>>4);
        float[] av={accel.getAccelX(),accel.getAccelY(),accel.getAccelZ()};
        for(int i=0;i<3;i++) put24(p,16+3*i,clampCount(av[i]/(9.80665*(i==0?1.060:1.0))*12800,-524288,524287));
        double[] gv={(gyro.getGyroX()+0.2525f)/0.03051757812,(gyro.getGyroY()+0.2441f)/0.03051757812,(gyro.getGyroZ()+0.4376f)/0.03051757812};
        for(int i=0;i<3;i++) out.putShort(25+2*i,(short)clampCount(gv[i],-32768,32767));
        int rawT=clampCount((baro.getTemperature()*100.0-2000)*(1<<23)/0x6D91+0x8405*256.0,0,0xffffff);
        float dT=rawT-(float)0x8405*256;
        long off=(long)(((long)0x953A)*(1L<<17)+dT*(float)0x6305/(1<<6));
        long sens=(long)((float)0xA579*(1L<<16)+dT*(float)0x68AC/(1<<7));
        int rawP=clampCount((baro.getPressure()*100.0+off/(double)(1<<15))*(1<<15)/(sens/(double)(1<<21)),0,0xffffff);
        put24(p,53,rawP); put24(p,56,rawT);
        p[31]=(byte)gps.getFixType(); out.putInt(32,gps.getLatE7()); out.putInt(36,gps.getLonE7()); out.putFloat(40,gps.getHeight());
        out.putFloat(59,baro.getFilteredAltitude()); p[63]=(byte)currentState.ID;
        out.putFloat(64,gyro.getRoll()); out.putFloat(68,gyro.getPitch()); out.putFloat(72,gyro.getYaw());
        out.putShort(76,(short)baro.getMaxAlt()); out.putShort(78,(short)gps.getMaxAlt()); out.putInt(80,(int)FCtime); out.putShort(84,(short)packetNum);
        for(int i=0;i<3;i++) out.putShort(87+2*i,(short)(powerBoard.getCellVoltage(i)*1000));
        p[95]=40; p[97]=(byte)(powerBoard.protectionsEnabled()?1:0);
        for(int i=0;i<6;i++) out.putShort(98+2*i,(short)Math.round(powerBoard.getVoltageByRail(i)/0.0016));
        out.putFloat(122,accel.getIntegratedVelo()); for(int i=0;i<127;i++) p[127]+=p[i];
        trace.log("telemetry.construct", "packet="+packetNum+" raw_sensors=synthesized unmodeled_fields=zero"); return p;
    }
}
