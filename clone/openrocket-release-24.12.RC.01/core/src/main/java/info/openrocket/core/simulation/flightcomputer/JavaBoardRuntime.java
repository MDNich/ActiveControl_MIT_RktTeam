package info.openrocket.core.simulation.flightcomputer;

import jakarta.json.JsonObject;
import java.util.*;
import java.util.function.*;

/** Independent virtual board schedules with shared measured inputs; no inter-board transport model. */
public final class JavaBoardRuntime {
    @FunctionalInterface public interface Output {
        void write(String boardId,String port,double value,long sampleUs);
    }
    private static final class Board {
        final String id,name;final long period,cost;final JavaBoardProgram program;
        long nextStartUs,lastStartUs=-1,finishUs=-1;
        JavaBoardProgram.Context context;
        Board(JsonObject node){
            id=node.getString("id");name=node.getString("name");var p=node.getJsonObject("properties");
            period=p.getJsonNumber("periodUs").longValueExact();cost=p.getJsonNumber("executionUs").longValueExact();
            program=JavaBoardProgram.compile(node.getString("program")).create();
        }
    }
    private final List<Board> boards;
    public JavaBoardRuntime(FlightComputerDesign design){
        JavaBoardProgram.requireApproved(design);
        boards=design.nodes().getValuesAs(JsonObject.class).stream().filter(n->n.getString("type").equals("java_board"))
            .sorted(Comparator.comparing(n->n.getString("id"))).map(Board::new).toList();
    }
    public long nextDeadlineUs(){
        long next=Long.MAX_VALUE;for(var b:boards)next=Math.min(next,b.finishUs>=0?b.finishUs:b.nextStartUs);return next;
    }
    public void dispatch(long nowUs,long sampleUs,String state,ToDoubleFunction<String> signals,Predicate<String> connected,
                         Output output,Consumer<String> log){
        // Complete existing work first. Each board's period/cost affects only that board.
        for(var b:boards)if(b.finishUs==nowUs)complete(b,nowUs,log);
        Map<String,Double> snapshot=null;Set<String> ports=null;
        for(var b:boards)if(b.finishUs<0&&b.nextStartUs==nowUs){
            if(snapshot==null){
                var values=new HashMap<String,Double>();for(var key:FlightComputerDesign.SIGNALS)values.put(key,signals.applyAsDouble(key));snapshot=Map.copyOf(values);
                var present=new HashSet<String>();for(var port:FlightComputerModels.REQUIRED_PORTS)if(connected.test(port))present.add(port);ports=Set.copyOf(present);
            }
            final var readings=snapshot;final var available=ports;
            long dt=b.lastStartUs<0?0:nowUs-b.lastStartUs;b.lastStartUs=nowUs;b.finishUs=nowUs+b.cost;
            b.nextStartUs=Math.max(nowUs+b.period,b.finishUs);
            b.context=new JavaBoardProgram.Context(nowUs,dt,state,key->{var v=readings.get(key);if(v==null)throw new IllegalArgumentException("Unknown board signal "+key);return v;},available::contains,
                (port,value)->{log.accept("board="+b.id+" event=output port="+port+" value="+value+" input_sample_us="+sampleUs);output.write(b.id,port,value,sampleUs);},
                text->log.accept("board="+b.id+" "+text));
            log.accept("board="+b.id+" name="+b.name+" event=start start_us="+nowUs+" complete_us="+b.finishUs+" period_us="+b.period+" cost_us="+b.cost+" input_sample_us="+sampleUs);
        }
        // All boards starting together get the same immutable snapshot before zero-cost outputs.
        for(var b:boards)if(b.finishUs==nowUs)complete(b,nowUs,log);
    }
    private void complete(Board b,long nowUs,Consumer<String> log){
        log.accept("board="+b.id+" name="+b.name+" event=step start_us="+b.lastStartUs+" complete_us="+nowUs+" cost_us="+b.cost+
                " overrun_us="+Math.max(0,b.cost-b.period)+" next_start_us="+b.nextStartUs);
        try{b.program.step(b.context);}catch(RuntimeException | LinkageError e){throw new IllegalStateException("Java board '"+b.name+"' failed at "+nowUs+" µs: "+e.getMessage(),e);}
        b.finishUs=-1;b.context=null;
    }
}
