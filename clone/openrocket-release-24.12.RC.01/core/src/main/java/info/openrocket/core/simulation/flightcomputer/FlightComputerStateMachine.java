package info.openrocket.core.simulation.flightcomputer;

import jakarta.json.*;
import java.util.*;
import java.util.function.ToDoubleFunction;

/** Editable state graph. At most one transition is taken at each completed FC iteration. */
public final class FlightComputerStateMachine {
    public static final List<String> PHASES = List.of("ground", "preflight", "flight", "apogee", "main");
    public static final Map<String,String> ACTIONS;
    static {
        var actions = new LinkedHashMap<String,String>();
        actions.put("prepare", "Prepare flight: zero sensors, start logging");
        actions.put("launch", "Start flight clock and controllers");
        actions.put("apogee", "Mark apogee, close controllers, fire recovery 0 + 1");
        actions.put("main", "Fire recovery 2 and disable roll outputs");
        actions.put("finish", "Stop flight logging");
        actions.put("fire_recovery", "Fire recovery channel");
        actions.put("close_controllers", "Close airbrakes and stop roll control");
        actions.put("airbrakes", "Set airbrake fraction");
        actions.put("roll", "Set roll servo angle (deg)");
        actions.put("log", "Write a log message");
        ACTIONS = Collections.unmodifiableMap(actions);
    }
    public interface Host {
        void enter(String id, String name, String phase);
        void action(JsonObject action);
        void transition(String from, String to);
    }
    private final JsonObject graph;
    private final Host host;
    private JsonObject state;
    private long enteredUs;
    public FlightComputerStateMachine(JsonObject graph, Host host) { validate(graph); this.graph=graph; this.host=host; }
    public String id() { return state==null?graph.getString("initial"):state.getString("id"); }
    public String name() { return state==null?state(graph,id()).getString("name"):state.getString("name"); }
    public long stateAgeMs(long nowUs) { return Math.max(0,nowUs-enteredUs)/1000; }
    public void start(long nowUs) { if(state!=null)throw new IllegalStateException("State machine already started");enter(id(),nowUs); }
    public void step(long nowUs,ToDoubleFunction<String> signals,double apogeeLockoutMs) {
        for(var t:transitions(graph).stream().filter(t->t.getString("from").equals(id())).sorted(Comparator.comparingInt(t->t.getInt("priority"))).toList()) {
            if(t.getBoolean("apogeeLockout",false)&&signals.applyAsDouble("flight_ms")<=apogeeLockoutMs)continue;
            if(FlightComputerDesign.evaluate(t.getJsonObject("condition"),s->s.equals("state_ms")?stateAgeMs(nowUs):signals.applyAsDouble(s))){
                String from=id();enter(t.getString("to"),nowUs);host.transition(from,id());return;
            }
        }
    }
    private void enter(String id,long nowUs) {
        state=state(graph,id);enteredUs=nowUs;
        host.enter(id,state.getString("name"),state.getString("phase"));
        for(var action:state.getJsonArray("actions").getValuesAs(JsonObject.class))host.action(action);
    }
    public static List<JsonObject> states(JsonObject g) { return g.getJsonArray("states").getValuesAs(JsonObject.class); }
    public static List<JsonObject> transitions(JsonObject g) { return g.getJsonArray("transitions").getValuesAs(JsonObject.class); }
    public static JsonObject state(JsonObject g,String id) { return states(g).stream().filter(s->s.getString("id").equals(id)).findFirst().orElse(null); }
    public static JsonObject condition(String signal,String op,double value) { return Json.createObjectBuilder().add("signal",signal).add("op",op).add("value",value).build(); }
    public static JsonObject defaults(FlightComputerDesign d) {
        var states=Json.createArrayBuilder();
        String[] names={"Ground","Preflight","Flight","Apogee","Main"},actions={"finish","prepare","launch","apogee","main"};
        for(int i=0;i<PHASES.size();i++)states.add(Json.createObjectBuilder().add("id",PHASES.get(i)).add("name",names[i]).add("phase",PHASES.get(i)).add("actions",Json.createArrayBuilder().add(Json.createObjectBuilder().add("type",actions[i]))));
        var links=Json.createArrayBuilder();String[] from={"ground","preflight","flight","flight","apogee","main"},to={"preflight","flight","apogee","apogee","main","ground"};
        for(var rule:d.rules().getValuesAs(JsonObject.class)){
            int index=new ArrayList<>(FlightComputerModels.RULES.keySet()).indexOf(rule.getString("id"));
            if(index<0)continue;
            links.add(Json.createObjectBuilder().add("id",rule.getString("id")).add("from",from[index]).add("to",to[index])
                .add("priority",index==3?1:0).add("apogeeLockout",index==2).add("condition",rule.getJsonObject("condition")));
        }
        return Json.createObjectBuilder().add("initial","ground").add("automaticRecovery",true).add("states",states).add("transitions",links).build();
    }
    /** Insert a real intermediate state, moving all outgoing transitions behind it. */
    public static JsonObject insert(JsonObject graph,String after,String id,String name,JsonObject condition,JsonArray actions) {
        var previous=Objects.requireNonNull(state(graph,after),"Select a state first");
        var node=Json.createObjectBuilder().add("id",id).add("name",name).add("phase",previous.getString("phase")).add("actions",actions).build();
        var nodes=Json.createArrayBuilder();for(var s:states(graph)){nodes.add(s);if(s.getString("id").equals(after))nodes.add(node);}
        var links=Json.createArrayBuilder();
        for(var t:transitions(graph))links.add(t.getString("from").equals(after)?Json.createObjectBuilder(t).add("from",id).build():t);
        links.add(Json.createObjectBuilder().add("id",UUID.randomUUID().toString()).add("from",after).add("to",id).add("priority",0).add("condition",condition));
        return Json.createObjectBuilder(graph).add("states",nodes).add("transitions",links).build();
    }
    public static void validate(JsonObject graph) {
        if(!Set.of("initial","automaticRecovery","states","transitions").containsAll(graph.keySet()))throw new IllegalArgumentException("Unknown state machine field");
        var states=states(graph);var transitions=transitions(graph);
        if(states.isEmpty()||states.size()>64||transitions.size()>256)throw new IllegalArgumentException("Use 1–64 states and at most 256 transitions");
        var ids=new HashSet<String>();
        for(var s:states){
            if(!Set.of("id","name","phase","actions").containsAll(s.keySet()))throw new IllegalArgumentException("Unknown state field");
            if(s.getString("id").isBlank()||!ids.add(s.getString("id"))||s.getString("name").isBlank())throw new IllegalArgumentException("States need unique IDs and nonempty names");
            if(!PHASES.contains(s.getString("phase")))throw new IllegalArgumentException("Unknown flight phase");
            if(s.getJsonArray("actions").size()>64)throw new IllegalArgumentException("Too many entry actions");
            for(var a:s.getJsonArray("actions").getValuesAs(JsonObject.class))validateAction(a);
        }
        if(!ids.contains(graph.getString("initial")))throw new IllegalArgumentException("Initial state does not exist");
        graph.getBoolean("automaticRecovery",true);
        var linkIds=new HashSet<String>();var priorities=new HashSet<String>();
        for(var t:transitions){
            if(!Set.of("id","from","to","priority","apogeeLockout","condition").containsAll(t.keySet()))throw new IllegalArgumentException("Unknown transition field");
            if(t.getString("id").isBlank()||!linkIds.add(t.getString("id")))throw new IllegalArgumentException("Transition IDs must be unique");
            if(!ids.contains(t.getString("from"))||!ids.contains(t.getString("to")))throw new IllegalArgumentException("Transition endpoint does not exist");
            if(!t.getJsonNumber("priority").isIntegral()||t.getInt("priority")<0||!priorities.add(t.getString("from")+":"+t.getInt("priority")))throw new IllegalArgumentException("Outgoing transition priorities must be unique and nonnegative");
            t.getBoolean("apogeeLockout",false);FlightComputerDesign.validateCondition(t.getJsonObject("condition"),0);
        }
        var reachable=new HashSet<String>();reachable.add(graph.getString("initial"));
        boolean changed;do{changed=false;for(var t:transitions)if(reachable.contains(t.getString("from")))changed|=reachable.add(t.getString("to"));}while(changed);
        if(!reachable.containsAll(ids))throw new IllegalArgumentException("Some states are unreachable from the initial state");
    }
    public static void validateAction(JsonObject a) {
        String type=a.getString("type");if(!ACTIONS.containsKey(type))throw new IllegalArgumentException("Unknown entry action: "+type);
        Set<String> fields=switch(type){case "fire_recovery"->Set.of("type","channel");case "airbrakes","roll"->Set.of("type","value");case "log"->Set.of("type","message");default->Set.of("type");};
        if(!fields.containsAll(a.keySet()))throw new IllegalArgumentException("Unsupported action field");
        if(type.equals("fire_recovery")&&(a.getInt("channel")<0||a.getInt("channel")>5||!a.getJsonNumber("channel").isIntegral()))throw new IllegalArgumentException("Recovery channel must be 0–5");
        if(type.equals("airbrakes")||type.equals("roll")){double value=a.getJsonNumber("value").doubleValue();double lo=type.equals("roll")?-90:0,hi=type.equals("roll")?90:1;if(!Double.isFinite(value)||value<lo||value>hi)throw new IllegalArgumentException("Output value out of range");}
        if(type.equals("log"))a.getString("message");
    }
}
