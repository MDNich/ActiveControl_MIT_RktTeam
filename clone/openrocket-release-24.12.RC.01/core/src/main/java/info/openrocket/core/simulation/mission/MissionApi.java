package info.openrocket.core.simulation.mission;

import com.sun.net.httpserver.*;
import info.openrocket.core.document.*;
import info.openrocket.core.file.GeneralRocketLoader;
import info.openrocket.core.motor.*;
import info.openrocket.core.rocketcomponent.*;
import info.openrocket.core.simulation.extension.impl.ZephyrusFlightComputer;
import info.openrocket.core.startup.Application;
import jakarta.json.*;
import java.io.*;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.security.SecureRandom;
import java.util.*;
import java.util.concurrent.*;

/** Authenticated loopback configuration service. File access always uses the application's chooser. */
public final class MissionApi implements AutoCloseable {
    public interface Host {
        Path choose(String purpose, boolean save) throws Exception;
        OpenRocketDocument currentDocument() throws Exception;
    }
    public interface Jobs extends AutoCloseable {
        JsonObject capabilities();
        default JsonArray validate(JsonObject mission) { return JsonValue.EMPTY_JSON_ARRAY; }
        JsonObject start(MissionStore.Resolved resolved) throws Exception;
        JsonObject request(String id, String action, JsonObject body) throws Exception;
        @Override void close();
    }
    private final HttpServer server;
    private final ExecutorService executor;
    private final MissionStore store;
    private final Host host;
    private final Jobs jobs;
    private final String token, origin;
    private final Map<String, Path> results = new HashMap<>();
    private final Map<String, OpenRocketDocument> documents = new HashMap<>();
    private record Start(String hash, JsonObject response) { }
    private final Map<String, Start> starts = new LinkedHashMap<>();
    private JsonObject preferences = JsonValue.EMPTY_JSON_OBJECT;

    public MissionApi(MissionStore store, Host host, Jobs jobs) throws IOException {
        this.store = store; this.host = host; this.jobs = jobs;
        Path preferencesFile = store.root().resolve("machine-preferences.json");
        if (Files.isRegularFile(preferencesFile)) try (var input = Files.newInputStream(preferencesFile)) { preferences = MissionJson.read(input); }
        byte[] secret = new byte[32]; new SecureRandom().nextBytes(secret); token = HexFormat.of().formatHex(secret);
        server = HttpServer.create(new InetSocketAddress(InetAddress.getByName("127.0.0.1"), 0), 16);
        origin = "http://127.0.0.1:" + server.getAddress().getPort();
        executor = new ThreadPoolExecutor(2, 4, 30, TimeUnit.SECONDS, new ArrayBlockingQueue<>(32), r -> {
            Thread thread = new Thread(r, "mission-api"); thread.setDaemon(true); return thread;
        });
        server.setExecutor(executor); server.createContext("/", this::handle); server.start();
    }
    public URI uri(String draft) { return URI.create(origin + "/#token=" + token + "&draft=" + draft); }
    public String origin() { return origin; }
    /** For the local browser's bootstrap only; never persist or archive. */
    public String token() { return token; }

    private void handle(HttpExchange exchange) throws IOException {
        try {
            if (!exchange.getRemoteAddress().getAddress().isLoopbackAddress()
                    || !("127.0.0.1:" + server.getAddress().getPort()).equals(exchange.getRequestHeaders().getFirst("Host"))) {
                respond(exchange, 403, error("Local origin required")); return;
            }
            String path = exchange.getRequestURI().getRawPath();
            if (!path.startsWith("/mission-api/v1/")) { resource(exchange, path); return; }
            if (!token.equals(exchange.getRequestHeaders().getFirst("X-Mission-Token"))) { respond(exchange, 403, error("Workspace credential required")); return; }
            String suppliedOrigin = exchange.getRequestHeaders().getFirst("Origin");
            boolean write = !exchange.getRequestMethod().equals("GET");
            if ((suppliedOrigin != null && !origin.equals(suppliedOrigin)) || (write && !origin.equals(suppliedOrigin))) {
                respond(exchange, 403, error("Workspace origin required")); return;
            }
            if (!Set.of("GET", "POST", "PUT").contains(exchange.getRequestMethod())) { respond(exchange, 405, error("Unsupported method")); return; }
            JsonObject body = write ? MissionJson.read(exchange.getRequestBody()) : JsonValue.EMPTY_JSON_OBJECT;
            JsonObject result = route(exchange.getRequestMethod(), path.substring("/mission-api/v1/".length()), body);
            respond(exchange, 200, result);
        } catch (MissionStore.Conflict e) { respond(exchange, 409, error(e.getMessage())); }
        catch (IllegalArgumentException | IOException e) { respond(exchange, 400, error(message(e))); }
        catch (Exception e) { respond(exchange, 500, error(message(e))); }
        finally { exchange.close(); }
    }

    private synchronized JsonObject route(String method, String path, JsonObject body) throws Exception {
        if (!path.matches("[a-zA-Z0-9/_-]+")) throw new IllegalArgumentException("Invalid endpoint");
        String[] parts = path.split("/");
        if ((path.equals("results/open") || path.equals("results/recover")) && method.equals("POST")) {
            Path file;
            if(path.endsWith("recover")) {
                Path staging=host.choose("Choose interrupted mission staging folder",false);if(staging==null)return cancelled();
                file=host.choose("Save recovered .rktmission",true);if(file==null)return cancelled();
                Path directory=store.root().resolve("result-utilities");Files.createDirectories(directory);
                MissionResults.recover(staging,file,directory);
            } else {file=host.choose("Open .rktmission result",false);if(file==null)return cancelled();}
            JsonObject summary=MissionResults.query(file,store.root().resolve("result-utilities"),JsonValue.EMPTY_JSON_OBJECT);
            String id=UUID.randomUUID().toString();results.put(id,file);
            return Json.createObjectBuilder(summary).add("id",id).add("filename",file.getFileName().toString()).build();
        }
        if(parts.length==3 && parts[0].equals("results") && parts[2].equals("derive") && method.equals("POST")) {
            Path file=results.get(parts[1]);if(file==null)throw new IllegalArgumentException("Open a result first");
            return store.deriveResult(file,body.getString("member"),body.getString("telemetrySource"));
        }
        if(parts.length==3 && parts[0].equals("results") && parts[2].equals("export") && method.equals("POST")) {
            Path file=results.get(parts[1]);if(file==null)throw new IllegalArgumentException("Open a result first");
            Path destination=host.choose("Export selected station .rktflight",true);if(destination==null)return cancelled();
            return MissionResults.utility(store.root().resolve("result-utilities"),Json.createObjectBuilder().add("operation","export")
                    .add("archive",file.toString()).add("destination",destination.toString()).add("selection",body).build());
        }
        if(parts.length==3 && parts[0].equals("results") && parts[2].equals("query") && method.equals("POST")) {
            Path file=results.get(parts[1]);if(file==null)throw new IllegalArgumentException("Open a result first");
            return MissionResults.query(file,store.root().resolve("result-utilities"),body);
        }
        if (path.equals("preferences") && method.equals("GET")) return preferences;
        if (path.equals("preferences") && method.equals("POST")) {
            String kind = body.getString("kind");
            if(kind.equals("checkApplication"))return MissionWorker.applicationCapabilities(preferences);
            if(kind.equals("attachment") || kind.equals("detach") || kind.equals("checkAttachment")) {
                String binding=body.getString("bindingId");
                if(!binding.matches("[A-Za-z0-9][A-Za-z0-9._-]{0,63}"))throw new IllegalArgumentException("Invalid station binding");
                var saved=preferences.getJsonObject("attachments");if(saved==null)saved=JsonValue.EMPTY_JSON_OBJECT;
                if(kind.equals("checkAttachment"))return MissionAttachment.check(Path.of(saved.getString(binding)));
                var attachments=Json.createObjectBuilder(saved);
                if(kind.equals("detach"))attachments.remove(binding);
                else {
                    Path ticket=host.choose("Choose running Telemetry UI connection file",false);if(ticket==null)return cancelled();
                    MissionAttachment.check(ticket);attachments.add(binding,ticket.toAbsolutePath().toString());
                }
                preferences=Json.createObjectBuilder(preferences).add("attachments",attachments).build();
                MissionJson.writeAtomic(store.root().resolve("machine-preferences.json"),preferences);return preferences;
            }
            if (!Set.of("sourceProject", "pythonExecutable", "packagedApp").contains(kind)) throw new IllegalArgumentException("Unknown application location");
            String purpose = kind.equals("sourceProject") ? "Choose Telemetry UI project folder" : kind.equals("pythonExecutable") ? "Choose Python executable" : "Choose Telemetry UI application";
            Path file = host.choose(purpose, false); if (file == null) return cancelled();
            var selection=Json.createObjectBuilder(preferences);
            if(kind.equals("packagedApp"))selection.remove("sourceProject").remove("pythonExecutable");else selection.remove("packagedApp");
            preferences = selection.add(kind, file.toAbsolutePath().toString()).build();
            MissionJson.writeAtomic(store.root().resolve("machine-preferences.json"), preferences); return preferences;
        }
        if (path.equals("capabilities") && method.equals("GET")) return Json.createObjectBuilder()
                .add("protocol", "mission-api.v1").add("missionVersion", 1).add("runtime", jobs == null ? Json.createObjectBuilder().add("available", false).build() : jobs.capabilities())
                .add("geoid", MissionGeoid.bundled().provenance()).add("commands", MissionJson.resource("fc-command-capabilities.json"))
                .add("choices", choices()).build();
        if (path.equals("coordinates") && method.equals("POST")) return MissionCoordinates.mgrs(body.getString("mgrs"), body.getString("reference", "grid-point"));
        if (path.equals("motors") && method.equals("POST")) return motors(body.getString("query", ""));
        if (path.equals("drafts") && method.equals("POST")) return store.create(body.getString("title", "URRG mission"));
        if (path.equals("drafts/load") && method.equals("POST")) {
            Path file = host.choose("Load mission configuration", false); if (file == null) return cancelled();
            if (file.getFileName().toString().endsWith(".rktmission-setup")) return store.importSetup(file);
            try (var input = Files.newInputStream(file)) {
                var json = MissionJson.read(input);
                return store.load(json.getString("format", "").equals("openrocket.mission-configuration-preview")
                        ? MissionPreviewImport.migrate(json) : new MissionDefinition(json));
            }
        }
        if (parts[0].equals("drafts") && parts.length >= 2) {
            String id = parts[1];
            if (parts.length == 2 && method.equals("GET")) return store.get(id);
            if (parts.length == 2 && method.equals("PUT")) return store.update(id, body.getInt("expectedRevision"), body.getJsonObject("mission"));
            if (parts.length == 3 && method.equals("POST")) {
                switch (parts[2]) {
                    case "export": {
                        Path file = host.choose("Export portable .rktmission-setup", true); if (file == null) return cancelled();
                        store.exportSetup(id, file); return Json.createObjectBuilder().add("saved", file.getFileName().toString()).build();
                    }
                    case "save": {
                        Path file = host.choose("Save mission configuration", true); if (file == null) return cancelled();
                        store.save(id, file); return Json.createObjectBuilder().add("saved", file.getFileName().toString()).build();
                    }
                    case "validate": {
                        var issuesBuilder=Json.createArrayBuilder(store.validate(id));
                        if(jobs!=null)jobs.validate(store.get(id)).forEach(issuesBuilder::add);
                        var issues = issuesBuilder.build();
                        return Json.createObjectBuilder().add("issues", issues).add("configurationValid", issues.isEmpty())
                                .add("runtimeAvailable", jobs != null && jobs.capabilities().getBoolean("available", false)).build();
                    }
                    case "resolve": {
                        var resolved = store.resolve(id, body.getInt("expectedRevision"));
                        return Json.createObjectBuilder().add("hash", resolved.hash()).add("mission", resolved.mission()).build();
                    }
                    case "rocket": {
                        OpenRocketDocument doc;
                        if (body.getBoolean("current", false)) doc = host.currentDocument();
                        else { Path file = host.choose("Choose rocket .ork", false); if (file == null) return cancelled(); doc = new GeneralRocketLoader(file.toFile()).load(); }
                        if (doc == null) throw new IllegalArgumentException("No rocket document is open");
                        documents.put(id, doc); return simulations(doc);
                    }
                    case "flight": {
                        var doc = documents.get(id); if (doc == null) throw new IllegalArgumentException("Choose a rocket first");
                        int selected = body.getInt("simulation", 0); Simulation simulation;
                        if (doc.getSimulationCount() == 0) {
                            simulation = new Simulation(doc.getRocket());
                            simulation.setFlightConfigurationId(doc.getRocket().getSelectedConfiguration().getFlightConfigurationID());
                        } else simulation = doc.getSimulation(selected);
                        return store.bindFlight(id, body.getInt("expectedRevision"), simulation,
                                doc.getFile() == null ? doc.getRocket().getName() + ".ork" : doc.getFile().getName());
                    }
                    case "fc": {
                        Path file = host.choose("Choose flight computer .fc", false); if (file == null) return cancelled();
                        var simulation = store.editableFlight(id); ZephyrusFlightComputer.selectFile(simulation, file);
                        var fc = ZephyrusFlightComputer.read(simulation); var config = fc.getConfig(); config.put("enabled", true); fc.setConfig(config);
                        return store.bindFlight(id, body.getInt("expectedRevision"), simulation, store.get(id).getJsonObject("flight").getString("filename"));
                    }
                }
            }
        }
        if (path.equals("jobs") && method.equals("POST")) {
            if (jobs == null || !jobs.capabilities().getBoolean("available", false)) throw new IllegalArgumentException("Mission runtime is unavailable");
            String key = body.getString("idempotencyKey"), hash = body.getString("resolvedHash");
            if (!key.matches("[a-zA-Z0-9_-]{16,128}")) throw new IllegalArgumentException("Invalid start key");
            var previous = starts.get(key);
            if (previous != null) { if (!previous.hash().equals(hash)) throw new MissionStore.Conflict("Start key was already used for different inputs"); return previous.response(); }
            if (starts.size() >= 10000) throw new IllegalArgumentException("This workspace has reached its job limit; open a new workspace");
            var resolved = store.resolve(body.getString("draftId"), body.getInt("expectedRevision"));
            if (!resolved.hash().equals(hash)) throw new MissionStore.Conflict("Resolved inputs changed; validate and resolve again");
            var result = jobs.start(resolved); starts.put(key, new Start(hash, result)); return result;
        }
        if (parts[0].equals("jobs") && parts.length >= 2 && parts.length <= 3 && jobs != null) {
            String action = parts.length == 2 ? "status" : parts[2];
            if (Set.of("status", "events", "results").contains(action) ? method.equals("GET") : method.equals("POST"))
                return jobs.request(parts[1], action, body);
        }
        throw new IllegalArgumentException("Unknown endpoint or method");
    }

    private static JsonObject choices() {
        var result = Json.createObjectBuilder();
        result.add("ignition", names(IgnitionEvent.values())).add("separation", names(StageSeparationConfiguration.SeparationEvent.values()))
                .add("deployment", names(DeploymentConfiguration.DeployEvent.values()));
        return result.build();
    }
    private static JsonArray names(Enum<?>[] values) { var a = Json.createArrayBuilder(); for (var value : values) a.add(value.name()); return a.build(); }
    private static JsonObject simulations(OpenRocketDocument doc) {
        var array = Json.createArrayBuilder();
        for (int i = 0; i < doc.getSimulationCount(); i++) array.add(Json.createObjectBuilder().add("index", i).add("name", doc.getSimulation(i).getName()));
        if (doc.getSimulationCount() == 0) array.add(Json.createObjectBuilder().add("index", 0).add("name", "New flight from selected motor configuration"));
        return Json.createObjectBuilder().add("simulations", array).add("rocketName", doc.getRocket().getName()).build();
    }
    private static JsonObject motors(String query) {
        if (query.length() > 100) throw new IllegalArgumentException("Motor search is too long");
        var array = Json.createArrayBuilder(); int count = 0;
        for (Motor motor : Application.getMotorSetDatabase().findMotors(null, null, null, null, Double.NaN, Double.NaN)) {
            if (!(motor instanceof ThrustCurveMotor curve)) continue;
            String label = curve.getManufacturer().getDisplayName() + " " + curve.getDesignation();
            if (!label.toLowerCase(Locale.ROOT).contains(query.toLowerCase(Locale.ROOT))) continue;
            array.add(Json.createObjectBuilder().add("digest", curve.getDigest()).add("name", label)
                    .add("diameterM", curve.getDiameter()).add("lengthM", curve.getLength())); if (++count == 100) break;
        }
        return Json.createObjectBuilder().add("motors", array).build();
    }

    private void resource(HttpExchange exchange, String path) throws IOException {
        if (!exchange.getRequestMethod().equals("GET")) { respond(exchange, 405, error("GET required")); return; }
        if (path.equals("/")) path = "/index.html";
        if (!path.matches("/[a-zA-Z0-9/_.,-]+") || path.contains("..")) { respond(exchange, 404, error("Not found")); return; }
        try (var input = MissionApi.class.getResourceAsStream("/mission/workspace" + path)) {
            if (input == null) { respond(exchange, 404, error("Not found")); return; }
            byte[] bytes = input.readAllBytes();
            String type = path.endsWith(".html") ? "text/html" : path.endsWith(".js") ? "text/javascript" : path.endsWith(".css") ? "text/css" : path.endsWith(".svg") ? "image/svg+xml" : "image/png";
            headers(exchange, type); exchange.getResponseHeaders().set("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data: https:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'none'");
            exchange.sendResponseHeaders(200, bytes.length); exchange.getResponseBody().write(bytes);
        }
    }
    private static JsonObject cancelled() { return Json.createObjectBuilder().add("cancelled", true).build(); }
    private static JsonObject error(String message) { return Json.createObjectBuilder().add("error", message).build(); }
    private static String message(Exception e) { return e.getMessage() == null ? e.getClass().getSimpleName() : e.getMessage(); }
    private static void headers(HttpExchange e, String type) {
        e.getResponseHeaders().set("Content-Type", type + (type.startsWith("image/") ? "" : "; charset=utf-8"));
        e.getResponseHeaders().set("Cache-Control", "no-store"); e.getResponseHeaders().set("X-Content-Type-Options", "nosniff");
        e.getResponseHeaders().set("Referrer-Policy", "strict-origin-when-cross-origin"); e.getResponseHeaders().set("X-Frame-Options", "DENY");
    }
    private static void respond(HttpExchange e, int status, JsonObject object) throws IOException {
        byte[] bytes = object.toString().getBytes(StandardCharsets.UTF_8); headers(e, "application/json");
        e.sendResponseHeaders(status, bytes.length); e.getResponseBody().write(bytes);
    }
    @Override public void close() { if (jobs != null) jobs.close(); server.stop(0); executor.shutdownNow(); }
}
