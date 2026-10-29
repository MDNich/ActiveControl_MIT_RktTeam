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
    public interface Host extends MissionService.Host { }
    public interface Jobs extends MissionService.Jobs { }
    private final HttpServer server;
    private final ExecutorService executor;
    private final MissionService service;
    private final String token, origin;

    public MissionApi(MissionStore store, MissionService.Host host, MissionService.Jobs jobs) throws IOException {
        service = new MissionService(store, host, jobs);
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
            JsonObject result = service.request(exchange.getRequestMethod(), path.substring("/mission-api/v1/".length()), body);
            respond(exchange, 200, result);
        } catch (MissionStore.Conflict e) { respond(exchange, 409, error(e.getMessage())); }
        catch (IllegalArgumentException | IOException e) { respond(exchange, 400, error(message(e))); }
        catch (Exception e) { respond(exchange, 500, error(message(e))); }
        finally { exchange.close(); }
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
    @Override public void close() { service.close(); server.stop(0); executor.shutdownNow(); }
}
