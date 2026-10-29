package info.openrocket.core.simulation.mission;

import jakarta.json.*;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;

/**
 * OpenRocket owns one set of ordinary rt-gs Away publishers; control stays on private process
 * pipes.
 */
public final class MissionNetwork implements AutoCloseable {
  private final Path root;
  private Process process;
  private OutputStream output;
  private InputStream input;
  private ExecutorService reader;
  private JsonObject configuration;
  private JsonObject status =
      Json.createObjectBuilder()
          .add("state", "off")
          .add("services", JsonValue.EMPTY_JSON_ARRAY)
          .build();

  public MissionNetwork(Path root) {
    this.root = root;
  }

  public static boolean enabled(JsonObject mission) {
    return mission.containsKey("network")
        && mission.getJsonObject("network").getBoolean("enabled", false);
  }

  private static JsonObject signature(JsonObject mission) {
    var value =
        Json.createObjectBuilder()
            .add("missionId", MissionStore.id(mission))
            .add("title", mission.getJsonObject("identity").getString("title"));
    value.add(
        "recordEncodedVideo",
        mission.getJsonObject("outputs").getBoolean("recordEncodedVideo", false));
    for (String key : List.of("sites", "video", "network", "vehicleBindings", "pointers"))
      value.add(key, mission.get(key));
    return value.build();
  }

  public synchronized JsonObject start(JsonObject mission) throws IOException {
    if (!enabled(mission)) throw new IllegalArgumentException("Enable network sharing first");
    JsonObject selected = signature(mission);
    if (process != null && process.isAlive() && selected.equals(configuration))
      return request("status", JsonValue.EMPTY_JSON_OBJECT);
    close();
    Files.createDirectories(root);
    byte[] bundle;
    JsonObject manifest;
    try (var in =
        MissionNetwork.class.getResourceAsStream("/mission/network/station-network.zip")) {
      if (in == null) throw new IOException("The bundled station network publisher is missing");
      bundle = in.readNBytes(4 * 1024 * 1024);
    }
    try (var in = MissionNetwork.class.getResourceAsStream("/mission/network/network.json")) {
      manifest = MissionJson.read(in);
    }
    if (!MissionWorker.digest(bundle).equals(manifest.getString("sha256")))
      throw new IOException("Station network bundle hash mismatch");
    Path zip = root.resolve("station-network.zip");
    Files.write(zip, bundle);
    Path python = MissionWorker.interpreter();
    Path runtime =
        python.getParent().getFileName().toString().equals("bin")
            ? python.getParent().getParent()
            : python.getParent();
    var builder =
        new ProcessBuilder(
            python.toString(),
            "-I",
            "-u",
            "-c",
            "import"
                + " sys,runpy;sys.path.insert(0,sys.argv.pop(1));runpy.run_module('rocket_gnc_monitor.mission_network',run_name='__main__')",
            zip.toString(),
            root.resolve("stations").toString());
    Path media = runtime.resolve("network-media");
    if (Files.isDirectory(media))
      try (var files = Files.list(media)) {
        var library =
            files
                .filter(
                    p ->
                        p.getFileName()
                            .toString()
                            .matches("(?i)(lib)?srt.*\\.(dylib|dll)|libsrt\\.so.*"))
                .findFirst();
        if (library.isPresent())
          builder.environment().put("ROCKET_SRT_LIBRARY", library.get().toString());
      }
    // A developer override is optional; packaged applications always carry their own runtime.
    String library = System.getProperty("openrocket.mission.srt");
    if (library != null) builder.environment().put("ROCKET_SRT_LIBRARY", library);
    builder.redirectError(root.resolve("publisher.stderr.log").toFile());
    process = builder.start();
    output = process.getOutputStream();
    input = new BufferedInputStream(process.getInputStream());
    reader =
        Executors.newSingleThreadExecutor(
            r -> {
              var t = new Thread(r, "mission-network-reply");
              t.setDaemon(true);
              return t;
            });
    try {
      status =
          request(
              "configure",
              Json.createObjectBuilder()
                  .add("mission", mission)
                  .add("discovery", !Boolean.getBoolean("openrocket.mission.noDiscovery"))
                  .build());
      configuration = selected;
      return status;
    } catch (IOException | RuntimeException e) {
      close();
      throw e;
    }
  }

  public synchronized JsonObject status() throws IOException {
    return process == null ? status : request("status", JsonValue.EMPTY_JSON_OBJECT);
  }

  public synchronized JsonObject finish() throws IOException {
    return request("finish", JsonValue.EMPTY_JSON_OBJECT);
  }

  public synchronized void begin(JsonArray streams) throws IOException {
    request("begin", Json.createObjectBuilder().add("streams", streams).build());
  }

  public synchronized void advance(long timeUs, JsonArray batches, JsonArray stations)
      throws IOException {
    request(
        "advance",
        Json.createObjectBuilder()
            .add("timeUs", Long.toString(timeUs))
            .add("batches", batches)
            .add("stations", stations)
            .build());
  }

  public synchronized void phase(String state) throws IOException {
    if (process != null)
      status = request("phase", Json.createObjectBuilder().add("state", state).build());
  }

  private JsonObject request(String operation, JsonObject fields) throws IOException {
    if (process == null || !process.isAlive())
      throw new IOException(
          "Station publisher stopped; see " + root.resolve("publisher.stderr.log"));
    byte[] bytes =
        (Json.createObjectBuilder(fields).add("operation", operation).build() + "\n")
            .getBytes(StandardCharsets.UTF_8);
    if (bytes.length > 4 * 1024 * 1024)
      throw new IOException("Station network message exceeds its size limit");
    output.write(bytes);
    output.flush();
    Future<JsonObject> response =
        reader.submit(
            () -> {
              var line = new ByteArrayOutputStream();
              for (int count = 0; count < 4 * 1024 * 1024; count++) {
                int b = input.read();
                if (b < 0)
                  throw new EOFException(
                      "Station publisher stopped; see " + root.resolve("publisher.stderr.log"));
                if (b == '\n')
                  try (var json = Json.createReader(new ByteArrayInputStream(line.toByteArray()))) {
                    return json.readObject();
                  }
                line.write(b);
              }
              throw new IOException("Station publisher reply exceeds its size limit");
            });
    try {
      JsonObject reply =
          response.get(
              operation.equals("configure")
                      || operation.equals("begin")
                      || operation.equals("finish")
                  ? 30
                  : 10,
              TimeUnit.SECONDS);
      if (!reply.getBoolean("ok"))
        throw new IOException(reply.getString("error", "Station publisher rejected the request"));
      return reply.getJsonObject("result");
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      throw new IOException("Station publication interrupted", e);
    } catch (ExecutionException | TimeoutException e) {
      close();
      throw new IOException("Station publisher did not respond: " + e.getMessage(), e);
    }
  }

  @Override
  public synchronized void close() {
    if (process != null) {
      try {
        output.close();
        if (!process.waitFor(10, TimeUnit.SECONDS)) process.destroy();
      } catch (IOException e) {
        process.destroy();
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        process.destroy();
      }
      if (process.isAlive()) process.destroyForcibly();
    }
    if (reader != null) reader.shutdownNow();
    process = null;
    configuration = null;
    status =
        Json.createObjectBuilder()
            .add("state", "off")
            .add("services", JsonValue.EMPTY_JSON_ARRAY)
            .build();
  }
}
