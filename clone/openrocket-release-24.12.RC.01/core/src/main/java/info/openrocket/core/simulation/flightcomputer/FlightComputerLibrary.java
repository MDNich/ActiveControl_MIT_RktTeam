package info.openrocket.core.simulation.flightcomputer;

import info.openrocket.core.arch.SystemInfo;
import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Per-user managed library. Import never overwrites the source or an existing distinct design. */
public final class FlightComputerLibrary {
    private FlightComputerLibrary(){}
    public static Path directory() {
        return Path.of(System.getProperty("openrocket.fc.library",SystemInfo.getUserApplicationDirectory().toPath().resolve("FlightComputers").toString())).toAbsolutePath().normalize();
    }
    public static synchronized Path template() throws IOException {
        Path p=directory().resolve("Templates/zephyrus.fc");
        var baseline=FlightComputerDesign.zephyrus();
        if(!Files.exists(p)) baseline.write(p);
        else if(!FlightComputerDesign.read(p).json().equals(baseline.json())) {
            // Preserve a manually modified older template; never silently replace its contents.
            p=directory().resolve("Templates/zephyrus-"+baseline.fingerprint().substring(0,12)+".fc");
            if(!Files.exists(p))baseline.write(p);
        }
        return p;
    }
    public static boolean protectedFile(Path p){return p.toAbsolutePath().normalize().startsWith(directory().resolve("Templates"));}
    public static synchronized Path importFile(Path source) throws IOException {
        Path from=source.toAbsolutePath().normalize();
        FlightComputerDesign.read(from); // Validate format before copying; editable drafts need not run yet.
        Files.createDirectories(directory());
        if(from.startsWith(directory())) return from;
        byte[] data=Files.readAllBytes(from); String hash=FlightComputerDesign.hash(data);
        for(Path existing:list()) if(FlightComputerDesign.hash(Files.readAllBytes(existing)).equals(hash))return existing;
        String name=from.getFileName().toString().replaceAll("[^A-Za-z0-9._-]","-");
        if(!name.toLowerCase(Locale.ROOT).endsWith(".fc"))name+=".fc";
        Path target=directory().resolve(name);
        if(Files.exists(target)) target=directory().resolve(name.substring(0,name.length()-3)+"-"+hash.substring(0,12)+".fc");
        int suffix=2; Path base=target;
        while(Files.exists(target))target=base.resolveSibling(base.getFileName().toString().replace(".fc","-"+(suffix++)+".fc"));
        Path temp=Files.createTempFile(directory(),".fc-import-",".tmp");
        try {Files.write(temp,data); Files.move(temp,target);} finally {Files.deleteIfExists(temp);}
        System.out.println("FC library.import source="+from+" library_file="+target+" sha256="+hash);
        return target;
    }
    public static List<Path> list() throws IOException {
        if(!Files.isDirectory(directory()))return List.of();
        try(var stream=Files.walk(directory(),2)) {return stream.filter(Files::isRegularFile).filter(p->p.toString().toLowerCase(Locale.ROOT).endsWith(".fc")).sorted().toList();}
    }
    public static String reference(Path p) {
        Path absolute=p.toAbsolutePath().normalize();
        if(!absolute.startsWith(directory()))throw new IllegalArgumentException("Import this design into the FC library first");
        return "library:"+directory().relativize(absolute).toString().replace('\\','/');
    }
    public static Path resolve(String reference) throws IOException {
        if(!reference.startsWith("library:"))throw new IOException("Locate and import this flight computer into the library: "+reference);
        Path p=directory().resolve(reference.substring(8)).normalize();
        if(!p.startsWith(directory())||!Files.isRegularFile(p))throw new IOException("Missing flight computer. Use Browse to locate/import: "+reference);
        return p;
    }
    public record Resolved(FlightComputerDesign design,Path path,String exactHash) {
        public Map<String,String> provenance(){var data=new java.util.LinkedHashMap<String,String>(Map.of("designId",design.id(),"name",design.name(),"computer",design.computer(),
                "model",design.model(),"semanticHash",design.fingerprint(),"fileHash",exactHash,"reference",reference(path)));data.putAll(FlightComputerData.stateNames(design));return Map.copyOf(data);}
    }
    public static Resolved load(String reference) throws IOException {
        Path p=resolve(reference);byte[] bytes=Files.readAllBytes(p);
        if(bytes.length>4_000_000)throw new IOException("FC file exceeds 4 MB");
        var d=FlightComputerDesign.read(new ByteArrayInputStream(bytes));d.requireRunnable();
        return new Resolved(d,p,FlightComputerDesign.hash(bytes));
    }
}
