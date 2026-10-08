package info.openrocket.swing.gui.flightcomputer;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;

/** An editing copy, never an executable input or a replacement for the saved .fc source. */
final class JavaBoardExternalSource {
    private final Path file;
    private String synchronizedText;
    JavaBoardExternalSource(Path directory,String source)throws IOException{
        Files.createDirectories(directory);
        file=Files.createTempDirectory(directory,"board-").resolve("BoardProgram.java");
        write(source);
    }
    Path file(){return file;}
    boolean editorChanged(String source){return !source.equals(synchronizedText);}
    boolean externallyChanged()throws IOException{return !read().equals(synchronizedText);}
    String read()throws IOException{
        if(Files.size(file)>400_000)throw new IOException("External source exceeds 100,000 characters");
        String text=Files.readString(file,StandardCharsets.UTF_8);check(text);return text;
    }
    void acknowledge(String source){synchronizedText=source;}
    void write(String source)throws IOException{
        check(source);Files.writeString(file,source,StandardCharsets.UTF_8);synchronizedText=source;
    }
    private static void check(String source)throws IOException{
        if(source.length()>100_000)throw new IOException("External source exceeds 100,000 characters");
    }
}
