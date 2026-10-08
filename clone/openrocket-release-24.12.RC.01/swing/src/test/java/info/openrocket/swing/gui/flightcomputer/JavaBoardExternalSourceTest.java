package info.openrocket.swing.gui.flightcomputer;

import java.nio.file.*;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import static org.junit.jupiter.api.Assertions.*;

class JavaBoardExternalSourceTest {
    @TempDir Path directory;
    @Test void workingCopiesAreIndependentAndTrackExternalAndLocalEdits()throws Exception{
        var first=new JavaBoardExternalSource(directory,"class BoardProgram {}\n");var other=new JavaBoardExternalSource(directory,"other source");
        assertNotEquals(first.file(),other.file());assertFalse(first.externallyChanged());assertFalse(first.editorChanged(first.read()));
        Files.writeString(first.file(),"// edited externally\n");assertTrue(first.externallyChanged());assertTrue(first.editorChanged("// edited inside OR\n"));assertEquals("other source",other.read());
        var text=first.read();first.acknowledge(text);assertFalse(first.externallyChanged());assertFalse(first.editorChanged(text));
        first.write("// exported update\n");assertEquals("// exported update\n",Files.readString(first.file()));
        assertThrows(java.io.IOException.class,()->first.write("x".repeat(100001)));assertEquals("// exported update\n",Files.readString(first.file()));
        Files.writeString(first.file(),"x".repeat(100001));assertThrows(java.io.IOException.class,first::read);
    }
}
