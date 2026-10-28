package info.openrocket.core.simulation.mission;

import java.io.*;
import java.nio.*;
import java.nio.channels.FileChannel;
import java.nio.file.*;

/** Bounds ZIP central-directory allocation before Commons Compress constructs its entry objects. */
final class MissionZipPreflight {
    static void check(Path path) throws IOException {
        try(var channel=FileChannel.open(path,StandardOpenOption.READ)) {
            long length=channel.size();if(length<22 || length>MissionArchive.MAX_BYTES+128L*1024*1024)throw new IOException("Invalid ZIP size");
            int tail=(int)Math.min(length,65557);ByteBuffer end=read(channel,length-tail,tail);int at=-1;
            for(int i=tail-22;i>=0;i--)if(end.getInt(i)==0x06054b50 && i+22+u16(end,i+20)==tail){at=i;break;}
            if(at<0)throw new IOException("Missing ZIP end directory");
            if(u16(end,at+4)!=0 || u16(end,at+6)!=0 || u16(end,at+8)!=u16(end,at+10))throw new IOException("Multi-volume ZIP is unsupported");
            long count=u16(end,at+10),bytes=u32(end,at+12),offset=u32(end,at+16),boundary=length-tail+at;
            if(count==65535 || bytes==0xffffffffL || offset==0xffffffffL) {
                if(boundary<20)throw new IOException("Missing ZIP64 locator");
                ByteBuffer locator=read(channel,boundary-20,20);
                if(locator.getInt(0)!=0x07064b50 || locator.getInt(4)!=0 || locator.getInt(16)!=1)throw new IOException("Invalid ZIP64 locator");
                long zip64=locator.getLong(8);ByteBuffer record=read(channel,zip64,56);
                if(record.getInt(0)!=0x06064b50 || record.getLong(4)<44 || record.getLong(4)>1024 || record.getInt(16)!=0 || record.getInt(20)!=0 || record.getLong(24)!=record.getLong(32))throw new IOException("Invalid ZIP64 end record");
                count=record.getLong(32);bytes=record.getLong(40);offset=record.getLong(48);boundary=zip64;
            }
            if(count<1 || count>MissionArchive.MAX_ENTRIES || bytes<46 || bytes>128L*1024*1024 || offset<0 || offset>boundary || bytes>boundary-offset)throw new IOException("ZIP directory exceeds supported bounds");
            long position=offset;
            for(long i=0;i<count;i++) {
                if(position>offset+bytes-46)throw new IOException("Truncated ZIP directory");
                var entry=read(channel,position,46);
                if(entry.getInt(0)!=0x02014b50 || (u16(entry,8)&1)!=0)throw new IOException("Invalid or encrypted ZIP entry");
                int name=u16(entry,28),extra=u16(entry,30),comment=u16(entry,32);
                if(name<1 || name>1024)throw new IOException("Invalid ZIP entry name size");
                position+=46L+name+extra+comment;
            }
            if(position!=offset+bytes)throw new IOException("ZIP directory count or length mismatch");
        }
    }
    private static int u16(ByteBuffer b,int at){return Short.toUnsignedInt(b.getShort(at));}
    private static long u32(ByteBuffer b,int at){return Integer.toUnsignedLong(b.getInt(at));}
    private static ByteBuffer read(FileChannel channel,long at,int count)throws IOException {
        if(at<0 || at>channel.size()-count)throw new IOException("Truncated ZIP structure");
        var buffer=ByteBuffer.allocate(count).order(ByteOrder.LITTLE_ENDIAN);while(buffer.hasRemaining()){int n=channel.read(buffer,at+buffer.position());if(n<0)throw new EOFException();}buffer.flip();return buffer;
    }
}
