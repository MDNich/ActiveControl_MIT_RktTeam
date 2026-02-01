package edu.mit.rocket_team.zephyrus.util.hardware;

public class RTHardwareSerial {
    public RTHardwareSerial() {

    }
    public void begin(int rate) {
        // no-op
    }
    public void println(Object toPrint) {
        System.out.println(toPrint);
    }
    public void println(double toPrint,int roundTo) {
        System.out.println(toPrint);
    }
    public void print(Object toPrint) {
        System.out.print(toPrint);
    }
    public void print(double toPrint,int roundTo) {
        System.out.println(toPrint);
    }
}
