package edu.mit.rocket_team.util.hardware;

/**
 * Arduino serial compatibility wrapper around standard output. It does not open a serial port or
 * implement baud/precision behavior.
 */
public class RTHardwareSerial {
    /** Creates a RTHardwareSerial instance with its default state. */
    public RTHardwareSerial() {}

    /**
     * Retains the Arduino serial initialization signature; no serial device is opened.
     *
     * @param rate rate
     */
    public void begin(int rate) {
        // no-op
    }

    /**
     * Writes the supplied value to standard output followed by a newline; the compatibility
     * precision argument is not applied.
     *
     * @param toPrint to print
     */
    public void println(Object toPrint) {
        System.out.println(toPrint);
    }

    /**
     * Writes the supplied value to standard output followed by a newline; the compatibility
     * precision argument is not applied.
     *
     * @param toPrint to print
     * @param roundTo round to
     */
    public void println(double toPrint, int roundTo) {
        System.out.println(toPrint);
    }

    /**
     * Writes the supplied value to standard output; the numeric compatibility overload retains its
     * existing newline and ignores precision.
     *
     * @param toPrint to print
     */
    public void print(Object toPrint) {
        System.out.print(toPrint);
    }

    /**
     * Writes the supplied value to standard output; the numeric compatibility overload retains its
     * existing newline and ignores precision.
     *
     * @param toPrint to print
     * @param roundTo round to
     */
    public void print(double toPrint, int roundTo) {
        System.out.println(toPrint);
    }
}
