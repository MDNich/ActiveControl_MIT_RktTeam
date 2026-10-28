package edu.mit.rocket_team.instrument;

import edu.mit.rocket_team.util.*;
import edu.mit.rocket_team.util.data.*;

/** GPS.cpp decoded NAV-PVT boundary, height in millimeters before source zeroing. */
public class RTGPS extends RTInstrument {
    private RTGPSData pending;
    private float heightOffset, maxAlt, pdop, vdop, hdop;
    private int latitudeE7, longitudeE7;
    private int heightMm, fixType;

    /** Creates a RTGPS with the supplied dependencies and per-instance state. */
    public RTGPS() {
        super();
    }

    /**
     * Creates a RTGPS with the supplied dependencies and per-instance state.
     *
     * @param trace run-local virtual clock and action-log sink
     */
    public RTGPS(RTUtilLibrary.Trace trace) {
        super(trace);
    }

    /** Records initialization of the held GPS-fix model. */
    @Override
    public void setup() {
        trace.log("gps.setup", "rate_hz=10 datum=simulator_absolute_height");
    }

    /**
     * Loads a GPS observation without inventing new fixes between acquisitions.
     *
     * @param data input data for this operation
     */
    @Override
    public void backdoorFudge(RTFudgedData data) {
        pending = (RTGPSData) data;
    }

    /**
     * Consumes a pending fix, quantizes coordinates to 1e-7 degrees and altitude to millimetres,
     * and updates maximum relative height only for a 3D fix.
     */
    public void updateAndParse() {
        if (pending != null) {
            latitudeE7 = (int) Math.round(pending.getLatitude() * 1e7);
            longitudeE7 = (int) Math.round(pending.getLongitude() * 1e7);
            heightMm = (int) Math.round(pending.getAltitude() * 1000);
            fixType = pending.getFixType();
            pdop = pending.getPDOP();
            vdop = pending.getVDOP();
            hdop = pending.getHDOP();
            pending = null;
            if (fixType == 3 && getHeight() > maxAlt) maxAlt = getHeight();
            trace.log(
                    "gps.update",
                    "new_data=true lat_deg="
                            + getLatitude()
                            + " lon_deg="
                            + getLongitude()
                            + " height_mm="
                            + heightMm
                            + " relative_m="
                            + getHeight()
                            + " maximum_m="
                            + maxAlt
                            + " fix_type="
                            + fixType);
        } else trace.log("gps.update", "new_data=false fix_type=" + fixType);
    }

    /** Captures the current MSL altitude as the GPS height reference. */
    private long referenceEpoch;
    public double getReferenceMslM() { return heightOffset / 1000.0; }
    public long getReferenceEpoch() { return referenceEpoch; }

    public void zeroAlt() {
        referenceEpoch++;
        heightOffset = heightMm;
        maxAlt = 0;
        trace.log("gps.zero", "offset_mm=" + heightOffset);
    }

    /**
     * Returns GPS altitude above the captured zero reference, in metres.
     *
     * @return GPS altitude above the captured zero reference, in metres
     */
    public float getHeight() {
        return (float) ((heightMm - heightOffset) / 1000.0);
    }

    /**
     * Returns relative GPS height in metres; this compatibility accessor delegates to getHeight,
     * not the original MSL input.
     *
     * @return relative GPS height in metres; this compatibility accessor delegates to getHeight,
     *     not the original MSL input
     */
    public float getAltitude() {
        return getHeight();
    }

    /**
     * Returns the greatest relative GPS height observed since zeroing, in metres.
     *
     * @return the greatest relative GPS height observed since zeroing, in metres
     */
    public float getMaxAlt() {
        return maxAlt;
    }

    /**
     * Returns the fix type.
     *
     * @return the fix type
     */
    public int getFixType() {
        return fixType;
    }

    /**
     * Reports whether the held fix qualifies as a three-dimensional position fix.
     *
     * @return true when the condition described above holds
     */
    public boolean getFix() {
        return fixType == 3;
    }

    /**
     * Returns the latitude in decimal degrees.
     *
     * @return the latitude in decimal degrees
     */
    public double getLatitude() {
        return latitudeE7 * 1e-7;
    }

    /**
     * Returns latitude encoded as integer degrees times ten million.
     *
     * @return latitude encoded as integer degrees times ten million
     */
    public int getLatE7() {
        return latitudeE7;
    }

    /**
     * Returns the longitude in decimal degrees.
     *
     * @return the longitude in decimal degrees
     */
    public double getLongitude() {
        return longitudeE7 * 1e-7;
    }

    /**
     * Returns longitude encoded as integer degrees times ten million.
     *
     * @return longitude encoded as integer degrees times ten million
     */
    public int getLonE7() {
        return longitudeE7;
    }

    /**
     * Returns the pdop as a dimensionless dilution of precision.
     *
     * @return the pdop as a dimensionless dilution of precision
     */
    public float getPDOP() {
        return pdop;
    }

    /**
     * Returns the vdop as a dimensionless dilution of precision.
     *
     * @return the vdop as a dimensionless dilution of precision
     */
    public float getVDOP() {
        return vdop;
    }

    /**
     * Returns the hdop as a dimensionless dilution of precision.
     *
     * @return the hdop as a dimensionless dilution of precision
     */
    public float getHDOP() {
        return hdop;
    }
}
