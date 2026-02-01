package edu.mit.rocket_team.zephyrus.internal;


// individual cell voltage
// total battery voltage

// total battery current

// ! there are 6 power rails
// 28 V
// 8.4 V
// 7.4 V
// 5 V
// 3.3 V - !! THIS IS THE FC POWER SOURCE
// 3 V

// for each rail:
// voltage
// current

// FC action:
// turn on/off any of the rails (except for 3.3 V)

// override screw switch option


public class RTPowerBoard {

    public static double[] voltageByCell = new double[3];

    public static double totalBatteryVoltage = 0;

    // RAILS ARE LABELED BY VOLTAGE
    public static double[] voltageByRail = new double[6];
    public static final double[] defaultVoltageByRail = new double[]{
            28,
            8.4,
            7.4,
            5,
            3.3,
            3
    };
    public static final String[] railNames = new String[]{
            "28 V",
            "8.4 V",
            "7.4 V",
            "5 V",
            "3.3 V <FC POWER SOURCE>",
            "3 V"
    };
    public static double[] currentByRail = new double[6];

    public static boolean SCREW_SWITCH_OVERRIDEN = false;

    public RTPowerBoard() {
        for (int i = 0; i < voltageByRail.length; i++) {
            voltageByRail[i] = defaultVoltageByRail[i];
            currentByRail[i] = 0;
            if (i < 3) {
                voltageByCell[i] = 3.7;
            }
        }
    }

    public double getCellVoltage(int cell) {
        return voltageByCell[cell];
    }
    public double getTotalBatteryVoltage() {
        double total = 0;
        for (double voltage : voltageByCell) {
            total += voltage;
        }
        return total;
    }
    public double getCurrentByRail(int rail) {
        return currentByRail[rail];
    }
    public double getVoltageByRail(int rail) {
        return voltageByRail[rail];
    }

    public boolean shutOffRail(int rail) {
        if (rail == 4) {
            // CANNOT SHUT OFF 3.3 V
            return false;
        }
        voltageByRail[rail] = 0;
        // hardware: shut off rail.
        return true;
    }

    public boolean turnOnRail(int rail) {
        if (rail == 4) {
            // CANNOT SHUT OFF 3.3 V
            return false;
        }
        voltageByRail[rail] = defaultVoltageByRail[rail];
        return true;
    }

    public void setOverrideScrewSwitch(boolean shouldOverride) {
        // hardware: set override screw switch
        SCREW_SWITCH_OVERRIDEN = shouldOverride;
    }
    public boolean getOverrideScrewSwitch() {
        return SCREW_SWITCH_OVERRIDEN;
    }



}
