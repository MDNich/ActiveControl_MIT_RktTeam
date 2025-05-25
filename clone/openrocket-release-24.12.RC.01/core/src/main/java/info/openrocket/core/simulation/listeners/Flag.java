package info.openrocket.core.simulation.listeners;

public class Flag {

    private boolean flag = false;

    public void engage() {
        flag = true;
    }

    public void disengage() {
        flag = false;
    }

    public boolean get() {
        return flag;
    }

    public Flag() {
        flag = false;
    }

}
