package redrocklib.logging;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardNumber {
    public static final boolean kEnableDashboardTuning = true;

    private double defaultValue;
    private double lastValue;
    private String key;
    private boolean tuningEnabled = true;

    public SmartDashboardNumber(String key, double defaultValue){
        this(key, defaultValue, true);
    }

    public SmartDashboardNumber(String key, double defaultValue, boolean tunable){
        this.key = key;
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;
        this.withTuningEnabled(tunable);

        if (this.tuningEnabled) SmartDashboard.putNumber(this.key, this.defaultValue);
    }

    public SmartDashboardNumber withTuningEnabled(boolean enabled) {
        this.tuningEnabled = kEnableDashboardTuning && enabled;
        return this;
    }
    
    public void putNumber(double val){
        this.defaultValue = val;
        SmartDashboard.putNumber(this.key, val);
    }

    public double getNumber(){
        if (tuningEnabled) this.lastValue = SmartDashboard.getNumber(this.key, this.defaultValue);
        else this.lastValue = this.defaultValue;
        return this.lastValue;
    }

    public boolean hasChanged(){
        if (!tuningEnabled) return false;
        return Double.compare(this.lastValue, this.getNumber()) != 0;
    }
}
