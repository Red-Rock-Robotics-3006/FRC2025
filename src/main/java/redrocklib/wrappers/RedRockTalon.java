package redrocklib.wrappers;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redrocklib.logging.SmartDashboardNumber;

public class RedRockTalon {
    public static final boolean kEnableMotorDashboardTuning = true;

    public TalonFX motor;

    private SmartDashboardNumber kS;
    private SmartDashboardNumber kA;
    private SmartDashboardNumber kV;
    private SmartDashboardNumber kP;
    private SmartDashboardNumber kI;
    private SmartDashboardNumber kD;
    private SmartDashboardNumber kG;

    private SmartDashboardNumber motionJerk;
    private SmartDashboardNumber motionAccel;
    private SmartDashboardNumber motionVel;

    private SmartDashboardNumber spikeThreshold;

    private boolean tuningEnabled = true;

    private Slot0Configs slot0Configs;
    private MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

    private String name;

    public RedRockTalon(int motorID) {
        this(motorID, "motor-" + motorID);
    }

    /**
     * Creates a wrapper for TalonFX that handles commonly used configs, commonly used telemetry, and updating config code
     * 
     * REMMEBER TO CALL {@link #update()} every loop somewhere in code
     * @param motorID
     * @param name
     * @param canivore
     */
    public RedRockTalon(int motorID, String name, String canivore) {
        this(motorID, name, canivore, new MotorOutputConfigs(), new Slot0Configs());
    }

    /**
     *Creates a wrapper for TalonFX that handles commonly used configs, commonly used telemetry, and updating config code
     * 
     * REMMEBER TO CALL {@link #update()} every loop somewhere in code
     * @param motorID
     * @param name
     */
    public RedRockTalon(int motorID, String name) {
        this(motorID, name, new MotorOutputConfigs(), new Slot0Configs());
    }

    /**
     * Creates a wrapper for TalonFX that handles commonly used configs, commonly used telemetry, and updating config code
     * 
     * REMMEBER TO CALL {@link #update()} every loop somewhere in code
     * @param motorID
     * @param name
     * @param canivore
     * @param outPutConfigs
     * @param slot0Configs
     */
    public RedRockTalon(int motorID, String name, String canivore, MotorOutputConfigs outPutConfigs, Slot0Configs slot0Configs) {
        this.motor = new TalonFX(motorID, canivore);

        this.name = name;

        this.withMotorOutputConfigs(outPutConfigs)
            .withSlot0Configs(slot0Configs)
            .withCurrentLimitConfigs(new CurrentLimitsConfigs()
                                        .withSupplyCurrentLimit(60)
                                        .withSupplyCurrentLimitEnable(true)
                                        .withStatorCurrentLimit(100)
                                        .withStatorCurrentLimitEnable(true))
            .withSpikeThreshold(5)
            .withMotionMagicConfigs(new MotionMagicConfigs())
            .withTuningEnabled(true);


    }

    /**
     * Creates a wrapper for TalonFX that handles commonly used configs, commonly used telemetry, and updating config code
     * 
     * REMMEBER TO CALL {@link #update()} every loop somewhere in code
     * @param motorID
     * @param name
     * @param outPutConfigs
     * @param slot0Configs
     */
    public RedRockTalon(int motorID, String name, MotorOutputConfigs outPutConfigs, Slot0Configs slot0Configs) {
        this.motor = new TalonFX(motorID);

        this.name = name;

        this.withMotorOutputConfigs(outPutConfigs)
            .withSlot0Configs(slot0Configs)
            .withCurrentLimitConfigs(new CurrentLimitsConfigs()
                                        .withSupplyCurrentLimit(60)
                                        .withSupplyCurrentLimitEnable(true)
                                        .withStatorCurrentLimit(100)
                                        .withStatorCurrentLimitEnable(true))
            .withSpikeThreshold(5)
            .withMotionMagicConfigs(new MotionMagicConfigs())
            .withTuningEnabled(true);


    }

    public RedRockTalon withTuningEnabled(boolean enabled) {
        this.tuningEnabled = kEnableMotorDashboardTuning && enabled;
        return this;
    }

    public RedRockTalon withSpikeThreshold(double threshold) {
        this.spikeThreshold = new SmartDashboardNumber(name + "/" + name + "-spike-threshold", threshold);
        return this;
    }

    public RedRockTalon withCurrentLimitConfigs(CurrentLimitsConfigs configs) {
        this.motor.getConfigurator().apply(configs);
        return this;
    }

    public RedRockTalon withMotorOutputConfigs(MotorOutputConfigs configs) {
        this.motor.getConfigurator().apply(configs);
        return this;
    }

    public RedRockTalon withFeedbackConfigs(FeedbackConfigs configs){
        this.motor.getConfigurator().apply(configs);
        return this;
    }

    public RedRockTalon withSlot0Configs(Slot0Configs slot0Configs) {
        this.slot0Configs = slot0Configs;
        this.kS = new SmartDashboardNumber(name + "/" + name + "-slot0/kS", slot0Configs.kS);
        this.kA = new SmartDashboardNumber(name + "/" + name + "-slot0/kA", slot0Configs.kA);
        this.kV = new SmartDashboardNumber(name + "/" + name + "-slot0/kV", slot0Configs.kV);
        this.kP = new SmartDashboardNumber(name + "/" + name + "-slot0/kP", slot0Configs.kP);
        this.kI = new SmartDashboardNumber(name + "/" + name + "-slot0/kI", slot0Configs.kI);
        this.kD = new SmartDashboardNumber(name + "/" + name + "-slot0/kD", slot0Configs.kD);
        this.kG = new SmartDashboardNumber(name + "/" + name + "-slot0/kG", slot0Configs.kG);

        this.motor.getConfigurator().apply(slot0Configs);

        return this;
    }

    public RedRockTalon withMotionMagicConfigs(MotionMagicConfigs configs) {
        this.motionMagicConfigs = configs;
        this.motionAccel = new SmartDashboardNumber(name + "/" + name + "-motion-magic/accel", configs.MotionMagicAcceleration);
        this.motionVel = new SmartDashboardNumber(name + "/" + name + "-motion-magic/velocity", configs.MotionMagicCruiseVelocity);
        this.motionJerk = new SmartDashboardNumber(name + "/" + name + "-motion-magic/jerk", configs.MotionMagicJerk);

        this.motor.getConfigurator().apply(configs);

        return this;
    }

    public boolean aboveSpikeThreshold() {
        return Math.abs(motor.getTorqueCurrent().getValueAsDouble()) >= spikeThreshold.getNumber();
    }

    public void setMotionMagicPosition(double position) {
        this.motor.setControl(
            new MotionMagicVoltage(position)
            .withSlot(0)
            .withEnableFOC(true)
            .withOverrideBrakeDurNeutral(true)
        );
    }

    public void setMotionMagicVelocity(double rpm) {
        this.motor.setControl(
            new MotionMagicVelocityVoltage(rpm / 60d)
            .withSlot(0)
            .withEnableFOC(true)
            .withOverrideBrakeDurNeutral(true)
        );
    }

    public double getSpikeThreshold() {
        return this.spikeThreshold.getNumber();
    }

    /**
     * Updates telemetry, and applies any changed config numbers for the motor
     * 
     * MUST BE CALLED EVERY LOOP
     */
    public void update() {
        if (tuningEnabled) {
            if (kS.hasChanged()
                || kA.hasChanged()
                || kV.hasChanged()
                || kP.hasChanged()
                || kI.hasChanged()
                || kD.hasChanged()
                || kG.hasChanged()) {
                slot0Configs.kS = kS.getNumber();
                slot0Configs.kA = kA.getNumber();
                slot0Configs.kV = kV.getNumber();
                slot0Configs.kP = kP.getNumber();
                slot0Configs.kI = kI.getNumber();
                slot0Configs.kD = kD.getNumber();
                slot0Configs.kG = kG.getNumber();
    
                motor.getConfigurator().apply(slot0Configs);
            }
    
            if (motionVel.hasChanged() && Double.compare(motionVel.getNumber(), 0) != 0) {
                motionMagicConfigs.MotionMagicCruiseVelocity = motionVel.getNumber();
                motor.getConfigurator().apply(motionMagicConfigs);
            }
    
            if (motionAccel.hasChanged() && Double.compare(motionAccel.getNumber(), 0) != 0) {
                motionMagicConfigs.MotionMagicAcceleration = motionAccel.getNumber();
                motor.getConfigurator().apply(motionMagicConfigs);
            }
    
            if (motionJerk.hasChanged() && Double.compare(motionJerk.getNumber(), 0) != 0) {
                motionMagicConfigs.MotionMagicJerk = motionJerk.getNumber();
                motor.getConfigurator().apply(motionMagicConfigs);
            }
        }

        SmartDashboard.putNumber(name + "/" + name + "-current-position", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(name + "/" + name + "-current-velocity", motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(name + "/" + name + "-current-acceleration", motor.getAcceleration().getValueAsDouble());
        SmartDashboard.putNumber(name + "/" + name + "-torque-current", motor.getTorqueCurrent().getValueAsDouble());

        SmartDashboard.putBoolean(name + "/" + name + "-above-spike-threshold", this.aboveSpikeThreshold());
    }

}