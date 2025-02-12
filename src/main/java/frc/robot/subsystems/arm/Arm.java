package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;

public class Arm extends SubsystemBase {
    private static Arm instance = null;

    private final TalonFX m_armMotor = new TalonFX(0); // TODO FILLER

    private SmartDashboardNumber minMotorPosition = new SmartDashboardNumber("arm/minMotorPosition", 0);
    private SmartDashboardNumber maxMotorPosition = new SmartDashboardNumber("arm/maxMotorPosition", 0);
    private SmartDashboardNumber minDegrees = new SmartDashboardNumber("arm/minDegrees", 0);
    private SmartDashboardNumber maxDegrees = new SmartDashboardNumber("arm/maxDegrees", 0);
    

    private Slot0Configs armSlot0Configs = new Slot0Configs();

    private MotionMagicConfigs armMotionConfigs = new MotionMagicConfigs();

    private CurrentLimitsConfigs armCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(50)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true);

    private SmartDashboardNumber armMotionAccel = new SmartDashboardNumber("arm/arm-mm-accel", 10); // update
    private SmartDashboardNumber armMotionVel = new SmartDashboardNumber("arm/arm-mm-vel", 10);
    private SmartDashboardNumber armMotionJerk = new SmartDashboardNumber("arm/arm-mm-jerk", 0);

    private SmartDashboardNumber armKs = new SmartDashboardNumber("arm/ks", 0.12);
    private SmartDashboardNumber armKa = new SmartDashboardNumber("arm/ka", 0);
    private SmartDashboardNumber armKv = new SmartDashboardNumber("arm/kv", 0); // to be tuned;
    private SmartDashboardNumber armKp = new SmartDashboardNumber("arm/kp", 1);
    private SmartDashboardNumber armKi = new SmartDashboardNumber("arm/ki", 0);
    private SmartDashboardNumber armKd = new SmartDashboardNumber("arm/kd", 0);

    private SmartDashboardNumber armKg = new SmartDashboardNumber("arm/kg", 0.022);

    private SmartDashboardNumber delta = new SmartDashboardNumber("delta", 5);
    private SmartDashboardNumber target = new SmartDashboardNumber("target", 0);

                                                                                                             

    private Arm() {
        super("Arm");

        this.m_armMotor.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withPeakForwardDutyCycle(1d)
                        .withPeakReverseDutyCycle(-1d)
                        .withNeutralMode(NeutralModeValue.Brake));

        
        this.armSlot0Configs = new Slot0Configs()
                .withKS(armKs.getNumber())
                .withKA(armKa.getNumber())
                .withKV(armKv.getNumber())
                .withKP(armKp.getNumber())
                .withKI(armKi.getNumber())
                .withKD(armKd.getNumber())
                .withKG(armKg.getNumber())
                .withGravityType(GravityTypeValue.Arm_Cosine);

        this.armMotionConfigs = new MotionMagicConfigs()   
                .withMotionMagicAcceleration(armMotionAccel.getNumber());

        this.m_armMotor.getConfigurator().apply(armSlot0Configs);
        
        this.m_armMotor.getConfigurator().apply(armMotionConfigs);
      
        this.m_armMotor.getConfigurator().apply(armCurrentLimitsConfigs);

    }

    public void setPosition(double rotations) {
        this.m_armMotor.setControl(
            new MotionMagicVoltage(rotations)
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(false)
        );
    }

    public void setPostionDegrees(double degrees) {
        this.setPosition(this.degreesToRotations(degrees));
    }

    @Override
    public void periodic() {
        if (armKs.hasChanged()
                || armKa.hasChanged()
                || armKv.hasChanged()
                || armKp.hasChanged()
                || armKi.hasChanged()
                || armKd.hasChanged()
                || armKg.hasChanged()) {
            armSlot0Configs.kS = armKs.getNumber();
            armSlot0Configs.kA = armKa.getNumber();
            armSlot0Configs.kV = armKv.getNumber();
            armSlot0Configs.kP = armKp.getNumber();
            armSlot0Configs.kI = armKi.getNumber();
            armSlot0Configs.kD = armKd.getNumber();
            armSlot0Configs.kG = armKg.getNumber();

            this.m_armMotor.getConfigurator().apply(armSlot0Configs);
            
            System.out.println("applied"); // comment when necessary
        }

        if (armMotionAccel.hasChanged() || armMotionVel.hasChanged()) {
            armMotionConfigs.MotionMagicAcceleration = armMotionAccel.getNumber();
            armMotionConfigs.MotionMagicCruiseVelocity = armMotionVel.getNumber();
            this.m_armMotor.getConfigurator().apply(armMotionConfigs);
        }

        if (armMotionJerk.hasChanged()) {
            armMotionConfigs.MotionMagicJerk = armMotionJerk.getNumber();
            this.m_armMotor.getConfigurator().apply(armMotionConfigs);
        }

        SmartDashboard.putNumber("arm/arm-Motor-position", m_armMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("arm/arm-Motor-spike", m_armMotor.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("arm/arm-Motor-velocity", m_armMotor.getVelocity().getValueAsDouble());
    }

    public void increaseTarget() {
        target.putNumber(target.getNumber() + delta.getNumber());
      }
    
    public void decreaseTarget() {
        target.putNumber(target.getNumber() - delta.getNumber());
    }

    public void setTarget() {
        this.setPosition(this.target.getNumber());
    }
    public double degreesToRotations(double angle){
        return (maxMotorPosition.getNumber()- minMotorPosition.getNumber())/(maxDegrees.getNumber()-minDegrees.getNumber())*(angle- minDegrees.getNumber())+minMotorPosition.getNumber();
    }
    public static Arm getInstance() {
        if (instance == null)
            instance = new Arm();
        return instance;
    }
}
