package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Intake extends SubsystemBase{
    private static Intake instance = null;

    private SmartDashboardNumber minPivotRotation = new SmartDashboardNumber("min-rotation", 0);
    private SmartDashboardNumber maxPivotRotation = new SmartDashboardNumber("max-rotation", 0);


    private RedRockTalon pivotMotor = new RedRockTalon(0, "intake-pivot-motor", "*");
    private RedRockTalon intakeMotor = new RedRockTalon(0, "intake-pivot-motor", "*");

    private CANrange caNrange = new CANrange(0, "*");

    private SmartDashboardNumber intakeDeployPosition = new SmartDashboardNumber("intake/intake-deploy-position", 0);
    private SmartDashboardNumber intakeStowPosition = new SmartDashboardNumber("intake/intake-stow-position", 0);
    
    private SmartDashboardNumber intakeSpeed = new SmartDashboardNumber("intake/intake-speed", 0.2);
    private SmartDashboardNumber outtakeSpeed = new SmartDashboardNumber("intake/outtake-speed", -0.3);
    private SmartDashboardNumber resetSpeed = new SmartDashboardNumber("intake/reset-speed", -0.3);

    private SmartDashboardNumber delta = new SmartDashboardNumber("intake/tuning/delta", 5);
    private SmartDashboardNumber target = new SmartDashboardNumber("intake/tuning/target", 0);

    private SmartDashboardNumber tofThreshold = new SmartDashboardNumber("intake/intake-tof-threshold", 0.1);

    private Intake() {
        super();

        this.pivotMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKP(0)
            .withKI(0)
            .withKD(0)
        )
        .withSpikeThreshold(55)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(25)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(40)
            .withStatorCurrentLimitEnable(true)
        );
                
        this.intakeMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withGravityType(GravityTypeValue.Arm_Cosine)
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(0)
            .withMotionMagicCruiseVelocity(0)
        )
        .withSpikeThreshold(55)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
        );       

        
    }

    public void setPivotResetSpeed() {
        this.pivotMotor.motor.setControl(new DutyCycleOut(this.resetSpeed.getNumber()));
    }

    public void resetPivot() {
        this.pivotMotor.motor.setControl(new CoastOut());
        this.pivotMotor.motor.setPosition(0);
    }

    public void setOuttakeSpeed() {
        this.setIntakeSpeed(outtakeSpeed.getNumber());
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

    public void setPosition(double rotations) {
        this.pivotMotor.setMotionMagicPosition(MathUtil.clamp(rotations, minPivotRotation.getNumber(), maxPivotRotation.getNumber()));
    }

    public void setIntakeSpeed(double speed) {
        this.intakeMotor.motor.setControl(new DutyCycleOut(speed));
    }

    public void setIntakeDeploy() {
        this.setPosition(intakeDeployPosition.getNumber());
    }

    public void setIntakeStow() {
        this.setPosition(intakeStowPosition.getNumber());
    }

    public void startIntake() {
        this.setIntakeSpeed(intakeSpeed.getNumber());
    }

    public void stopIntake() {
        this.setIntakeSpeed(0);
    }

    public boolean coralDetected() {
        return this.caNrange.getDistance().getValueAsDouble() < this.tofThreshold.getNumber();
    }

    @Override
    public void periodic() {
        this.intakeMotor.update();
        this.pivotMotor.update();
    }

    public Command resetIntakePivot() {
        return Commands.sequence(
            Commands.runOnce(this::setPivotResetSpeed, this),
            Commands.waitUntil(() -> this.pivotMotor.aboveSpikeThreshold()),
            Commands.runOnce(this::resetPivot, this)
        );
    }

}
