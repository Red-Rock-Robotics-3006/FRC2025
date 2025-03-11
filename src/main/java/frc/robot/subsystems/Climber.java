package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Climber extends SubsystemBase{
    private static Climber instance = null;

    private RedRockTalon climbMotor = new RedRockTalon(61, "climb-motor", "*");

    private SmartDashboardNumber deployPosition = new SmartDashboardNumber("climber/climb-deploy", 0);
    private SmartDashboardNumber stowPosition = new SmartDashboardNumber("climber/climb-stow", 0);
    private SmartDashboardNumber normalizeSpeed = new SmartDashboardNumber("climber/climb-normalize-speed", -0.02);

    private SmartDashboardNumber minRotation = new SmartDashboardNumber("climber/climb-min-rotation", 0);
    private SmartDashboardNumber maxRotation = new SmartDashboardNumber("climber/climb-max-rotation", 30);

    private Climber() {
        super("climber");

        this.climbMotor
        .withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1)
            .withPeakReverseDutyCycle(-1)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKG(0)
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKP(0)
            .withKI(0)
            .withKD(0)
        ).withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(20)
            .withMotionMagicCruiseVelocity(50)
            .withMotionMagicJerk(100000000)
        ).withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(50)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
        ).withSpikeThreshold(55)
        .withTuningEnabled(true);
    }

    public void setNormalizeSpeed() {
        this.climbMotor.motor.setControl(new DutyCycleOut(normalizeSpeed.getNumber()));
    }

    public void setPosition(double position) {
        this.climbMotor.setMotionMagicPosition(MathUtil.clamp(position, minRotation.getNumber(), maxRotation.getNumber()));
    }

    public void resetClimb() {
        this.climbMotor.motor.setControl(new NeutralOut());
        this.climbMotor.motor.setPosition(0);
    }

    public void setStowPos() {
        this.setPosition(stowPosition.getNumber());
    }

    public void setDeployPose() {
        this.setPosition(deployPosition.getNumber());
    }

    @Override
    public void periodic() {
        climbMotor.update();
    }

    public Command normalizeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> this.setNormalizeSpeed(), this),
            Commands.waitUntil(() -> this.climbMotor.aboveSpikeThreshold()),
            Commands.runOnce(() -> this.resetClimb())
        );
    }
}
