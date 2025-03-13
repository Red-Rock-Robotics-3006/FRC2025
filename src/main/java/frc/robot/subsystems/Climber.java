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

    private SmartDashboardNumber climbSpeed = new SmartDashboardNumber("climber/climb-speed", 0.15);
    private SmartDashboardNumber deploySpeed = new SmartDashboardNumber("climber/climb-deploy", -0.15);


    private Climber() {
        super("climber");

        this.climbMotor
        .withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
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
        .withTuningEnabled(false);
    }

    public void resetClimb() {
        this.climbMotor.motor.setControl(new NeutralOut());
        this.climbMotor.motor.setPosition(0);
    }

    public void setStowSpeed() {
        this.climbMotor.motor.setControl(new DutyCycleOut(deploySpeed.getNumber()));
    }

    public void setDeploySpeed() {
        this.climbMotor.motor.setControl(new DutyCycleOut(climbSpeed.getNumber()));
    }

    public void stopClimb() {
        this.climbMotor.motor.setControl(new DutyCycleOut(0));
    }

    @Override
    public void periodic() {
        climbMotor.update();
    }

    public static Climber getInstance() {
        if (instance == null) instance = new Climber();
        return instance;
    }
}
