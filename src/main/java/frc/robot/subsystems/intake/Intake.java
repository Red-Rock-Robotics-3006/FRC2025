package frc.robot.subsystems.intake;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;

public class Intake extends SubsystemBase {
    private static Intake instance = null;

    private final TalonFX m_intakeLeft = new TalonFX(21, "*"); // Left intake motor
    private final TalonFX m_intakeRight = new TalonFX(22, "*"); // Right intake motor
    private final TalonFX m_pivot = new TalonFX(20, "*"); // Pivot motor

    private Slot0Configs intakeSlot0Configs = new Slot0Configs();
    private Slot0Configs pivotSlot0Configs = new Slot0Configs();

    private MotionMagicConfigs intakeMotionConfigs = new MotionMagicConfigs();
    private MotionMagicConfigs pivotMotionConfigs = new MotionMagicConfigs();

    private CurrentLimitsConfigs intakeCurrentLimitConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true);

    private CurrentLimitsConfigs pivotCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true);

    private SmartDashboardNumber intakeSpeed = new SmartDashboardNumber("intake/intake-speed", -3200);

    private SmartDashboardNumber pivotMotionAccel = new SmartDashboardNumber("pivot/pivot-mm-accel", 175);
    private SmartDashboardNumber pivotMotionVelo = new SmartDashboardNumber("pivot/pivot-mm-velo", 75);

    private SmartDashboardNumber pivotStowPosition = new SmartDashboardNumber("pivot/pivot-stow-position", 1.5);
    private SmartDashboardNumber pivotDeployPosition = new SmartDashboardNumber("pivot/pivot-deploy-position", 28.6);

    private Intake() {
        super("Intake");

        this.m_intakeLeft.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withPeakForwardDutyCycle(1d)
                        .withPeakReverseDutyCycle(-1d)
                        .withNeutralMode(NeutralModeValue.Brake));

        this.m_intakeRight.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive) // Opposite direction
                        .withPeakForwardDutyCycle(1d)
                        .withPeakReverseDutyCycle(-1d)
                        .withNeutralMode(NeutralModeValue.Brake));

        this.m_pivot.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withPeakForwardDutyCycle(1d)
                        .withPeakReverseDutyCycle(-1d)
                        .withNeutralMode(NeutralModeValue.Brake));

        pivotMotionConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(pivotMotionAccel.getNumber())
                .withMotionMagicCruiseVelocity(pivotMotionVelo.getNumber());

        this.m_pivot.getConfigurator().apply(pivotMotionConfigs);
        this.m_pivot.getConfigurator().apply(pivotCurrentLimitsConfigs);
        this.m_intakeLeft.getConfigurator().apply(intakeCurrentLimitConfigs);
        this.m_intakeRight.getConfigurator().apply(intakeCurrentLimitConfigs);

        this.m_intakeRight.setControl(new Follower(21, true));
    }

    public void enableIntake() {
        double speed = intakeSpeed.getNumber() / 60d;
        this.m_intakeLeft.setControl(new MotionMagicVelocityVoltage(speed)
                .withSlot(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true));
    }

    public void reverseIntake() {
        double speed = -intakeSpeed.getNumber() / 60d;
        this.m_intakeLeft.setControl(new MotionMagicVelocityVoltage(speed)
                .withSlot(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true));
    }

    public void disableIntake() {
        this.m_intakeLeft.setControl(new MotionMagicVelocityVoltage(0)
                .withSlot(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true));
    }

    public void setStowPosition() {
        this.m_pivot.setControl(new MotionMagicVoltage(pivotStowPosition.getNumber())
                .withSlot(0).withEnableFOC(true).withOverrideBrakeDurNeutral(false));
    }

    public void setDeployPosition() {
        this.m_pivot.setControl(new MotionMagicVoltage(pivotDeployPosition.getNumber())
                .withSlot(0).withEnableFOC(true).withOverrideBrakeDurNeutral(false));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake/left-speed", m_intakeLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("intake/right-speed", m_intakeRight.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("pivot/position", m_pivot.getPosition().getValueAsDouble());
    }

    public Command enableIntakeCommand() {
        return Commands.runOnce(this::enableIntake, this);
    }

    public Command reverseIntakeCommand() {
        return Commands.runOnce(this::reverseIntake, this);
    }

    public Command disableIntakeCommand() {
        return Commands.runOnce(this::disableIntake, this);
    }

    public Command setStowPositionCommand() {
        return Commands.runOnce(this::setStowPosition, this);
    }

    public Command setDeployPositionCommand() {
        return Commands.runOnce(this::setDeployPosition, this);
    }

    public static Intake getInstance() {
        if (instance == null)
            instance = new Intake();
        return instance;
    }
}
