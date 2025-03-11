package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Climber extends SubsystemBase{
    private static Climber instance = null;

    private RedRockTalon climberMotor = new RedRockTalon(61, "climber-motor", "*"); //TODO FILLER

    private SmartDashboardNumber spoolInSpeed = new SmartDashboardNumber("climber/climber-spoolInSpeed", 0.2);
    private SmartDashboardNumber spoolOutSpeed = new SmartDashboardNumber("climber/climber-spoolOutSpeed", -0.1);
    private SmartDashboardNumber normalizeSpeed = new SmartDashboardNumber("climber/climber-normalize-speed", -0.5);
    
    private SmartDashboardNumber spoolOutLimit = new SmartDashboardNumber("climber/climber-spool-out-limit", 0);
    private SmartDashboardNumber spoolInLimit = new SmartDashboardNumber("climber/climber-spool-in-limit", 0);

    private Climber() {
        super("Climber");


        this.climberMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive) //TODO FILLER
                .withPeakForwardDutyCycle(1d) //TODO FILLER
                .withPeakReverseDutyCycle(-1d) //TODO FILLER
                .withNeutralMode(NeutralModeValue.Brake)
        ).withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true)
        ).withSpikeThreshold(50);
    }

    public void spoolInClimber() {
        if(!atSpoolInLimit())
            this.climberMotor.motor.setControl(new DutyCycleOut(spoolInSpeed.getNumber()));
    }

    public void spoolOutClimber() {
        if(!atSpoolOutLimit())
            this.climberMotor.motor.setControl(new DutyCycleOut(spoolOutSpeed.getNumber()));
    }

    public void stopClimber() {
        this.climberMotor.motor.setControl(new DutyCycleOut(0));
    }

    public boolean atSpoolInLimit(){
        return this.climberMotor.motor.getPosition().getValueAsDouble() <= this.spoolInLimit.getNumber();
    }

    public boolean atSpoolOutLimit(){
        return this.climberMotor.motor.getPosition().getValueAsDouble() >= this.spoolOutLimit.getNumber();
    }

    public void setNormalizeSpeed() {
        this.climberMotor.motor.setControl(new DutyCycleOut(this.normalizeSpeed.getNumber()));
    }

    public void resetClimber() {
        this.stopClimber();
        this.climberMotor.motor.setPosition(0);
    }

    public Command spoolInCommand()
    {
        return Commands.sequence(
            Commands.runOnce(() -> this.spoolInClimber(), this),
            Commands.waitUntil(() -> this.atSpoolInLimit()),
            Commands.runOnce(() -> this.stopClimber(), this)
        );
    }

    public Command spoolOutCommand()
    {
        return new FunctionalCommand(
            () -> {this.spoolOutClimber();},
            null,
            (interrupted) -> {this.stopClimber();},
            () -> {return this.atSpoolOutLimit();},
            this);
    }

    public Command normalizeClimberCommand(){
        return new SequentialCommandGroup(
            new FunctionalCommand(
                () -> this.setNormalizeSpeed(),
                () -> {},
                (interrupted) -> this.resetClimber(),
                () -> this.climberMotor.aboveSpikeThreshold(),
                this
            )
            // ,
            // this.goToPosition(this.targetPosition)
        );
    }

    public static Climber getInstance() {
        if (instance == null) instance = new Climber();
        return instance;
    }
}