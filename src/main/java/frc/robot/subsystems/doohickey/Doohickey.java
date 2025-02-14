package frc.robot.subsystems.doohickey;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Doohickey extends SubsystemBase{
    private static Doohickey instance = null;

    private RedRockTalon spinMaster = new RedRockTalon(20, "doohickey-motor", "*");

    private SmartDashboardNumber intakeSpeed = new SmartDashboardNumber("doo/doo-intake-speed", 0.3);
    private SmartDashboardNumber outtakeSpeed = new SmartDashboardNumber("doo/doo-outtake-speed", 0.3);

    private Doohickey() {
        super("dhickey");

        spinMaster.withMotorOutputConfigs(
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
        )
        .withSpikeThreshold(5);
    }

    public void setSpeed(double speed) {
        spinMaster.motor.set(speed);
    }

    public void setIntakeSpeed() {
        this.setSpeed(intakeSpeed.getNumber());
    }

    public void setOuttakeSpeed() {
        this.setSpeed(outtakeSpeed.getNumber());
    }

    public void stop() {
        this.setSpeed(0);
    }

    public Command intakeCommand() {
        return new FunctionalCommand(
            () -> this.setIntakeSpeed(), 
            () -> {}, 
            (interrupted) -> this.stop(), 
            () -> this.spinMaster.aboveSpikeThreshold(), 
            this);
    }

    public Command startOuttakeCommand() {
        return Commands.runOnce(this::setOuttakeSpeed, this);
    }

    public Command stopCommand() {
        return Commands.runOnce(this::stop, this);
    }

    @Override
    public void periodic() {
        spinMaster.update();
    }

    public static Doohickey getInstance() {
        if (instance == null) instance = new Doohickey();
        return instance;
    }
}
