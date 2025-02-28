package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import redrocklib.wrappers.RedRockTalon;
import redrocklib.logging.SmartDashboardNumber;
import frc.robot.Superstructure.Position;

/* TODO
 * Tune speeds
 * Tune tolerance
 * Find positions
 * Tune Slot0s
 * Tune MMs
 * Tune spikeThresholds
 * Tune outtake delays
 * Tune scoreBarge
 * Implement double-jointed Slot0 angle sensing (How?)
 */

public class EndEffector extends SubsystemBase {
    private final RedRockTalon driveMotor = new RedRockTalon(51,"endeffector-drive","*");
    private final RedRockTalon wristMotor = new RedRockTalon(52,"endeffector-wrist","*");
    private final CANrange timeOfFlight = new CANrange(53);

    private SmartDashboardNumber coralIntakeSpeed = new SmartDashboardNumber("endeffector/coral-intake-speed", 0);
    private SmartDashboardNumber coralOuttakeSpeed = new SmartDashboardNumber("endeffector/coral-outtake-speed", 0);
    private SmartDashboardNumber tofThreshold = new SmartDashboardNumber("endeffector/coral-threshold", 0);
    private SmartDashboardNumber normalizeSpeed = new SmartDashboardNumber("endeffector/normalize-speed", -0.1);
    private SmartDashboardNumber algaeIntakeSpeed = new SmartDashboardNumber("endeffector/algae-intake-speed", -0.2);
    private SmartDashboardNumber algaeOuttakeSpeed = new SmartDashboardNumber("endeffector/algae-outtake-speed", 0);
    private SmartDashboardNumber wristTolerance = new SmartDashboardNumber("endeffector/wrist-tolerance", 0.1);
    
    private Position targetPosition = Position.STOW;
    private boolean running = false;

    private static EndEffector instance = null;

    private static Map<Position, SmartDashboardNumber > POSITION_CONVERSIONS = Map.of(
        Position.L4, new SmartDashboardNumber("endeffector/position/endeffector-l4", 0),
        Position.L3, new SmartDashboardNumber("endeffector/position/endeffector-l3", 0),
        Position.L2, new SmartDashboardNumber("endeffector/position/endeffector-l2", 0),
        Position.L1, new SmartDashboardNumber("endeffector/position/endeffector-l1", 0),
        Position.SOURCE, new SmartDashboardNumber("endeffector/position/endeffector-source", 0),
        Position.CORAL_GROUND, new SmartDashboardNumber("endeffector/position/endeffector-coral-ground", 0),
        Position.ALGAE_GROUND, new SmartDashboardNumber("endeffector/position/endeffector-algae-ground", 0),
        Position.PROCESSOR, new SmartDashboardNumber("endeffector/position/endeffector-processor", 0),
        Position.STOW, new SmartDashboardNumber("endeffector/position/endeffector-stow", 0),
        Position.BARGE, new SmartDashboardNumber("endeffector/position/endeffector-barge", 0)
    );

    private EndEffector(){
        super("End Effector");
        this.driveMotor.withMotorOutputConfigs(
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
        .withSpikeThreshold(55);
                
        this.wristMotor.withMotorOutputConfigs(
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
        .withSpikeThreshold(55);       
    }

    /**
     * Set the drive speed to a specified power
     * @param speed the power to drive at
     */
    private void setSpeed(double speed){
        if(speed == 0)
            this.running = false;
        else
            this.running = true;
        this.driveMotor.motor.set(speed);
    }
    
    /**
     * Check if the endeffector is at target position
     * @return true if the endeffector is on target
     */
    public boolean atTarget(){
        return Math.abs(POSITION_CONVERSIONS.get(this.targetPosition).getNumber()
            - this.wristMotor.motor.getPosition().getValueAsDouble()) < this.wristTolerance.getNumber();
    }

    /**
     * Move the endeffector to a Position
     * @param pos the Position to move to
     * @return a Command to do so
     */
    public Command goToPosition(Position pos){
        this.targetPosition = pos;
        return Commands.runOnce(
            () -> this.wristMotor.setMotionMagicPosition(POSITION_CONVERSIONS.get(pos).getNumber()),
            this);
    }

    /**
     * Dispense Algae with momentum into the Barge
     * @return
     */
    public Command scoreBarge(){
        return new ParallelCommandGroup(
            this.goToPosition(Position.PROCESSOR), // Should open up algae
            this.outtakeAlgae()
        ); // Will have to be tuned experimentally
    };

    /**
     * Detect if Coral is present in the endeffector
     * @return true if coral is present
     */
    private boolean coralDetected(){
        return this.timeOfFlight.getDistance().getValueAsDouble() < this.tofThreshold.getNumber();
    }

    /**
     * Auto intake Coral
     * @return a Command to do so
     */
    public Command intakeCoral(){
        return new FunctionalCommand(
            () -> setSpeed(this.coralIntakeSpeed.getNumber()),
            () -> {},
            (interrupted) -> setSpeed(0),
            () -> coralDetected(),
            this
        );
    }

    /**
     * Auto intake Algae
     * @return a Command to do so
     */
    public Command intakeAlgae(){
        return new FunctionalCommand(
            () -> setSpeed(this.algaeIntakeSpeed.getNumber()),
            () -> {},
            (interrupted) -> setSpeed(0),
            () -> this.driveMotor.aboveSpikeThreshold(),
            this
        );
    }

    /**
     * Auto dispense Coral
     * @return a Command to do so
     */
    public Command outtakeCoral(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setSpeed(this.coralOuttakeSpeed.getNumber())),
            new WaitUntilCommand(() -> !this.coralDetected()),
            new WaitCommand(.2),
            Commands.runOnce(() -> setSpeed(0))
        );
    }

    /**
     * Auto dispense Algae
     * @return a Command to do so
     */
    public Command outtakeAlgae(){
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setSpeed(this.algaeOuttakeSpeed.getNumber())),
            new WaitCommand(.25),
            Commands.runOnce(() -> setSpeed(0))
        );
    }

    /**
     * Move the endeffector to a normal position and zero it
     * @return a Command to do so
     */
    public Command normalizeCommand(){
        return new SequentialCommandGroup(
            new FunctionalCommand(
                () -> setSpeed(this.normalizeSpeed.getNumber()),
                () -> {},
                (interrupted) -> {setSpeed(0);
                    this.wristMotor.motor.setPosition(0);
                },
                () -> this.wristMotor.aboveSpikeThreshold(),
                this
            ),
            this.goToPosition(this.targetPosition)
        );
    }

    /**
     * Check if the endeffector has finished control
     * @return true if the endeffector is idle
     */
    public boolean isIdle()
    {
        return this.atTarget() && !this.running;
    }

    /**
     * Get singleton instance
     * @return the EndEffector
     */
    public static EndEffector getInstance()
    {
        if(instance == null)
            instance = new EndEffector();
        return instance;
    }
}
