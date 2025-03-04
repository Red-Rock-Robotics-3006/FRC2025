package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
 * Find positions & combine accordingly
 * Tune Slot0s
 * Tune MMs
 * Tune spikeThresholds
 * Tune outtake delays
 * Tune scoreBarge
 * Implement double-jointed Slot0 angle sensing (How?)
 */

public class EndEffector extends SubsystemBase {
    public static final double kCoralOuttakeWaitTime = 0.2;
    public static final double kAlgaeOUttakeWaitTime = 0.2;

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

    private static EndEffector instance = null;

    private SmartDashboardNumber l1Position = new SmartDashboardNumber("endeffector/position/endeffector-l1", 0);
    private SmartDashboardNumber l2Position = new SmartDashboardNumber("endeffector/position/endeffector-l2", 0);
    private SmartDashboardNumber l3Position = new SmartDashboardNumber("endeffector/position/endeffector-l3", 0);
    private SmartDashboardNumber l4Position = new SmartDashboardNumber("endeffector/position/endeffector-l4", 0);
    private SmartDashboardNumber sourcePosition = new SmartDashboardNumber("endeffector/position/endeffector-source", 0);
    private SmartDashboardNumber coralGroundPosition = new SmartDashboardNumber("endeffector/position/endeffector-coral-ground", 0);
    private SmartDashboardNumber algaeGroundPosition = new SmartDashboardNumber("endeffector/position/endeffector-algae-ground", 0);
    private SmartDashboardNumber processorPosition = new SmartDashboardNumber("endeffector/position/endeffector-processor", 0);
    private SmartDashboardNumber stowPosition = new SmartDashboardNumber("endeffector/position/endeffector-stow", 0);
    private SmartDashboardNumber bargePosition = new SmartDashboardNumber("endeffector/position/endeffector-barge", 0);
    

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
     * Converts a Position to its corresponding value
     * @param pos the Position to convert
     * @return the numerical value
     */
    private double convertPosition(Position pos)
    {
        switch (pos) {
            case L1:
                return this.l1Position.getNumber();
            case L2:
                return this.l2Position.getNumber();
            case L3:
                return this.l3Position.getNumber();
            case L4:
                return this.l4Position.getNumber();
            case SOURCE:
                return this.sourcePosition.getNumber();
            case CORAL_GROUND:
                return this.coralGroundPosition.getNumber();
            case ALGAE_GROUND:
                return this.algaeGroundPosition.getNumber();
            case PROCESSOR:
                return this.processorPosition.getNumber();
            default: // Unreachable; Just to keep the compiler from complaining
            case STOW:
                return this.stowPosition.getNumber();
            case BARGE:
                return this.bargePosition.getNumber();
        }
    }

    /**
     * Set the drive speed to a specified power
     * @param speed the power to drive at
     */
    private void setSpeed(double speed){
        this.driveMotor.motor.setControl(
            new DutyCycleOut(speed)
        );
    }

    public void setCoralIntakeSpeed() {
        this.setSpeed(this.coralIntakeSpeed.getNumber());
    }

    public void setCoralOuttakeSpeed() {
        this.setSpeed(this.coralOuttakeSpeed.getNumber());
    }

    public void setAlgaeIntakeSpeed() {
        this.setSpeed(this.algaeIntakeSpeed.getNumber());
    }

    public void setAlgaeOuttakeSpeed() {
        this.setSpeed(this.algaeOuttakeSpeed.getNumber());
    }

    public void setNormalizeSpeed() {
        this.driveMotor.motor.setControl(new DutyCycleOut(this.normalizeSpeed.getNumber()));
    }

    public void stop() {
        this.driveMotor.motor.setControl(new DutyCycleOut(0));
    }

    public void resetWrist() {
        this.wristMotor.motor.setControl(new CoastOut());
        this.wristMotor.motor.setPosition(0);
    }
    
    /**
     * Check if the endeffector is at target position
     * @return true if the endeffector is on target
     */
    public boolean atTarget(){
        // return Math.abs(this.convertPosition(this.targetPosition)
        //     - this.wristMotor.motor.getPosition().getValueAsDouble()) < this.wristTolerance.getNumber();
        return this.driveMotor.motor.getClosedLoopError().getValueAsDouble() < this.wristTolerance.getNumber() 
            || Math.abs(this.convertPosition(this.targetPosition) - this.wristMotor.motor.getPosition().getValueAsDouble()) < this.wristTolerance.getNumber();
    }

    @Override
    public void periodic() {
        this.driveMotor.update();
        this.wristMotor.update();
    }

    /**
     * Move the endeffector to a Position
     * @param pos the Position to move to
     * @return a Command to do so
     */
    public Command goToPosition(Position pos){
        this.targetPosition = pos;
        return Commands.runOnce(
            () -> this.wristMotor.setMotionMagicPosition(this.convertPosition(pos)),
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
            () -> this.setCoralIntakeSpeed(),
            () -> {},
            (interrupted) -> this.stop(),
            () -> this.coralDetected(),
            this
        );
    }

    /**
     * Auto intake Algae
     * @return a Command to do so
     */
    public Command intakeAlgae(){
        return new FunctionalCommand(
            () -> this.setSpeed(this.algaeIntakeSpeed.getNumber()),
            () -> {},
            (interrupted) -> this.stop(),
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
            new InstantCommand(this::setCoralOuttakeSpeed, this),
            new WaitUntilCommand(() -> !this.coralDetected()),
            new WaitCommand(kCoralOuttakeWaitTime),
            this.stopCommand()
        );
    }

    /**
     * Auto dispense Algae
     * @return a Command to do so
     */
    public Command outtakeAlgae(){
        return new SequentialCommandGroup(
            new InstantCommand(this::setAlgaeIntakeSpeed, this),
            new WaitCommand(kAlgaeOUttakeWaitTime),
            this.stopCommand()
        );
    }

    public Command stopCommand(){
        return this.runOnce(this::stop);
    }

    /**
     * Move the endeffector to a normal position and zero it
     * @return a Command to do so
     */
    public Command normalizeEndEffectorCommand(){
        return new SequentialCommandGroup(
            new FunctionalCommand(
                () -> this.setNormalizeSpeed(),
                () -> {},
                (interrupted) -> this.resetWrist(),
                () -> this.wristMotor.aboveSpikeThreshold(),
                this
            )
            // ,
            // this.goToPosition(this.targetPosition)
        );
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
