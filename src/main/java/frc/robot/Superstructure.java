package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;

/* TODO
 * Tune scoreBarge delays
 */

public class Superstructure {
    private Elevator elevator = Elevator.getInstance();
    private Arm arm = Arm.getInstance();
    private EndEffector endEffector = EndEffector.getInstance();
    
    private static Superstructure instance = null;

    public static enum Position {
        L1,
        L2,
        L3,
        L4,
        SOURCE,
        CORAL_GROUND,
        ALGAE_GROUND,
        PROCESSOR,
        STOW,
        BARGE
    }

    private Superstructure() {
        // this.initialize();
    }

    /**
     * Prepare subsystems' hardware
     * @return a Command to do so
     */
    public Command normalizeCommand() {
        return new ParallelCommandGroup(
            this.endEffector.normalizeEndEffectorCommand(),
            this.arm.goToPosition(Position.STOW),
            this.elevator.normalizeElevatorCommand()
        );
    }

    /**
     * Move subsystems to a Position
     * @param pos the position to move to
     * @return a Command to do so
     */
    public Command goToPosition(Position pos) {
        return new SequentialCommandGroup(
            this.arm.goToPosition(pos),
            new WaitUntilCommand(() -> !(this.elevator.posBelowThreshold(pos) && this.arm.belowFloorThreshold())),
            this.elevator.goToPosition(pos),
            this.endEffector.goToPosition(pos)
        );
    }

    /**
     * Intake Coral to EndEffector
     * @return a Command to do so
     */
    public Command intakeCoral() {
        return new InstantCommand(
            () -> this.endEffector.intakeCoral(),
            this.endEffector
        );    
    }

    /**
     * Intake Algae to EndEffector
     * @return a Command to do so
     */
    public Command intakeAlgae() {
        return new InstantCommand(
            () -> this.endEffector.intakeAlgae(),
            this.endEffector
        );
    }

    /**
     * Dispense Coral from EndEffector
     * @return a Command to do so
     */
    public Command outtakeCoral() {
        return this.endEffector.outtakeCoral();
    }

    /**
     * Dispense Algae from EndEffector
     * @return a Command to do so
     */
    public Command outtakeAlgae() {
        return this.endEffector.outtakeAlgae();
    }

    /**
     * Stop the endeffector
     * @return a Command to do so
     */
    public Command stopEndEffector() {
        return this.endEffector.stopCommand();
    }

    /**
     * Abstracted full Barge scoring
     * @return a Command to do so
     */
    public Command autoScoreBarge() {
        return new SequentialCommandGroup(
            this.goToPosition(Position.BARGE),
            new WaitUntilCommand(() -> this.atTargets()),
            new WaitCommand(.1),
            this.arm.scoreBarge(),
            new WaitCommand(.3),
            this.endEffector.scoreBarge()
        );
    }

    /**
     * Check if subsystems are at target positions
     * @return true if subsystems are on target
     */
    public boolean atTargets() {
        return this.elevator.atTarget() && this.arm.atTarget() && this.endEffector.atTarget();
    }

    /**
     * Abstracted full Coral scoring
     * @param pos the Position to score Coral at
     * @return a Command to do so
     */
    public Command autoScoreCoral(Position pos) {
        return Commands.sequence(
            this.goToPosition(pos),
            new WaitUntilCommand(() -> this.atTargets()),
            this.endEffector.outtakeCoral(),
            this.goToPosition(Position.STOW)
        );
    }

    /**
     * Abstracted full Proc scoring
     * @return a Command to do so
     */
    public Command autoScoreProcessor() {
        return Commands.sequence(
            this.goToPosition(Position.PROCESSOR),
            new WaitUntilCommand(() -> this.atTargets()),
            this.endEffector.outtakeAlgae(),
            this.goToPosition(Position.STOW)
        );
    }
    
    /**
     * Get singleton instance
     * @return the Superstructure
     */
    public static Superstructure getInstance() {
        if(instance == null)
            instance = new Superstructure();
        return instance;
    }
}