package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intake;

/* TODO
 * Tune scoreBarge delays
 */

public class Superstructure {
    private Elevator elevator = Elevator.getInstance();
    private Arm arm = Arm.getInstance();
    private EndEffector endEffector = EndEffector.getInstance();

    private Intake intake = Intake.getInstance();
    
    private static Superstructure instance = null;

    private Position requestedScoringPosition = Position.STOW;

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
        BARGE,
        L2_ALGAE,
        L3_ALGAE
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
            this.elevator.normalizeElevatorCommand(),
            this.intake.resetIntakePivot()
        );
    }

    public Command normalizeEFCommand() { // TODO Temp
        return this.endEffector.normalizeEndEffectorCommand();
    }

    public Command normalizeECommand() { // TODO Temp
        return this.elevator.normalizeElevatorCommand();
    }

    public void setRequestedScoringPosition(Position pos) {
        this.requestedScoringPosition = pos;
    }

    public Position getRequestedScoringPosition() {
        return this.requestedScoringPosition;
    }

    public Command goToReefPosition(Position pos) {
        // return Commands.sequence(
        //     this.arm.goToPosition(Position.STOW),
        //     this.endEffector.goToPosition(Position.STOW),
        //     Commands.waitUntil(() -> this.arm.atTarget()),
        //     this.intake.stowIntakeCommand(),
        //     this.elevator.goToPosition(pos),
        //     Commands.waitUntil(() -> this.elevator.atTarget()),
        //     this.arm.goToPosition(pos),
        //     this.endEffector.goToPosition(pos)
        // );
        return this.goToReefPosition(() -> pos);
    }

    public Command goToReefPosition(Supplier<Position> pos) {
        return Commands.sequence(
            Commands.print(pos.get().toString()),
            this.arm.goToPosition(Position.STOW),
            this.endEffector.goToPosition(Position.STOW),
            Commands.waitUntil(() -> this.arm.atTarget()),
            this.intake.stowIntakeCommand(),
            this.elevator.goToPosition(pos.get()),
            Commands.waitUntil(() -> this.elevator.atTarget()),
            this.arm.goToPosition(pos.get()),
            this.endEffector.goToPosition(pos.get())
        );
    }

    public Command setRequestedScoringPositionCommand(Position pos) {
        return Commands.runOnce(() -> this.setRequestedScoringPosition(pos));
    }

    public Command goToRequestedPositionCommand() {
        return this.goToReefPosition(() -> this.getRequestedScoringPosition());
    }

    /**
     * Move subsystems to a Position
     * @param pos the position to move to
     * @return a Command to do so
     */
    public Command goToPosition(Position pos) {
        return new SequentialCommandGroup(
            this.arm.goToPosition(pos),
            // new WaitUntilCommand(() -> !(this.elevator.posBelowThreshold(pos) && this.arm.belowFloorThreshold())),
            // new WaitUntilCommand(() -> !(this.elevator.posBelowThreshold(pos) && !this.arm.inSafeZone())),
            Commands.waitUntil(() -> this.arm.inSafeZone()),
            // this.intake.stowIntakeCommand(),
            this.elevator.goToPosition(pos),
            this.endEffector.goToPosition(pos)
        );
    }

    public Command goToAlgaeGroundCommand() {
        return this.goToPosition(Position.ALGAE_GROUND);
    }

    public Command intakeAlgaeGroundCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> endEffector.setAlgaeIntakeSpeed(), endEffector),
            this.goToAlgaeGroundCommand(),
            Commands.waitUntil(() -> endEffector.currentSpike()),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> endEffector.setAlgaeHoldSpeed(), endEffector)
        );
    }

    public Command outtakeAlgaeWithArm() {
        return Commands.sequence(
            Commands.runOnce(() -> endEffector.setAlgaeOuttakeSpeed(), endEffector),
            Commands.runOnce(() -> endEffector.setAlgaeOuttakePosition(), endEffector),
            Commands.runOnce(() -> arm.setAlgaeOuttakePosition(), arm)
        );
    }

    public Command goToIntakePosition() {
        return Commands.sequence(
            this.intake.deployIntakeCommand(),
            this.endEffector.goToPosition(Position.CORAL_GROUND),
            this.elevator.goToPosition(Position.CORAL_GROUND),
            Commands.waitUntil(() -> this.elevator.aboveGroundIntakeThreshold()),
            this.arm.goToPosition(Position.CORAL_GROUND)
        );
    }


    public Command stowCommand() {
        return new SequentialCommandGroup(
            this.arm.goToPosition(Position.STOW),
            this.intake.stopIntakeCommand(),
            // new WaitUntilCommand(() -> !(this.elevator.posBelowThreshold(Position.STOW) && this.arm.belowFloorThreshold())),
            // new WaitUntilCommand(() -> !(this.elevator.posBelowThreshold(Position.STOW) && !this.arm.inSafeZone())),
            Commands.waitUntil(() -> this.arm.inSafeZone()),
            this.intake.stowIntakeCommand(),
            this.elevator.goToPosition(Position.STOW),
            this.endEffector.goToPosition(Position.STOW),
            this.endEffector.stopCommand()
        );
    }

    public Command stowReefCommand() {
        return new SequentialCommandGroup(
            this.arm.goToPosition(Position.STOW),
            this.intake.stopIntakeCommand(),
            // new WaitUntilCommand(() -> !(this.elevator.posBelowThreshold(Position.STOW) && this.arm.belowFloorThreshold())),
            // new WaitUntilCommand(() -> !(this.elevator.posBelowThreshold(Position.STOW) && !this.arm.inSafeZone())),
            Commands.waitUntil(() -> this.arm.inSafeZone()),
            this.intake.stowIntakeCommand(),
            Commands.select(
                Map.ofEntries(
                    Map.entry(Position.L1, Commands.runOnce(() -> this.elevator.setL1Stow(), elevator)),
                    Map.entry(Position.L2, Commands.runOnce(() -> this.elevator.setL2Stow(), elevator)),
                    Map.entry(Position.L3, Commands.runOnce(() -> this.elevator.setL3Stow(), elevator)),
                    Map.entry(Position.L4, Commands.runOnce(() -> this.elevator.setL4Stow(), elevator)),
                    Map.entry(Position.STOW, this.elevator.goToPosition(Position.STOW))
                ), 
                () -> this.getRequestedScoringPosition()),
            this.endEffector.goToPosition(Position.STOW),
            this.endEffector.stopCommand()
        );
    }

    public Command goToL4Command() {
        return this.goToReefPosition(Position.L4);
    }

    public Command goToL3Command() {
        return this.goToReefPosition(Position.L3);
    }

    public Command goToL2Command() {
        return this.goToReefPosition(Position.L2);
    }

    public Command goToL1Command() {
        return this.goToReefPosition(Position.L1);
    }

    public Command goToSourceIntakePosition() {
        return this.goToPosition(Position.SOURCE);
    }

    public Command goToL2RemoveCommand() {
        return this.goToReefPosition(Position.L2_ALGAE);
    }

    public Command goToL3RemoveCommand() {
        return this.goToReefPosition(Position.L3_ALGAE);
    }

    /**
     * Intake Coral to EndEffector
     * @return a Command to do so
     */
    public Command intakeCoral() {
        return this.endEffector.intakeCoral();
    }

    /**
     * Intake Algae to EndEffector
     * @return a Command to do so
     */
    public Command intakeAlgaeEndeffector() {
        return this.endEffector.intakeAlgae();
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

    public Command setEndEfffectorAlgaeRemovalSpeedCommand() {
        return this.endEffector.setAlgaeRemovalSpeedCommand();
    }

    public Command goToIntakeL1Position() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.setIntakeDeploy(), intake)
            // Commands.waitUntil(() -> intake.atPositionTarget())
        );
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

    public void update() {
        SmartDashboard.putString("requested-position", this.requestedScoringPosition.toString());
        SmartDashboard.putBoolean("at-targets", this.atTargets());
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