package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutputManager;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
        AutoLogOutputManager.addObject(this);
        // this.initialize();
    }

    /**
     * Prepare subsystems' hardware
     * @return a Command to do so
     */
    public Command normalizeCommand() {
        return new ParallelCommandGroup(
            this.endEffector.normalizeEndEffectorCommand(),
            // this.arm.goToPosition(Position.STOW),
            Commands.runOnce(() -> this.arm.setNormalizePosition(), this.arm),
            this.elevator.normalizeElevatorCommand(),
            this.intake.resetIntakePivot()
        );
    }

    public Command normalizeEFCommand() { // TODO Temp
        return this.endEffector.normalizeEndEffectorCommand();
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
            // this.endEffector.goToPosition(Position.STOW),
            this.endEffector.goToPosition(pos.get()),
            Commands.waitUntil(() -> this.arm.atTarget()),
            this.intake.stowIntakeCommand(),
            this.elevator.goToPosition(pos.get()),
            Commands.waitUntil(() -> this.elevator.atTarget()),
            this.arm.goToPosition(pos.get())
            // this.endEffector.goToPosition(pos.get())
        );
    }

    public Command goToReefPositionAuto(Supplier<Position> pos) {
        return Commands.sequence(
            Commands.print(pos.get().toString()),
            this.arm.goToPosition(Position.STOW),
            // this.endEffector.goToPosition(Position.STOW),
            this.endEffector.goToPosition(pos.get()),
            Commands.waitUntil(() -> this.arm.atTarget()),
            this.elevator.goToPosition(pos.get()),
            Commands.waitUntil(() -> this.elevator.atTarget()),
            this.arm.goToPosition(pos.get())
            // this.endEffector.goToPosition(pos.get())
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
            Commands.waitUntil(() -> !this.elevator.posBelowThreshold(pos) || this.arm.inSafeZone()),
            // this.intake.stowIntakeCommand(),
            this.elevator.goToPosition(pos),
            this.endEffector.goToPosition(pos)
        );
    }

    public Command goToGroundIntakePosition() {
        // return Commands.sequence(
            // this.intake.deployIntakeCommand(),
            // Commands.runOnce(() -> this.endEffector.setPreGroundIntakePosition(), this.endeffector),
            // Commands.waitUntil(() -> this.atTargets()),
            // Commands.waitUntil(() -> this.elevator.aboveGroundIntakeThreshold()),
            // Commands.waitUntil(() -> this.intake.pastIntakeDeployThreshold()),
            // this.endEffector.goToPosition(Position.CORAL_GROUND),
            // this.arm.goToPosition(Position.CORAL_GROUND),
            // Commands.waitUntil(() -> this.atTargets()),
            // this.elevator.goToPosition(Position.CORAL_GROUND)
        // );
        return Commands.sequence(
            this.intake.deployIntakeCommand(),
            Commands.either(
                Commands.runOnce(() -> {}), 
                Commands.runOnce(() -> this.elevator.setPreGroundIntakePosition(), this.elevator),
                () -> this.arm.belowFloorThreshold()),
            Commands.runOnce(() -> this.elevator.setPreGroundIntakePosition(), this.elevator),
            // Commands.waitUntil(() -> elevator.atTarget()),
            Commands.waitUntil(() -> this.elevator.aboveGroundIntakeThreshold()),
            Commands.waitUntil(() -> this.intake.pastIntakeDeployThreshold()),
            this.endEffector.goToPosition(Position.CORAL_GROUND),
            this.arm.goToPosition(Position.CORAL_GROUND),
            Commands.waitUntil(() -> this.atTargets()),
            this.elevator.goToPosition(Position.CORAL_GROUND)
        );
    }

    public Command goToGroundIntakePositionAuto() {
        return Commands.sequence(
            this.intake.deployIntakeCommand(),
            this.arm.goToPosition(Position.CORAL_GROUND),
            Commands.waitUntil(() -> arm.inSafeZone()),
            Commands.runOnce(() -> elevator.setPreGroundIntakePosition(), elevator),
            // Commands.waitUntil(() -> elevator.atTarget()),
            Commands.waitUntil(() -> elevator.aboveGroundIntakeThreshold()),
            Commands.waitUntil(() -> intake.pastIntakeDeployThreshold()),
            this.endEffector.goToPosition(Position.CORAL_GROUND),
            Commands.waitUntil(() -> this.atTargets()),
            this.elevator.goToPosition(Position.CORAL_GROUND)
        );
    }

    public Command goToBargePosition() {
        return Commands.sequence(
            this.elevator.goToPosition(Position.BARGE),
            Commands.waitUntil(() -> elevator.aboveBargeThreshold()),
            Commands.runOnce(() -> endEffector.setBargeInbetweenPosition(), endEffector),
            this.arm.goToPosition(Position.BARGE),
            Commands.waitUntil(() -> elevator.atTarget()),
            this.endEffector.goToPosition(Position.BARGE),
            this.intake.stowIntakeCommand()
        );
    }

    public Command goToAlgaeGroundCommand() {
        return Commands.sequence(
            Commands.either(
                Commands.sequence(
                    Commands.runOnce(() -> elevator.setPreGroundIntakePosition(), elevator),
                    Commands.waitUntil(() -> elevator.aboveGroundIntakeThreshold())
                    // Commands.waitUntil(() -> elevator.atTarget())
                ),
                Commands.runOnce(() -> {}), 
                () -> this.arm.belowFloorThreshold()),
            arm.goToPosition(Position.ALGAE_GROUND),
            endEffector.goToPosition(Position.ALGAE_GROUND),
            Commands.waitUntil(() -> this.arm.pastVerticalThreshold()),
            this.elevator.goToPosition(Position.ALGAE_GROUND)
        );
    }

    public Command goToProccessorPosition() {
        return Commands.sequence(
            Commands.either(
                Commands.sequence(
                    Commands.runOnce(() -> elevator.setPreGroundIntakePosition(), elevator),
                    Commands.waitUntil(() -> elevator.aboveGroundIntakeThreshold())
                    // Commands.waitUntil(() -> elevator.atTarget())
                ),
                Commands.runOnce(() -> {}), 
                () -> this.arm.belowFloorThreshold()),
            arm.goToPosition(Position.PROCESSOR),
            endEffector.goToPosition(Position.PROCESSOR),
            Commands.waitUntil(() -> this.arm.pastVerticalThreshold()),
            this.elevator.goToPosition(Position.PROCESSOR)
        );
    }

    public Command intakeAlgaeGroundCommand() {
        // return Commands.runOnce(() -> this.endEffector.intakeGroundAlgae(), this.endEffector);
        return Commands.sequence(
            this.goToAlgaeGroundCommand(),
            endEffector.intakeGroundAlgae()
        );
    }

    public Command removeAlgaeCommand() {
        return Commands.runOnce(() -> this.endEffector.removeAlgae(), this.endEffector);
    }

    @Deprecated
    public Command goToIntakePosition() {
        // return Commands.sequence(
        //     this.elevator.goToPosition(Position.CORAL_GROUND),
        //     this.intake.deployIntakeCommand(),
        //     this.endEffector.goToPosition(Position.CORAL_GROUND),
        //     Commands.waitUntil(() -> this.intake.pastIntakeDeployThreshold()),
        //     Commands.waitUntil(() -> this.elevator.aboveGroundIntakeThreshold()),
        //     this.arm.goToPosition(Position.CORAL_GROUND)
        // );
        return this.goToGroundIntakePosition();
    }

    @Deprecated
    public Command goToIntakePositionAuto() {
        // return Commands.sequence(
        //     this.arm.goToPosition(Position.CORAL_GROUND),
        //     this.intake.deployIntakeCommand(),
        //     this.endEffector.goToPosition(Position.CORAL_GROUND),
        //     Commands.waitUntil(() -> arm.inSafeZone()),
        //     Commands.waitUntil(() -> this.intake.pastIntakeDeployThreshold()),
        //     this.elevator.goToPosition(Position.CORAL_GROUND)
        // );
        return this.goToGroundIntakePositionAuto();
    }

    public Command stowCommand() {
        return Commands.either(
            stowAlgaeCommand(), 
            stowCoralCommand(), 
            () -> this.hasAlgae());
    }

    public Command stowCoralCommand() {
        return Commands.sequence(
            this.intake.stopIntakeCommand(),
            this.endEffector.stopCommand(),
            Commands.either(
                Commands.sequence(
                    Commands.runOnce(() -> elevator.setPreGroundIntakePosition(), elevator),
                    Commands.waitUntil(() -> elevator.aboveGroundIntakeThreshold())
                    // Commands.waitUntil(() -> elevator.atTarget())
                ),
                Commands.runOnce(() -> {}), 
                () -> this.arm.belowFloorThreshold()),
            this.arm.goToPosition(Position.STOW),
            this.endEffector.goToPosition(Position.STOW),
            // new WaitUntilCommand(() -> !(this.elevator.posBelowThreshold(Position.STOW) && this.arm.belowFloorThreshold())),
            // new WaitUntilCommand(() -> !this.elevator.posBelowThreshold(Position.STOW) || this.arm.inSafeZone()),
            Commands.waitUntil(() -> this.arm.inSafeZone()),
            this.elevator.goToPosition(Position.STOW),
            this.intake.stowIntakeCommand()
        );
    }

    public Command stowAlgaeCommand() {
        return Commands.sequence(
            // this.endEffector.goToPosition(Position.STOW),
            Commands.runOnce(() -> arm.setAlgaeStowPosition(), arm),
            Commands.waitUntil(() -> arm.atTarget()),
            Commands.runOnce(() -> endEffector.setAlgaeStowPosition()),
            this.elevator.goToPosition(Position.STOW)
        );
    }

    public Command stowReefCommand() {
        return new SequentialCommandGroup(
            this.endEffector.stopCommand(),
            this.intake.stopIntakeCommand(),
            Commands.either(
                Commands.sequence(
                    Commands.runOnce(() -> elevator.setPreGroundIntakePosition(), elevator),
                    Commands.waitUntil(() -> elevator.aboveGroundIntakeThreshold())
                    // Commands.waitUntil(() -> elevator.atTarget())
                ),
                Commands.runOnce(() -> {}), 
                () -> this.arm.belowFloorThreshold()),
            this.arm.goToPosition(Position.STOW),
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
            Commands.select(
                Map.ofEntries(
                    Map.entry(Position.L1, this.endEffector.goToPosition(Position.L1)),
                    Map.entry(Position.L2, this.endEffector.goToPosition(Position.L2)),
                    Map.entry(Position.L3, this.endEffector.goToPosition(Position.L3)),
                    Map.entry(Position.L4, this.endEffector.goToPosition(Position.L4)),
                    Map.entry(Position.STOW, this.endEffector.goToPosition(Position.STOW))
                ), 
                () -> this.getRequestedScoringPosition())
            // this.endEffector.goToPosition(Position.STOW),
        );
    }

    public Command stowReefAutoCommand() {
        return new SequentialCommandGroup(
            this.intake.stopIntakeCommand(),
            this.arm.goToPosition(Position.STOW),
            // new WaitUntilCommand(() -> !(this.elevator.posBelowThreshold(Position.STOW) && this.arm.belowFloorThreshold())),
            // new WaitUntilCommand(() -> !(this.elevator.posBelowThreshold(Position.STOW) && !this.arm.inSafeZone())),
            Commands.waitUntil(() -> this.arm.inSafeZone()),
            Commands.select(
                Map.ofEntries(
                    Map.entry(Position.L1, Commands.runOnce(() -> this.elevator.setL1Stow(), elevator)),
                    Map.entry(Position.L2, Commands.runOnce(() -> this.elevator.setL2Stow(), elevator)),
                    Map.entry(Position.L3, Commands.runOnce(() -> this.elevator.setL3Stow(), elevator)),
                    Map.entry(Position.L4, Commands.runOnce(() -> this.elevator.setL4Stow(), elevator)),
                    Map.entry(Position.STOW, this.elevator.goToPosition(Position.STOW))
                ), 
                () -> this.getRequestedScoringPosition()),
            Commands.select(
                Map.ofEntries(
                    Map.entry(Position.L1, this.endEffector.goToPosition(Position.L1)),
                    Map.entry(Position.L2, this.endEffector.goToPosition(Position.L2)),
                    Map.entry(Position.L3, this.endEffector.goToPosition(Position.L3)),
                    Map.entry(Position.L4, this.endEffector.goToPosition(Position.L4)),
                    Map.entry(Position.STOW, this.endEffector.goToPosition(Position.STOW))
                ), 
                () -> this.getRequestedScoringPosition()),
            // this.endEffector.goToPosition(Position.STOW),
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
        // return this.goToPosition(Position.SOURCE);
        return Commands.sequence(
            this.elevator.goToPosition(Position.SOURCE),
            this.endEffector.goToPosition(Position.SOURCE),
            Commands.waitUntil(() -> this.elevator.aboveSourceIntakeThreshold()),
            this.arm.goToPosition(Position.SOURCE)
            // new WaitUntilCommand(() -> !(this.elevator.posBelowThreshold(Position.SOURCE) && this.arm.belowFloorThreshold())),
            // new WaitUntilCommand(() -> !(this.elevator.posBelowThreshold(Position.SOURCE) && !this.arm.inSafeZone())),
            // this.intake.stowIntakeCommand(),
        );
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
    public Command intakeGroundAlgaeEndeffector() {
        return this.endEffector.intakeGroundAlgae();
    }

    /**
     * Intake Algae to EndEffector
     * @return a Command to do so
     */
    public Command removeAlgaeEndeffector() {
        return this.endEffector.removeAlgae();
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

    public Command setEndEffectorAlgaeRemovalSpeedCommand() {
        return this.endEffector.setAlgaeRemovalSpeedCommand();
    }

    public Command goToIntakeL1Position() {
        return Commands.sequence(
            Commands.runOnce(() -> intake.setIntakeDeploy(), intake)
            // Commands.waitUntil(() -> intake.atPositionTarget())
        );
    }

    /**
     * Check if subsystems are at target positions
     * @return true if subsystems are on target
     */
    public boolean atTargets() {
        return this.elevator.atTarget() && this.arm.atTarget() && this.endEffector.atTarget();
    }

    public boolean hasAlgae() {
        return endEffector.algaeDetected();
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