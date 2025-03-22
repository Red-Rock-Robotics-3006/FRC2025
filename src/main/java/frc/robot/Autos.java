package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Superstructure.Position;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Autos {
    private final AutoFactory factory;

    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Intake intake = Intake.getInstance();
    private final EndEffector endeffector = EndEffector.getInstance();

    public Autos(AutoFactory f) {
        factory = f;
    }

    public Command TESTPATH1() {
        return 
        Commands.sequence(
            followTrajectoryCommand("TESTPATH1", 0),
            followTrajectoryCommand("TESTPATH1", 0)
        );
    }

    public Command TESTPATH2() {
        return followTrajectoryCommand("TESTPATH2", 0);
    }

    public Command left3L4Paths() {
        return Commands.sequence(
            followTrajectoryCommand("L3GS", 0),
            Commands.waitSeconds(1.5),
            followTrajectoryCommand("L3GS", 1),
            Commands.waitSeconds(1.5),
            followTrajectoryCommand("L3GS", 2),
            Commands.waitSeconds(1.5),
            followTrajectoryCommand("L3GS", 3),
            Commands.waitSeconds(1.5),
            followTrajectoryCommand("L3GS", 4)
        );
    }

    public Command right3L4Paths() {
        return Commands.sequence(
            followTrajectoryCommand("R3GS", 0),
            Commands.waitSeconds(1.5),
            followTrajectoryCommand("R3GS", 1),
            Commands.waitSeconds(1.5),
            followTrajectoryCommand("R3GS", 2),
            Commands.waitSeconds(1.5),
            followTrajectoryCommand("R3GS", 3),
            Commands.waitSeconds(1.5),
            followTrajectoryCommand("R3GS", 4)
        );
    }
    
    public Command left3L4() {
        return Commands.sequence(
            superstructure.setRequestedScoringPositionCommand(Position.L4),

            followTrajectoryCommand("L3GS", 0),
            goToScoreAutoCommand(1),
            superstructure.outtakeCoral(),

            Commands.deadline(
                Commands.sequence(
                    followTrajectoryCommand("L3GS", 1),
                    followTrajectoryCommand("L3GS", 2)
                ),
                groundIntakeCommand()
            ),  
            Commands.either(
                Commands.sequence(
                    goToScoreAutoCommand(0), 
                    superstructure.outtakeCoral()
                ),
                Commands.print("SECOND CORAL MISSED"), 
                () -> endeffector.coralDetected()
            ),

            Commands.parallel(
                Commands.sequence(
                    followTrajectoryCommand("L3GS", 3),
                    followTrajectoryCommand("L3GS", 4)
                ),
                groundIntakeCommand()
            ),  
            goToScoreAutoCommand(1),
            superstructure.outtakeCoral()
        );
    }

    public Command right3L4() {
        return Commands.sequence(
            superstructure.setRequestedScoringPositionCommand(Position.L4),
            
            followTrajectoryCommand("R3GS", 0),
            goToScoreAutoCommand(0),
            superstructure.outtakeCoral(),

            Commands.deadline(
                Commands.sequence(
                    followTrajectoryCommand("R3GS", 1),
                    followTrajectoryCommand("R3GS", 2)
                ),
                groundIntakeCommand()
            ),  
            Commands.either(
                Commands.sequence(
                    goToScoreAutoCommand(1), 
                    superstructure.outtakeCoral()
                ),
                Commands.print("SECOND CORAL MISSED"), 
                () -> endeffector.coralDetected()
            ),

            Commands.parallel(
                Commands.sequence(
                    followTrajectoryCommand("R3GS", 3),
                    followTrajectoryCommand("R3GS", 4)
                ),
                groundIntakeCommand()
            ),  
            goToScoreAutoCommand(0),
            superstructure.outtakeCoral()
        );
    }

    public Command goToScoreAutoCommand(Pose2d pose) {
        return Commands.parallel(
            Commands.sequence(
                superstructure.goToL4Command(),
                Commands.waitUntil(() -> superstructure.atTargets()),
                Commands.waitSeconds(0.75)
            ),
            Commands.sequence(
                Commands.runOnce(() -> {drivetrain.setTargetPose(pose); drivetrain.enablePositionTargeting();}, drivetrain),
                drivetrain.setNearestRequestedReefPoseTargetCommand(),
                Commands.deadline(
                    Commands.waitSeconds(0.5), 
                    drivetrain.pidToPoseContinuousCommand()
                ),
                Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain)
            )
        );
    }

    public Command goToScoreAutoCommand(int reefSide) {
        return Commands.sequence(
            Commands.deadline(
                Commands.sequence(
                    superstructure.goToL4Command(),
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitSeconds(0.4)
                ),
                Commands.sequence(
                    Commands.runOnce(() -> {drivetrain.setReefSide(reefSide); drivetrain.setNearestRequestedReefPoseTarget(); drivetrain.enablePositionTargeting();}, drivetrain),
                    drivetrain.pidToPoseContinuousCommand()
                )
            ),
            drivetrain.disablePositionTargetingCommand()
        );
    }

    public Command groundIntakeCommand() {
        return Commands.sequence(
            superstructure.goToIntakePosition(),
            Commands.deadline(
                superstructure.intakeCoral(),
                intake.spasmIntakeCommand()
            ),
            superstructure.stowReefCommand()
        );
    }

    public Command goToSourceIntakeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {drivetrain.setNearestSourcePose(); drivetrain.enablePositionTargeting();}, drivetrain),
            Commands.deadline(
                Commands.sequence(
                    superstructure.goToSourceIntakePosition(),
                    superstructure.intakeCoral()
                ), 
                drivetrain.pidToPoseContinuousCommand()
            ),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            superstructure.stowCommand()
        );
    }

    public Command followTrajectoryCommand(String trajectoryName, int index) {
        return Commands.sequence(
            factory.resetOdometry(trajectoryName, index),
            factory.trajectoryCmd(trajectoryName, index)
        );
    }
}