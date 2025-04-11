package frc.robot;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Superstructure.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Autos {
    private final AutoFactory factory;

    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Arm arm = Arm.getInstance();
    private final EndEffector endeffector = EndEffector.getInstance();
    private final Elevator elevator = Elevator.getInstance();

    public Autos(AutoFactory f) {
        factory = f;
    }

    public Command TESTPATH1() {
        return 
        Commands.sequence(
            followTrajectoryWithPIDEndingCommand("TESTPATH1", 0),
            followTrajectoryWithPIDEndingCommand("TESTPATH1", 1)
            // Commands.deadline(
            //     Commands.waitSeconds(6),
            //     followTrajectoryWithPIDEndingCommand("TESTPATH1", 0)
            // ),
            
            // Commands.deadline(
            //     Commands.waitSeconds(6),
            //     followTrajectoryWithPIDEndingCommand("TESTPATH1", 1)
            // )
        );
    }

    public Command TESTPATH2() {
        return followTrajectoryWithPIDEndingCommand("TESTPATH2", 0);
    }

    public Command left3L4Paths() {
        return Commands.sequence(
            // followTrajectoryWithPIDEndingCommand("L3GS", 0),
            // Commands.waitSeconds(1.5),
            followTrajectoryWithPIDEndingCommand("L3GS", 1),
            // Commands.waitSeconds(1.5),
            followTrajectoryWithPIDEndingCommand("L3GS", 2),
            // Commands.waitSeconds(1.5),
            followTrajectoryWithPIDEndingCommand("L3GS", 3),
            // Commands.waitSeconds(1.5),
            followTrajectoryWithPIDEndingCommand("L3GS", 4)
        );
    }

    public Command right3L4Paths() {
        return Commands.sequence(
            // followTrajectoryCommand("R3GS", 0),
            // Commands.waitSeconds(1.5),
            followTrajectoryWithPIDEndingCommand("R3GS", 1),
            // Commands.waitSeconds(1.5),
            followTrajectoryWithPIDEndingCommand("R3GS", 2),
            // Commands.waitSeconds(1.5),
            followTrajectoryWithPIDEndingCommand("R3GS", 3),
            // Commands.waitSeconds(1.5),
            followTrajectoryWithPIDEndingCommand("R3GS", 4)
        );
    }
    
    public Command left3L4GroundSource() {
        return Commands.sequence(
            endeffector.stopCommand(),
            superstructure.setRequestedScoringPositionCommand(Position.L4),

            followTrajectoryCommand("L3GS", 0),
            scoreAutoCommand(1),
            superstructure.outtakeCoral(),

            Commands.deadline(
                Commands.sequence(
                    followTrajectoryCommand("L3GS", 1),
                    Commands.waitSeconds(0.2),
                    followTrajectoryCommand("L3GS", 2)
                ),
                groundIntakeCommand()
            ),  
            Commands.either(
                Commands.sequence(
                    scoreAutoCommand(0), 
                    superstructure.outtakeCoral()
                ),
                updatePoseVisionCommand(), 
                () -> endeffector.coralDetected()
            ),

            Commands.parallel(
                Commands.sequence(
                    followTrajectoryCommand("L3GS", 3),
                    Commands.waitSeconds(0.2),
                    followTrajectoryCommand("L3GS", 4)
                ),
                groundIntakeCommand()
            ),  
            scoreAutoCommand(1),
            superstructure.outtakeCoral()
        );
    }

    public Command right3L4GroundSource() {
        return Commands.sequence(
            endeffector.stopCommand(),
            superstructure.setRequestedScoringPositionCommand(Position.L4),
            
            followTrajectoryCommand("R3GS", 0),
            scoreAutoCommand(0),
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
                    scoreAutoCommand(1), 
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
            scoreAutoCommand(0),
            superstructure.outtakeCoral()
        );
    }

    public Command left3L4GroundLollipop() {
        return Commands.sequence(
            initializeAutoCommand(),
            intake.deployIntakeCommand(),
            
            followFirstTrajectoryCommand("L3GL", 0),
            scoreAutoCommand(1),

            Commands.deadline(
                Commands.sequence(
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    followTrajectoryCommand("L3GL", 1)
                ),
                groundIntakeCommand()
            ),  
            Commands.either(
                scoreAutoCommand(1),
                Commands.print("SECOND CORAL MISSED"), 
                () -> endeffector.coralDetected()
            ),

            Commands.parallel(
                Commands.sequence(
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    followFirstTrajectoryCommand("L3GL", 2)
                ),
                groundIntakeCommand()
            ),
            scoreAutoCommand(0),
            superstructure.stowCommand(),

            endAutoCommand()
        );
    }

    public Command left3L4GroundLollipopPaths() {
        return Commands.sequence(
            followFirstTrajectoryCommand("L3GL", 0),
            followTrajectoryCommand("L3GL", 1),
            followTrajectoryCommand("L3GL", 2)
        );
    }

    public Command right3L4GroundLollipop() {
        return Commands.sequence(
            initializeAutoCommand(),
            intake.deployIntakeCommand(),
            
            // followFirstTrajectoryCommand("R3GL", 0),
            scoreAutoCommand(0),

            Commands.deadline(
                Commands.sequence(
                    // Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitUntil(() -> elevator.belowAutoStowGroundThreshold()),
                    followFirstTrajectoryCommand("R3GL", 1)
                ),
                groundIntakeCommand()
            ),  
            Commands.either(
                scoreAutoCommand(0),
                // Commands.print("SECOND CORAL MISSED"), 
                updatePoseVisionCommand(),
                () -> endeffector.coralDetected()
            ),

            Commands.parallel(
                Commands.sequence(
                    // Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitUntil(() -> elevator.belowAutoStowGroundThreshold()),
                    followFirstTrajectoryCommand("R3GL", 2)
                ),
                groundIntakeCommand()
            ),
            scoreAutoCommand(1),
            superstructure.stowCommand(),

            endAutoCommand()
        );
    }

    public Command right3L4GroundLollipopPaths() {
        return Commands.sequence(
            followFirstTrajectoryCommand("R3GL", 1),
            followTrajectoryCommand("R3GL", 2),
            endAutoCommand()
        );
    }

    public Command left3L4Source() {
        return Commands.sequence(
            endeffector.stopCommand(),

            superstructure.setRequestedScoringPositionCommand(Position.L4),
            Commands.parallel(
                followFirstTrajectoryCommand("L3S", 0),
                superstructure.stowReefCommand()
            ),
            scoreAutoCommand(1),
            superstructure.outtakeCoral(),

            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain),
            Commands.parallel(
                superstructure.goToSourceIntakePosition(),
                followTrajectoryCommand("L3S", 1)
            ),
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            sourceIntakeCommand(),
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain),
            Commands.deadline(
                followTrajectoryCommand("L3S", 2),
                Commands.sequence(
                    Commands.waitSeconds(0.8),
                    superstructure.goToL4Command()
                )
            ),
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            scoreAutoCommand(0), 
            superstructure.outtakeCoral(),
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain),

            Commands.parallel(
                superstructure.goToSourceIntakePosition(),
                followTrajectoryCommand("L3S", 3)
            ),
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            sourceIntakeCommand(),
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain),
            Commands.deadline(
                followTrajectoryCommand("L3S", 4),
                Commands.sequence(
                    Commands.waitSeconds(0.8),
                    superstructure.goToL4Command()
                )
            ),
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            scoreAutoCommand(1), 
            superstructure.outtakeCoral(),
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain)
        );
    }

    public Command right3L4Source() {
        return Commands.sequence(
            endeffector.stopCommand(),

            superstructure.setRequestedScoringPositionCommand(Position.L4),
            
            Commands.parallel(
                followFirstTrajectoryCommand("R3S", 0),
                superstructure.stowReefCommand()
            ),
            scoreAutoCommand(0),
            superstructure.outtakeCoral(),

            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain),
            Commands.parallel(
                superstructure.goToSourceIntakePosition(),
                followTrajectoryCommand("R3S", 1)
            ),
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            sourceIntakeCommand(),
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain),
            Commands.deadline(
                followTrajectoryCommand("R3S", 2),
                Commands.sequence(
                    Commands.waitSeconds(0.8),
                    superstructure.goToL4Command()
                )
            ),
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            scoreAutoCommand(1), 
            superstructure.outtakeCoral(),
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain),

            Commands.parallel(
                superstructure.goToSourceIntakePosition(),
                followTrajectoryCommand("R3S",  3)
            ),
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            sourceIntakeCommand(),
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain),
            Commands.deadline(
                followTrajectoryCommand("R3S", 4),
                Commands.sequence(
                    Commands.waitSeconds(0.8),
                    superstructure.goToL4Command()
                )
            ),
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            scoreAutoCommand(0), 
            superstructure.outtakeCoral(),
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain)
        );
    }

    public Command slowLeft3L4Source() {
        return Commands.sequence(
            endeffector.stopCommand(),

            superstructure.setRequestedScoringPositionCommand(Position.L4),
            
            followTrajectoryCommand("L3SS", 0),
            slowScoreAutoCommand(1),
            superstructure.outtakeCoral(),

            Commands.parallel(
                superstructure.goToSourceIntakePosition(),
                followTrajectoryCommand("L3SS", 1)
            ),
            sourceIntakeCommand(),
            followTrajectoryCommand("L3SS", 2),
            slowScoreAutoCommand(0), 
            superstructure.outtakeCoral(),

            Commands.parallel(
                superstructure.goToSourceIntakePosition(),
                followTrajectoryCommand("L3SS", 3)
            ),
            sourceIntakeCommand(),
            followTrajectoryCommand("L3SS", 4),
            slowScoreAutoCommand(1), 
            superstructure.outtakeCoral()
        );
    }

    public Command slowRight3L4Source() {
        return Commands.sequence(
            endeffector.stopCommand(),

            superstructure.setRequestedScoringPositionCommand(Position.L4),
            
            followTrajectoryCommand("R3SS", 0),
            slowScoreAutoCommand(0),
            superstructure.outtakeCoral(),

            Commands.parallel(
                superstructure.goToSourceIntakePosition(),
                followTrajectoryCommand("R3SS", 1)
            ),
            sourceIntakeCommand(),
            followTrajectoryCommand("R3SS", 2),
            slowScoreAutoCommand(1), 
            superstructure.outtakeCoral(),

            Commands.parallel(
                superstructure.goToSourceIntakePosition(),
                followTrajectoryCommand("R3SS", 3)
            ),
            sourceIntakeCommand(),
            followTrajectoryCommand("R3SS", 4),
            slowScoreAutoCommand(0), 
            superstructure.outtakeCoral()
        );
    }

    public Command scoreAutoCommand(Pose2d pose) {
        return Commands.sequence(
            Commands.deadline(
                Commands.sequence(
                    superstructure.goToL4Command(),
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitSeconds(0.15)
                ),
                Commands.sequence(
                    Commands.runOnce(() -> {drivetrain.setTargetPose(pose); drivetrain.enablePositionTargeting();}, drivetrain),
                    drivetrain.pidToPoseContinuousCommand()
                )
            ),
            drivetrain.disablePositionTargetingCommand()
        );
    }

    public Command scoreAutoCommand(int reefSide) {
        return Commands.sequence(
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            Commands.deadline(
                Commands.sequence(
                    superstructure.goToReefPositionAuto(() -> Position.L4),
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitSeconds(0.35)
                ),
                Commands.sequence(
                    Commands.runOnce(() -> {drivetrain.setReefSide(reefSide); drivetrain.setNearestRequestedReefPoseTarget(); drivetrain.enablePositionTargeting();}, drivetrain),
                    drivetrain.pidToPoseContinuousCommand()
                )
            ),
            drivetrain.disablePositionTargetingCommand(),
            superstructure.outtakeCoral(),
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain)
        );
    }

    public Command slowScoreAutoCommand(int reefSide) {
        return Commands.sequence(
            Commands.deadline(
                Commands.sequence(
                    superstructure.goToL4Command(),
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitSeconds(0.8)
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
            superstructure.goToIntakePositionAuto(),
            intake.startIntakeCommand(),
            superstructure.intakeCoral(),
            superstructure.goToReefPositionAuto(() -> Position.L4)
        );
    }

    public Command sourceIntakeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {drivetrain.setNearestSourcePose(); drivetrain.enablePositionTargeting();}, drivetrain),
            Commands.deadline(
                // Commands.sequence(
                    // superstructure.goToSourceIntakePosition(),
                superstructure.intakeCoral(),
                // ), 
                drivetrain.pidToPoseContinuousCommand()
            ),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain)
            // Commands.waitSeconds()
            // superstructure.stowReefCommand()
        );
    }

    public Command followTrajectoryCommand(String trajectoryName, int index) {
        return Commands.sequence(
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain),
            // factory.resetOdometry(trajectoryName, index),
            factory.trajectoryCmd(trajectoryName, index),
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain)
        );
    }

    public Command initializeAutoCommand() {
        return Commands.sequence(
            endeffector.stopCommand(),
            superstructure.setRequestedScoringPositionCommand(Position.L4)
        );
    }

    public Command endAutoCommand() {
        return Commands.runOnce(() -> drivetrain.setTargetHeadingDegrees(drivetrain.getHeadingDegrees()), drivetrain);
    }

    public Command updatePoseVisionCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            Commands.waitSeconds(0.5),
            // drivetrain.pidToPoseContinuousCommand(),
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain)
        );
    }

    /**
     * PID to poses to the final pose position in the given trajectory
     * 
     * @return Command to PID to pose until at target pose
     */
    public Command pidToFinalPathPoseCommand(String trajectoryName, int index) {
        return Commands.sequence(
            
            Commands.runOnce(() -> {

                drivetrain.setTargetPose(factory.cache().loadTrajectory(trajectoryName, index).get().getFinalPose(DriverStation.getAlliance().get().equals(Alliance.Red)).get());
                drivetrain.enablePositionTargeting();
                }, drivetrain),
    
            Commands.deadline(
                Commands.race(
                    Commands.waitUntil(() -> drivetrain.atTargetPose()),
                    Commands.waitSeconds(0.4)
                ),
                drivetrain.pidToPoseContinuousCommand()
            ),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.deadline(
                Commands.waitSeconds(0.02), 
                drivetrain.driveContinuousStillCommand()
            )
        );
    }

    /**
     * Follows trajectory normally, but uses PID to pose to make up for any error at the end of the path
     * 
     * @return Command to follow path and then PID to final path pose
     */
    public Command followTrajectoryWithPIDEndingCommand(String trajectoryName, int index) {
        return Commands.sequence(followTrajectoryCommand(trajectoryName, index),
                pidToFinalPathPoseCommand(trajectoryName, index));
    }

    public Command followFirstTrajectoryCommand(String trajectoryName, int index) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                drivetrain.disableVision();
                factory.resetOdometry(trajectoryName, index);
                drivetrain.resetPose(
                    factory.cache().loadTrajectory(trajectoryName, index).get().getInitialPose(DriverStation.getAlliance().get().equals(Alliance.Red)).get()
                    ); 
            },
            drivetrain),
            // Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            // factory.resetOdometry(trajectoryName, index),
            factory.trajectoryCmd(trajectoryName, index),
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain)
        );
    }
    
}