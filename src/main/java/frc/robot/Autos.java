package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Superstructure.Position;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Autos {
    private final AutoFactory factory;

    private static final double kBargeAtTargetSettleWaitTime = 0.15;

    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Intake intake = Intake.getInstance();
    private final EndEffector endeffector = EndEffector.getInstance();
    private final Elevator elevator = Elevator.getInstance();

    public Autos(AutoFactory f) {
        factory = f;
    }

    public Command TESTPATH1() {
        return 
        Commands.sequence(
            followTrajectoryWithVisionCommand("TESTPATH1", 0),
            followTrajectoryWithVisionCommand("TESTPATH1", 1)
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
        return followTrajectoryWithVisionCommand("TESTPATH2", 0);
    }

    public Command left444GroundLollipop() {
        return Commands.sequence(
            initializeAutoCommand(),
            intake.deployIntakeCommand(),
            
            Commands.parallel(
                superstructure.goToReefPositionAuto(() -> Position.L4),
                followFirstTrajectoryWithoutVisionCommand("L3GL", 0)
            ),
            slowScoreAutoCommand(1),

            Commands.deadline(
                Commands.sequence(
                    // Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitUntil(() -> elevator.belowAutoStowGroundThreshold()),
                    followFirstTrajectoryWithoutVisionCommand("L3GL", 1)
                ),
                groundIntakeCommand()
            ),  
            Commands.either(
                scoreAutoCommand(1),
                // Commands.print("SECOND CORAL MISSED"), 
                updatePoseVisionCommand(),
                () -> endeffector.coralDetected()
            ),

            Commands.parallel(
                Commands.sequence(
                    // Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitUntil(() -> elevator.belowAutoStowGroundThreshold()),
                    followTrajectoryWithoutVisionCommand("L3GL", 2)
                ),
                groundIntakeCommand()
            ),
            scoreAutoCommand(0),
            superstructure.stowCommand(),

            endAutoCommand()
        );
    }

    public Command right444GroundLollipop() {
        return Commands.sequence(
            initializeAutoCommand(),
            intake.deployIntakeCommand(),
            
            Commands.parallel(
                superstructure.goToReefPositionAuto(() -> Position.L4),
                followFirstTrajectoryWithoutVisionCommand("R3GL", 0)
            ),
            slowScoreAutoCommand(0),

            Commands.deadline(
                Commands.sequence(
                    // Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitUntil(() -> elevator.belowAutoStowGroundThreshold()),
                    followFirstTrajectoryWithoutVisionCommand("R3GL", 1)
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
                    followTrajectoryWithoutVisionCommand("R3GL", 2)
                ),
                groundIntakeCommand()
            ),
            scoreAutoCommand(1),
            superstructure.stowCommand(),

            endAutoCommand()
        );
    }

    public Command left442SixOClock() {
        return Commands.sequence(
            initializeAutoCommand(),
            // intake.deployIntakeCommand(),
            
            Commands.parallel(
                // superstructure.goToReefPositionAuto(() -> Position.L4),
                followFirstTrajectoryWithVisionCommand("LL3GL", 0)
            ),
            superstructure.goToReefPositionAuto(() -> Position.L4),
            scoreAutoCommand(1),

            Commands.deadline(
                Commands.sequence(
                    // Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitUntil(() -> elevator.belowAutoStowGroundThreshold()),
                    followFirstTrajectoryWithVisionCommand("LL3GL", 1)
                ),
                groundIntakeCommand()
            ),  
            Commands.either(
                scoreAutoCommand(0),
                // Commands.print("SECOND CORAL MISSED"), 
                updatePoseVisionCommand(),
                () -> endeffector.coralDetected()
            ),

            superstructure.setRequestedScoringPositionCommand(Position.L2),
            Commands.parallel(
                Commands.sequence(
                    // Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitUntil(() -> elevator.belowAutoStowGroundThreshold()),
                    followTrajectoryWithVisionCommand("LL3GL", 2)
                ),
                groundIntakeCommand()
            ),
            scoreAutoL2Command(1),
            superstructure.stowCommand(),

            endAutoCommand()
        );
    }

    public Command right442SixOClock() {
        return Commands.sequence(
            initializeAutoCommand(),
            // intake.deployIntakeCommand(),
            
            Commands.parallel(
                // superstructure.goToReefPositionAuto(() -> Position.L4),
                followFirstTrajectoryWithVisionCommand("RR3GL", 0)
            ),
            // superstructure.stowReefAutoCommand(),
            slowScoreAutoCommand(0),

            Commands.deadline(
                Commands.sequence(
                    // Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitUntil(() -> elevator.belowAutoStowGroundThreshold()),
                    followFirstTrajectoryWithVisionCommand("RR3GL", 1)
                ),
                groundIntakeCommand()
            ),  
            Commands.either(
                scoreAutoCommand(1),
                // Commands.print("SECOND CORAL MISSED"), 
                updatePoseVisionCommand(),
                () -> endeffector.coralDetected()
            ),

            superstructure.setRequestedScoringPositionCommand(Position.L2),
            Commands.parallel(
                Commands.sequence(
                    // Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitUntil(() -> elevator.belowAutoStowGroundThreshold()),
                    followTrajectoryWithVisionCommand("RR3GL", 2)
                ),
                groundIntakeCommandL2()
            ),
            scoreAutoL2Command(0),
            superstructure.stowCommand(),

            endAutoCommand()
        );
    }

    public Command left444Source() {
        return Commands.sequence(
            initializeAutoCommand(),

            Commands.deadline(
                followFirstTrajectoryWithVisionCommand("L3S", 0),
                superstructure.stowReefAutoCommand()
            ),
            scoreAutoCommand(1),

            Commands.parallel(
                superstructure.goToSourceIntakePositionAuto(),
                followTrajectoryWithVisionCommand("L3S", 1)
            ),
            sourceIntakeCommand(),
            Commands.deadline(
                followTrajectoryWithVisionCommand("L3S", 2),
                superstructure.stowReefAutoCommand()
            ),
            scoreAutoCommand(0),

            Commands.parallel(
                superstructure.goToSourceIntakePosition(),
                followTrajectoryWithVisionCommand("L3S", 3)
            ),
            sourceIntakeCommand(),
            Commands.deadline(
                followTrajectoryWithVisionCommand("L3S", 4),
                superstructure.stowReefAutoCommand()
            ),
            scoreAutoCommand(1)
        );
    }

    public Command right444Source() {
        return Commands.sequence(
            initializeAutoCommand(),

            Commands.deadline(
                followFirstTrajectoryWithVisionCommand("R3S", 0),
                superstructure.stowReefAutoCommand()
            ),
            scoreAutoCommand(0),

            Commands.parallel(
                superstructure.goToSourceIntakePositionAuto(),
                followTrajectoryWithVisionCommand("R3S", 1)
            ),
            sourceIntakeCommand(),
            Commands.deadline(
                followTrajectoryWithVisionCommand("R3S", 2),
                superstructure.stowReefAutoCommand()
            ),
            scoreAutoCommand(1),

            Commands.parallel(
                superstructure.goToSourceIntakePosition(),
                followTrajectoryWithVisionCommand("R3S", 3)
            ),
            sourceIntakeCommand(),
            Commands.deadline(
                followTrajectoryWithVisionCommand("R3S", 4),
                superstructure.stowReefAutoCommand()
            ),
            scoreAutoCommand(0)
        );
    }

    public Command middle4BB() {
        return Commands.sequence(
            initializeAutoCommand(),

            Commands.deadline(
                followFirstTrajectoryWithVisionCommand("M3B", 0),
                superstructure.goToReefPositionAuto(() -> Position.L4)
            ),
            scoreAutoCommand(1),

            Commands.runOnce(() -> {drivetrain.setNearestPreAlgaeTargetPose(); drivetrain.enablePositionTargeting(); drivetrain.enableVision();}, drivetrain),
            Commands.deadline(
                Commands.waitUntil(() -> drivetrain.atTargetPose()), 
                drivetrain.pidToPoseContinuousCommand()),
            superstructure.goToL2RemoveCommand(),
            Commands.deadline(
                Commands.waitUntil(() -> superstructure.hasAlgae()),
                Commands.sequence(
                    Commands.runOnce(() -> {drivetrain.setNearestAlgaeRemovalTargetPose();}, drivetrain),
                    drivetrain.pidToPoseContinuousCommand()
                )
            ),

            followTrajectoryWithVisionCommand("M3B", 1),
            Commands.runOnce(() -> {drivetrain.setNearestBargePoseXTarget(); drivetrain.enablePositionTargeting(); drivetrain.enableVision();}, drivetrain),
            Commands.parallel(
                Commands.deadline(
                    Commands.waitUntil(() -> drivetrain.atTargetPose()), 
                    drivetrain.pidToPoseContinuousCommand()),
                Commands.sequence(
                    superstructure.goToBargePosition(),
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitSeconds(kBargeAtTargetSettleWaitTime)
                )
            ),
            superstructure.outtakeAlgae(),
            superstructure.stowCommand(),

            followFirstTrajectoryWithVisionCommand("M3B", 2),
            scoreAutoCommand(1),
            Commands.runOnce(() -> {drivetrain.setNearestPreAlgaeTargetPose(); drivetrain.enablePositionTargeting(); drivetrain.enableVision();}, drivetrain),
            Commands.deadline(
                Commands.waitUntil(() -> drivetrain.atTargetPose()), 
                drivetrain.pidToPoseContinuousCommand()),
            superstructure.goToL2RemoveCommand(),
            Commands.deadline(
                Commands.waitUntil(() -> superstructure.hasAlgae()),
                Commands.sequence(
                    Commands.runOnce(() -> {drivetrain.setNearestAlgaeRemovalTargetPose();}, drivetrain),
                    drivetrain.pidToPoseContinuousCommand()
                )
            ),

            followTrajectoryWithVisionCommand("M3B", 3),
            Commands.runOnce(() -> {drivetrain.setNearestBargePoseXTarget(); drivetrain.enablePositionTargeting(); drivetrain.enableVision();}, drivetrain),
            Commands.parallel(
                Commands.deadline(
                    Commands.waitUntil(() -> drivetrain.atTargetPose()), 
                    drivetrain.pidToPoseContinuousCommand()),
                Commands.sequence(
                    superstructure.goToBargePosition(),
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitSeconds(kBargeAtTargetSettleWaitTime)
                )
            ),
            superstructure.outtakeAlgae(),
            superstructure.stowCommand()
        );
    }

    public Command scoreAutoCommand(int reefSide) {
        return Commands.sequence(
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            Commands.parallel(
                Commands.sequence(
                    superstructure.goToReefPositionAuto(() -> Position.L4),
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitSeconds(0.35)
                ),
                Commands.deadline(
                    Commands.waitUntil(() -> drivetrain.atAutoTargetPose()),
                    Commands.sequence(
                        Commands.runOnce(() -> {drivetrain.setReefSide(reefSide); drivetrain.setNearestRequestedReefPoseTarget(); drivetrain.enablePositionTargeting();}, drivetrain),
                        drivetrain.pidToPoseContinuousCommand()
                    )
                )
            ),
            drivetrain.disablePositionTargetingCommand(),
            superstructure.outtakeCoral(),
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain)
        );
    }

    public Command scoreAutoL2Command(int reefSide) {
        return Commands.sequence(
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            Commands.parallel(
                Commands.sequence(
                    superstructure.goToReefPositionAuto(() -> Position.L2),
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitSeconds(0.35)
                ),
                Commands.deadline(
                    Commands.waitUntil(() -> drivetrain.atAutoTargetPose()),
                    Commands.sequence(
                        Commands.runOnce(() -> {drivetrain.setReefSide(reefSide); drivetrain.setNearestRequestedReefPoseTarget(); drivetrain.enablePositionTargeting();}, drivetrain),
                        drivetrain.pidToPoseContinuousCommand()
                    )
                )
            ),
            drivetrain.disablePositionTargetingCommand(),
            superstructure.outtakeCoral(),
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain)
        );
    }

    public Command slowScoreAutoCommand(int reefSide) {
        return Commands.sequence(
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            Commands.parallel(
                Commands.sequence(
                    superstructure.goToReefPositionAuto(() -> Position.L4),
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitSeconds(0.8)
                ),
                Commands.deadline(
                    Commands.waitUntil(() -> drivetrain.atAutoTargetPose()),
                    Commands.sequence(
                        Commands.runOnce(() -> {drivetrain.setReefSide(reefSide); drivetrain.setNearestRequestedReefPoseTarget(); drivetrain.enablePositionTargeting();}, drivetrain),
                        drivetrain.pidToPoseContinuousCommand()
                    )
                )
            ),
            drivetrain.disablePositionTargetingCommand(),
            superstructure.outtakeCoral(),
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain)
        );
    }

    public Command groundIntakeCommand() {
        return Commands.sequence(
            superstructure.goToGroundIntakePositionAuto(),
            intake.startIntakeCommand(),
            superstructure.intakeCoral(),
            superstructure.goToReefPositionAuto(() -> Position.L4)
        );
    }

    public Command groundIntakeCommandL2() {
        return Commands.sequence(
            superstructure.goToGroundIntakePositionAuto(),
            intake.startIntakeCommand(),
            superstructure.intakeCoral(),
            superstructure.goToReefPositionAuto(() -> Position.L2)
        );
    }

    public Command sourceIntakeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {drivetrain.enableVision(); drivetrain.setNearestSourcePose(); drivetrain.enablePositionTargeting();}, drivetrain),
            Commands.deadline(
                Commands.sequence(
                    superstructure.goToSourceIntakePosition(),
                    superstructure.intakeCoral()
                ), 
                drivetrain.pidToPoseContinuousCommand()
            ),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain)
            // Commands.waitSeconds()
            // superstructure.stowReefCommand()
        );
    }

    public Command followTrajectoryWithoutVisionCommand(String trajectoryName, int index) {
        return Commands.sequence(
            Commands.runOnce(() -> drivetrain.disableVision(), drivetrain),
            factory.trajectoryCmd(trajectoryName, index),
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain)
        );
    }

    public Command followTrajectoryWithVisionCommand(String trajectoryName, int index) {
        return Commands.sequence(
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            factory.trajectoryCmd(trajectoryName, index)
        );
    }

    public Command followFirstTrajectoryWithoutVisionCommand(String trajectoryName, int index) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                drivetrain.disableVision();
                factory.resetOdometry(trajectoryName, index);
                // drivetrain.resetKalaman();
                drivetrain.resetPose(
                    factory.cache().loadTrajectory(trajectoryName, index).get().getInitialPose(DriverStation.getAlliance().get().equals(Alliance.Red)).get()
                    ); 
            },
            drivetrain),
            // Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            // factory.resetOdometry(trajectoryName, index),
            factory.trajectoryCmd(trajectoryName, index)
            // Commands.runOnce(() -> drivetrain.enableVision(), drivetrain)
        );
    }

    public Command followFirstTrajectoryWithVisionCommand(String trajectoryName, int index) {
        return Commands.sequence(
            Commands.runOnce(() -> {
                drivetrain.disableVision();
                factory.resetOdometry(trajectoryName, index);
                // drivetrain.resetKalaman();
                drivetrain.resetPose(
                    factory.cache().loadTrajectory(trajectoryName, index).get().getInitialPose(DriverStation.getAlliance().get().equals(Alliance.Red)).get()
                    ); 
            },
            drivetrain),
            Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
            // factory.resetOdometry(trajectoryName, index),
            factory.trajectoryCmd(trajectoryName, index)
            // Commands.runOnce(() -> drivetrain.enableVision(), drivetrain)
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
}