package frc.robot;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Autos {
    private final AutoFactory factory;

    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();

    public Autos(AutoFactory f) {
        factory = f;
    }

    public Command leftOneL4Auto() {
        return Commands.sequence(
            factory.resetOdometry("BBML-J"),
            factory.trajectoryCmd("BBML-J"),
            goToScoreAutoCommand(new Pose2d(5.03, 5.25, Rotation2d.fromDegrees(60))),
            superstructure.outtakeCoral(),
            superstructure.stowCommand(),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.waitUntil(() -> superstructure.atTargets())
        );
    }

    public Command middleOneL4Auto() {
        return Commands.sequence(
            factory.resetOdometry("CL-H"),
            factory.trajectoryCmd("CL-H"),
            goToScoreAutoCommand(new Pose2d(5.755, 4.18, Rotation2d.fromDegrees(0))),
            superstructure.outtakeCoral(),
            superstructure.stowCommand(),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.waitUntil(() -> superstructure.atTargets())
        );
    }
    
    public Command rightOneL4Auto() {
        return Commands.sequence(
            factory.resetOdometry("RBML-F"),
            factory.trajectoryCmd("RBML-F"),
            goToScoreAutoCommand(new Pose2d(5.27, 2.97, Rotation2d.fromDegrees(-60))),
            superstructure.outtakeCoral(),
            superstructure.stowCommand(),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.waitUntil(() -> superstructure.atTargets())
        );
    }

    public Command middleTwoL4Auto() {
        return Commands.sequence(
            factory.resetOdometry("CL-H"),
            factory.trajectoryCmd("CL-H"),
            goToScoreAutoCommand(new Pose2d(5.755, 4.18, Rotation2d.fromDegrees(0))),
            superstructure.outtakeCoral(),
            superstructure.stowCommand(),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.waitUntil(() -> superstructure.atTargets()),

            factory.resetOdometry("HL-C"),
            factory.trajectoryCmd("HL-C"),
            Commands.runOnce(() -> {drivetrain.setTargetPose(new Pose2d(7.17, 4, Rotation2d.fromDegrees(0))); drivetrain.enablePositionTargeting();}, drivetrain),
            Commands.deadline(
                Commands.sequence(
                    superstructure.goToSourceIntakePosition(),
                    superstructure.intakeCoral()
                ), 
                drivetrain.pidToPoseContinuousCommand()
            ),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            superstructure.stowCommand(),

            factory.resetOdometry("CL-G"),
            factory.trajectoryCmd("CL-G"),
            goToScoreAutoCommand(new Pose2d(5.755, 3.84, Rotation2d.fromDegrees(0))),
            superstructure.outtakeCoral(),
            superstructure.stowCommand(),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.waitUntil(() -> superstructure.atTargets())
        );
    }

    public Command middleTwoL4Paths() {
        return Commands.sequence(
            factory.resetOdometry("CL-H"),
            factory.trajectoryCmd("CL-H"),
            factory.resetOdometry("HL-C"),
            factory.trajectoryCmd("HL-C"),
            factory.resetOdometry("CL-G"),
            factory.trajectoryCmd("CL-G")
        );
    }

    public Command justGiveItANameAuto() {
        return Commands.sequence(
            factory.resetOdometry("RandomL-E"),
            factory.trajectoryCmd("RandomL-E"),
            goToScoreAutoCommand(new Pose2d(5, 2.8, Rotation2d.fromDegrees(-60))),
            superstructure.outtakeCoral(),
            superstructure.stowCommand(),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.waitUntil(() -> superstructure.atTargets()),

            factory.resetOdometry("EL-RSM"),
            factory.trajectoryCmd("EL-RSM"),
            goToIntakeAutoCommand(),

            factory.resetOdometry("RSML-D"),
            factory.trajectoryCmd("RSML-D"),
            goToScoreAutoCommand(new Pose2d(3.975, 2.81, Rotation2d.fromDegrees(-120))),
            superstructure.outtakeCoral(),
            superstructure.stowCommand(),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.waitUntil(() -> superstructure.atTargets())
        );
    }

    public Command justGiveItANamePaths() {
        return Commands.sequence(
            factory.resetOdometry("RandomL-E"),
            factory.trajectoryCmd("RandomL-E"),
            factory.resetOdometry("EL-RSM"),
            factory.trajectoryCmd("EL-RSM"),
            factory.resetOdometry("RSML-D"),
            factory.trajectoryCmd("RSML-D")
        );
    }

    public Command goToScoreAutoCommand(Pose2d pose) {
        return Commands.parallel(
            Commands.sequence(
                superstructure.goToL4Command(),
                Commands.waitUntil(() -> superstructure.atTargets()),
                Commands.waitSeconds(0.2)
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
        return Commands.parallel(
            Commands.sequence(
                superstructure.goToL4Command(),
                Commands.waitUntil(() -> superstructure.atTargets()),
                Commands.waitSeconds(0.2)
            ),
            Commands.sequence(
                Commands.runOnce(() -> {drivetrain.setReefSide(reefSide); drivetrain.setNearestRequestedReefPoseTarget(); drivetrain.enablePositionTargeting();}, drivetrain),
                drivetrain.setNearestRequestedReefPoseTargetCommand(),
                Commands.deadline(
                    Commands.waitSeconds(0.5), 
                    drivetrain.pidToPoseContinuousCommand()
                ),
                Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain)
            )
        );
    }

    public Command goToIntakeAutoCommand() {
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
}