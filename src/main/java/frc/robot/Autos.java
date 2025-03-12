package frc.robot;

import choreo.auto.AutoFactory;
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
            factory.trajectoryCmd("BBML-J"),
            Commands.parallel(
                Commands.sequence(
                    superstructure.goToL4Command(),
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitSeconds(0.2)
                ),
                Commands.sequence(
                    Commands.runOnce(() -> {drivetrain.setTargetPose(new Pose2d(5.03, 5.25, Rotation2d.fromDegrees(60))); drivetrain.enablePositionTargeting();}, drivetrain),
                    drivetrain.setNearestRequestedReefPoseTargetCommand(),
                    Commands.deadline(
                        Commands.waitSeconds(0.5),
                        drivetrain.pidToPoseContinuousCommand()
                    )
                )
            ),
            superstructure.outtakeCoral(),
            superstructure.stowCommand(),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.waitUntil(() -> superstructure.atTargets())
        );
    }

    public Command middleOneL4Auto() {
        return Commands.sequence(
            factory.trajectoryCmd("CL-H"),
            Commands.parallel(
                Commands.sequence(
                    superstructure.goToL4Command(),
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitSeconds(0.2)
                ),
                Commands.sequence(
                    Commands.runOnce(() -> {drivetrain.setTargetPose(new Pose2d(5.755, 4.18, Rotation2d.fromDegrees(0))); drivetrain.enablePositionTargeting();}, drivetrain),
                    drivetrain.setNearestRequestedReefPoseTargetCommand(),
                    Commands.deadline(
                        Commands.waitSeconds(0.5), 
                        drivetrain.pidToPoseContinuousCommand()
                    )
                )
            ),
            superstructure.outtakeCoral(),
            superstructure.stowCommand(),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.waitUntil(() -> superstructure.atTargets())
        );
    }
    
    public Command rightOneL4Auto() {
        return Commands.sequence(
            factory.trajectoryCmd("RBML-F"),
            Commands.parallel(
                Commands.sequence(
                    superstructure.goToL4Command(),
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitSeconds(0.2)
                ),
                Commands.sequence(
                    Commands.runOnce(() -> {drivetrain.setTargetPose(new Pose2d(5.27, 2.97, Rotation2d.fromDegrees(-60))); drivetrain.enablePositionTargeting();}, drivetrain),
                    drivetrain.setNearestRequestedReefPoseTargetCommand(),
                    Commands.deadline(
                        Commands.waitSeconds(0.5), 
                        drivetrain.pidToPoseContinuousCommand()
                    )
                )
            ),
            superstructure.outtakeCoral(),
            superstructure.stowCommand(),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.waitUntil(() -> superstructure.atTargets())
        );
    }
}