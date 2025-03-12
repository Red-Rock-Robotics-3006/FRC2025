package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Superstructure.Position;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import redrocklib.logging.SmartDashboardNumber;

public class Autos {
    private final AutoFactory factory;

    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    // private final Elevator elevator = Elevator.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();

    private SmartDashboardNumber waitTimePID = new SmartDashboardNumber("PID wait time", 1);

    public Autos(AutoFactory f) {
        factory = f;
    }

    private void drivetrainSetTargetPoseConstruct() {
        drivetrain.setTargetPose(drivetrain.constructTestTargetPose());
    }

    public Command testpath2Auto() {
        return Commands.sequence(
            Commands.print("MMMMMMMMMMMMMMMM"),
            factory.trajectoryCmd("testpath2"),
            Commands.print("MMMMMMMMMMMMMMMM")
        );
    }

    public Command leftOneL4Auto() {
        return Commands.sequence(
            factory.trajectoryCmd("BBML-J"),
            Commands.parallel(
                Commands.sequence(
                    superstructure.goToL4Command(),
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitSeconds(0.2),
                    Commands.print("hi")
                ),
                Commands.sequence(
                    Commands.runOnce(() -> {drivetrain.setTargetPose(new Pose2d(5.03, 5.25, Rotation2d.fromDegrees(60))); drivetrain.enablePositionTargeting();}, drivetrain),
                    drivetrain.setNearestRequestedReefPoseTargetCommand(),
                    Commands.deadline(
                        Commands.waitSeconds(0.5), 
                        drivetrain.pidToPoseContinuousCommand()
                    ),
                    Commands.print("by")
                )
            ),
            superstructure.outtakeCoral(),
            superstructure.stowCommand(),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.waitUntil(() -> superstructure.atTargets())
            // factory.trajectoryCmd("testlong2")
        );
    }

    public Command middleOneL4Auto() {
        return Commands.sequence(
            factory.trajectoryCmd("CL-H"),
            Commands.parallel(
                Commands.sequence(
                    superstructure.goToL4Command(),
                    Commands.waitUntil(() -> superstructure.atTargets()),
                    Commands.waitSeconds(0.2),
                    Commands.print("hi")
                ),
                Commands.sequence(
                    Commands.runOnce(() -> {drivetrain.setTargetPose(new Pose2d(5.755, 4.18, Rotation2d.fromDegrees(0))); drivetrain.enablePositionTargeting();}, drivetrain),
                    drivetrain.setNearestRequestedReefPoseTargetCommand(),
                    Commands.deadline(
                        Commands.waitSeconds(0.5), 
                        drivetrain.pidToPoseContinuousCommand()
                    ),
                    Commands.print("by")
                )
            ),
            superstructure.outtakeCoral(),
            superstructure.stowCommand(),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.waitUntil(() -> superstructure.atTargets())
            // factory.trajectoryCmd("testlong2")
        );
    }

    public Command rightOneL4Auto() {
        return Commands.sequence(
            factory.trajectoryCmd("RBML-F"),
            Commands.runOnce(() -> {drivetrain.setTargetPose(new Pose2d(5.27, 2.97, Rotation2d.fromDegrees(-60))); drivetrain.enablePositionTargeting();}, drivetrain),
            drivetrain.setNearestRequestedReefPoseTargetCommand(),
            superstructure.goToL4Command(),
            Commands.deadline(
                Commands.waitSeconds(2), 
                Commands.waitUntil(() -> superstructure.atTargets())
            ),
            superstructure.outtakeCoral(),
            superstructure.stowCommand(),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.waitUntil(() -> superstructure.atTargets())
            // factory.trajectoryCmd("testlong2")
        );
    }

    public Command testlong1AutoCMD() {
        return Commands.sequence(
            factory.trajectoryCmd("testlong1"),
            Commands.runOnce(() -> {drivetrain.setTargetPose(new Pose2d(5.785, 3.83, Rotation2d.kZero)); drivetrain.enablePositionTargeting();}, drivetrain),
            drivetrain.setNearestRequestedReefPoseTargetCommand(),
            superstructure.goToL4Command(),
            Commands.deadline(
                Commands.waitSeconds(2), 
                Commands.waitUntil(() -> superstructure.atTargets())
            ),
            superstructure.outtakeCoral(),
            superstructure.stowCommand(),
            Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain),
            Commands.waitUntil(() -> superstructure.atTargets()),
            factory.trajectoryCmd("testlong2")
        );
    }

    public Command testlongPathsCMD() {
        return Commands.sequence(
            factory.trajectoryCmd("testlong1"),
            factory.trajectoryCmd("testlong2")
        );
    }

    public Command testlongPathsPIDCMD() {
        return Commands.sequence(
            factory.trajectoryCmd("testlong1"),
            Commands.runOnce(() -> drivetrain.setTargetPose(new Pose2d(5.8, 3.83, new Rotation2d(0)))),
            drivetrain.pidToPoseUntilCommand(() -> drivetrain.atTargetPose() && drivetrain.atTargetVelocity()),
            factory.trajectoryCmd("testlong2")
        );
    }

    public AutoRoutine testcuts1Auto() {
        final AutoRoutine routine = factory.newRoutine("Test Cuts 1 Auto Full");
        final AutoTrajectory path1 = routine.trajectory("testcuts1");
        final AutoTrajectory path2 = routine.trajectory("testleavefromscore");

        routine.active().onTrue(
            Commands.sequence(
                factory.resetOdometry("testcuts1"),
                path1.cmd(),
                drivetrain.goToPoseCommand(),
                superstructure.goToPosition(Position.L3),// elevator.goToPosition(Position.L3),
                new WaitUntilCommand(() -> superstructure.atTargets()),//elevator.withinTargetRotation(Position.L3)),
                // doohickey.startOuttakeCommand(),
                new WaitCommand(1),
                // doohickey.stopCommand(),
                new InstantCommand(() -> drivetrain.disablePositionTargeting()),
                superstructure.goToPosition(Position.SOURCE),// elevator.goToPosition(Position.SOURCE),
                path2.cmd()
            )
        );
        return routine;
    }

    public AutoRoutine testcuts1Paths() {
        final AutoRoutine routine = factory.newRoutine("Test Cuts 1 Auto Paths");
        final AutoTrajectory path1 = routine.trajectory("testcuts1");
        final AutoTrajectory path2 = routine.trajectory("testleavefromscore");

        routine.active().onTrue(
            Commands.sequence(
                factory.resetOdometry("testcuts1"),
                path1.cmd(),
                path2.cmd()
            )
        );
        return routine;
    }

    public Command testcutsPathsCMD() {
        return Commands.sequence(
            factory.trajectoryCmd("testcuts1"),
            factory.trajectoryCmd("testcuts2")
        );
    }

    public AutoRoutine testcuts1PathsPID() {
        final AutoRoutine routine = factory.newRoutine("Test Cuts 1 Auto Paths + PID");
        final AutoTrajectory path1 = routine.trajectory("testcuts1");
        final AutoTrajectory path2 = routine.trajectory("testleavefromscore");

        routine.active().onTrue(
            Commands.sequence(
                factory.resetOdometry("testcuts1"),
                path1.cmd(),
                drivetrain.goToPoseCommand(),
                new InstantCommand(() -> drivetrain.disablePositionTargeting()),
                path2.cmd()
            )
        );
        return routine;
    }

    public Command testcutsPathsPIDCMD() {
        return Commands.sequence(
            factory.trajectoryCmd("testcuts1"),
            Commands.runOnce(() -> drivetrain.setTargetPose(new Pose2d(5.8, 3.83, new Rotation2d(0)))),
            drivetrain.pidToPoseUntilCommand(() -> drivetrain.atTargetPose() && drivetrain.atTargetVelocity()),
            factory.trajectoryCmd("testcuts2")
        );
    }
}