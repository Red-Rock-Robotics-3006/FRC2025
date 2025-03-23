// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Superstructure.Position;
import frc.robot.subsystems.*;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import redrocklib.logging.SmartDashboardBoolean;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static double progressiveDriveExponent = 1.4;
    public static double progressiveTurnExponent = 1.7;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * CommandSwerveDrivetrain.getInstance().getDriveDeadBand()).withRotationalDeadband(MaxAngularRate * CommandSwerveDrivetrain.getInstance().getTurnDeadBand())
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController drivestick = new CommandXboxController(0);
    private final CommandXboxController mechstick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Climber climber = Climber.getInstance();
    private final LED leds = LED.getInstance();

    private final AutoFactory autoFactory;
    private final Autos autos;

    private SendableChooser<Command> m_chooser = new SendableChooser<>();

    private SmartDashboardBoolean inPIDTolerance = new SmartDashboardBoolean("dt/dt-in-pid-tolerance", false);


    public RobotContainer() {
        drivetrain.setSwerveRequest(this.driveFacingAngle);
        drivetrain.setDriveController(this.drivestick);
        drivetrain.setDriveRequest(this.drive);

        autoFactory = drivetrain.createAutoFactory();
        autos = new Autos(autoFactory);

        configureBindings();
        configureSelector();
    }

    public void configureSelector(){
        m_chooser.setDefaultOption("no auto", Commands.print("good luck drivers!"));

        m_chooser.addOption("TESTPATH1", autos.TESTPATH1());
        m_chooser.addOption("TESTPATH2", autos.TESTPATH2());

        m_chooser.addOption("Left 3 L4", autos.left3L4());
        m_chooser.addOption("Right 3 L4", autos.right3L4());

        m_chooser.addOption("Left 3 L4 Paths", autos.left3L4Paths());
        m_chooser.addOption("Right 3 L4 Paths", autos.right3L4Paths());
            
        SmartDashboard.putData("AUTO CHOOSER", m_chooser);
    }

    private void configureBindings() {
        configureDriveBindings();
        // configureTestBindings();
        configureCompBindings();
    }
    
    private void configureDriveBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(
              () -> {
                double[] drivestickValues = rotateBy(-drivestick.getLeftY(), -drivestick.getLeftX(), drivetrain.getFieldCentricOffset());
                if (drivetrain.isTargetingPosition()) {
                    inPIDTolerance.putBoolean(false);
                    return driveFacingAngle.withVelocityX(MathUtil.clamp(drivetrain.getPositionPIDValueX() * drivetrain.getPIDScale(), -drivetrain.getMaxPIDVelocity(), drivetrain.getMaxPIDVelocity()))
                                            .withVelocityY(MathUtil.clamp(drivetrain.getPositionPIDValueY() * drivetrain.getPIDScale(), -drivetrain.getMaxPIDVelocity(), drivetrain.getMaxPIDVelocity()))
                                            .withTargetDirection(Rotation2d.fromDegrees(drivetrain.getTargetHeadingDegrees()));
                }
                else if (!drivetrain.getUseHeadingPID() || Math.abs(drivestick.getRightX()) > drivetrain.getTurnDeadBand()) {
                    inPIDTolerance.putBoolean(false);
                  return drive.withVelocityX(progressiveInput(drivestickValues[0],progressiveDriveExponent) * MaxSpeed)
                              .withVelocityY(progressiveInput(drivestickValues[1],progressiveDriveExponent, true) * MaxSpeed)
                              .withRotationalRate(progressiveInput(-drivestick.getRightX(),progressiveTurnExponent) * MaxAngularRate);
                }
                else if (driveFacingAngle.HeadingController.atSetpoint()) {
                    inPIDTolerance.putBoolean(true);
                    return drive.withVelocityX(progressiveInput(drivestickValues[0],progressiveDriveExponent) * MaxSpeed)
                    .withVelocityY(progressiveInput(drivestickValues[1],progressiveDriveExponent, true) * MaxSpeed)
                    .withRotationalRate(0);
                }
                else {
                    inPIDTolerance.putBoolean(false);
                  return driveFacingAngle.withVelocityX(progressiveInput(drivestickValues[0],progressiveDriveExponent) * MaxSpeed)
                                         .withVelocityY(progressiveInput(drivestickValues[1],progressiveDriveExponent, true) * MaxSpeed)
                                         .withTargetDirection(Rotation2d.fromDegrees(drivetrain.getTargetHeadingDegrees()));
                  
                }
              }
            )
          );

        new Trigger(
            () -> Math.abs(drivestick.getRightX()) > drivetrain.getTurnDeadBand()
        ).onTrue(
            new FunctionalCommand(
                () -> {},
                () -> {drivetrain.setTargetHeadingDegrees(drivetrain.getHeadingDegrees());}, 
                (interrupted) -> {drivetrain.setTargetHeadingDegrees(drivetrain.getHeadingDegrees());}, 
                () -> !drivetrain.isRotating() && Math.abs(drivestick.getRightX()) < drivetrain.getTurnDeadBand())
        );

        drivestick.back().and(drivestick.povUp()).onTrue(
            new InstantCommand(drivetrain::toggleHeadingPID, drivetrain)
        );

        // drivestick.rightStick().onTrue(
        //     new InstantCommand(drivetrain::toggleUsingSingleAxis, drivetrain)
        // );

        // drivestick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        drivestick.start().and(drivestick.back()).onTrue(drivetrain.resetHeadingCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureCompBindings() {
        RobotModeTriggers.teleop().onTrue(
            Commands.sequence(
                // superstructure.normalizeCommand()
                superstructure.stopEndEffector(),
                intake.stopIntakeCommand()
            )
        );

        drivestick.povDown().onTrue(
            superstructure.normalizeCommand()
        );

        drivestick.povLeft().onTrue(
            superstructure.normalizeEFCommand()
        );

        drivestick.povRight().onTrue(
            superstructure.normalizeECommand()
        );

        drivestick.leftTrigger(0.25).onTrue(
            Commands.sequence(
                superstructure.goToIntakePosition(),
                Commands.deadline(
                    superstructure.intakeGroundCoral(),
                    // intake.spasmIntakeCommand()
                    intake.startIntakeCommand()
                ),
                this.rumbleControllerCommand(1, 0.6)
            )
        ).onFalse(
            Commands.sequence(
                // superstructure.stowCommand()
                superstructure.stowReefCommand()
            )
        );

        drivestick.rightBumper().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> {drivetrain.setNearestRequestedReefPoseTarget(); drivetrain.enablePositionTargeting();}, drivetrain),
                drivetrain.setNearestRequestedReefPoseTargetCommand(),
                this.rumbleControllerCommand(1, 0.15)
            )
        ).onFalse(
            Commands.sequence(
                Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain)
                // superstructure.stowCommand()
            )
        );

        drivestick.leftBumper().onTrue(
            Commands.parallel(
                // Commands.sequence(
                //     Commands.runOnce(() -> {drivetrain.setNearestSourcePose(); drivetrain.enablePositionTargeting();}, drivetrain),
                //     Commands.either(
                //         drivetrain.pidToPoseContinuousCommand(), 
                //         drivetrain.driveFacingAngleContinuousCommand(), 
                //         () -> drivetrain.getPositionTargeting()
                //     )
                //     // drivetrain.pidToPoseContinuousCommand()
                //     // drivetrain.setNearestRequestedReefPoseTargetCommand(),
                //     // this.rumbleControllerCommand(1, 0.15)
                // ),
                Commands.sequence(
                    Commands.runOnce(() -> drivetrain.setNearestSourcePoseTargetHeading(), drivetrain),
                    drivetrain.driveFacingAngleContinuousCommand()
                ),
                Commands.sequence(
                    superstructure.goToSourceIntakePosition(), //TODO add swerve thing
                    superstructure.intakeCoral(),
                    this.rumbleControllerCommand(1, 0.6)
                )
            )
        ).onFalse(
                // superstructure.stowCommand() //TODO add swerve thing
                superstructure.stowReefCommand()
            
        );

        drivestick.rightTrigger(0.25).onTrue(
            Commands.sequence(
                superstructure.goToIntakeL1Position(),
                intake.intakel1andHoldCommand(),
                superstructure.stowCommand()
            )
        ).onFalse(
            superstructure.stowCommand()
        );

        drivestick.rightTrigger(0.25).and(drivestick.x()).onTrue(
            intake.goL1OuttakeCommand()
        ).onFalse(
            superstructure.stowCommand()
        );

        drivestick.x().and(new Trigger(() -> drivestick.getHID().getRightTriggerAxis() < 0.25)).onTrue(
            superstructure.outtakeCoral()
        ).onFalse(
            superstructure.stowCommand()
        );

        drivestick.y().onTrue(
            Commands.select(
                Map.ofEntries(
                    Map.entry(Position.STOW, superstructure.stowCommand()),
                    Map.entry(Position.L1, superstructure.goToL1Command()),
                    Map.entry(Position.L2, superstructure.goToL2Command()),
                    Map.entry(Position.L3, superstructure.goToL3Command()),
                    Map.entry(Position.L4, superstructure.goToL4Command())
                ),
                () -> superstructure.getRequestedScoringPosition())
            // superstructure.goToRequestedPositionCommand()
        );

        drivestick.b().onTrue(
            superstructure.stowCommand()
        );

        mechstick.x().onTrue(
            superstructure.setRequestedScoringPositionCommand(Position.L2)
        );
        
        mechstick.y().onTrue(
            superstructure.setRequestedScoringPositionCommand(Position.L3)
        );

        mechstick.b().onTrue(
            superstructure.setRequestedScoringPositionCommand(Position.L4)
        );

        mechstick.a().onTrue(
            superstructure.setRequestedScoringPositionCommand(Position.L1)
        );

        mechstick.povLeft().onTrue(
            Commands.runOnce(() -> drivetrain.setReefSide(0), drivetrain)
        );

        mechstick.povRight().onTrue(
            Commands.runOnce(() -> drivetrain.setReefSide(1), drivetrain)
        );

        mechstick.povDown().onTrue(
            Commands.sequence(
                superstructure.setEndEfffectorAlgaeRemovalSpeedCommand(),
                superstructure.goToL2RemoveCommand()
            )
        // ).onFalse(
        //     superstructure.stowCommand()
        );

        mechstick.povUp().onTrue(
            Commands.sequence(
                superstructure.setEndEfffectorAlgaeRemovalSpeedCommand(),
                superstructure.goToL3RemoveCommand()
            )
        // ).onFalse(
        //     superstructure.stowCommand()
        );

        mechstick.leftBumper().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> Arm.getInstance().setClimbPosition(), Arm.getInstance()), //TODO Don't use Arm!
                Commands.runOnce(() -> climber.setStowSpeed(), climber)
            )
        ).onFalse(
            Commands.runOnce(() -> climber.stopClimb(), climber)
        );

        mechstick.rightBumper().onTrue(
            Commands.runOnce(() -> climber.setDeploySpeed(), climber)
        ).onFalse(
            Commands.runOnce(() -> climber.stopClimb(), climber)
        );

        mechstick.leftTrigger(0.25).onTrue(
            // intake.intakeAlgaeAndHoldCommand()
            superstructure.intakeAlgaeGroundCommand()
        ).onFalse(
            superstructure.stowCommand()
        );

        mechstick.rightTrigger(0.25).onTrue(
            // intake.outtakeAlgaeCommand()
            superstructure.outtakeAlgaeWithArm()
        ).onFalse(
            superstructure.stowCommand()
        );

        mechstick.back().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> {drivetrain.setNearestSourcePose(); drivetrain.enablePositionTargeting();}, drivetrain),
                drivetrain.pidToPoseContinuousCommand()
            )
        ).onFalse(
            Commands.sequence(
                Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain)
            )
        );

        mechstick.start().onTrue(
            Commands.runOnce(() -> leds.resetLEDs(), leds)
        );
    }

    public void configureTestBindings() {
        
        RobotModeTriggers.teleop().onTrue(
            Commands.sequence(
                // intake.stopIntakeCommand(),
                superstructure.normalizeCommand()// elevator.normalizeElevatorCommand(),
            )
        );

        // mechstick.povDown().onTrue(Commands.runOnce(() -> drivetrain.toggleHeadingPID()));

        drivestick.leftBumper().onTrue(
            Commands.sequence(
                superstructure.intakeCoral()// doohickey.intakeCommand()
                // superstructure.goToSourceIntakePosition()
            )
        );

        drivestick.leftTrigger(0.25).onTrue(
            Commands.sequence(
                superstructure.goToSourceIntakePosition(),
                superstructure.intakeCoral()
            )
        );

        drivestick.rightBumper().onTrue(
            this.superstructure.outtakeCoral()// doohickey.startOuttakeCommand()
        ).onFalse(
            this.superstructure.stopEndEffector()// doohickey.stopCommand()
        );

        drivestick.a().onTrue(
            this.superstructure.goToL1Command()
        );

        drivestick.x().onTrue(
            this.superstructure.goToL2Command()
        );

        drivestick.y().onTrue(
            this.superstructure.goToL3Command()
        );

        drivestick.b().onTrue(
            // superstructure.goToL4Command()
            this.superstructure.goToL4Command()
        );

        drivestick.povDown().onTrue(
            superstructure.stowCommand()
        );

        drivestick.back().and(drivestick.povLeft()).onTrue(
            superstructure.autoScoreCoral(Position.L3)
        );

        drivestick.back().and(drivestick.povRight()).onTrue(
            new SequentialCommandGroup(
                
                // new InstantCommand(() -> {this.drivetrainSetTargetPoseConstruct(); drivetrain.enablePositionTargeting();}),
                // new WaitUntilCommand(() -> !drivetrain.isTargetingPosition() || drivetrain.atTargetPose() && drivetrain.atTargetVelocity()),
                drivetrain.goToPoseCommand(),
                Commands.print("@@@@@@@@@@@@@@@@@@@@@@@@"),
                superstructure.autoScoreCoral(Position.L3),
                // elevator.setL3Command(),
                // new WaitUntilCommand(() -> elevator.withinTargetRotation(Elevator.Position.L3)),
                // doohickey.startOuttakeCommand(),
                // new WaitCommand(1),
                // doohickey.stopCommand(),
                // elevator.setSourceCommand(),
                new InstantCommand(() -> drivetrain.disablePositionTargeting())
            )
        );

        drivestick.rightStick().onTrue(
            superstructure.normalizeCommand()// elevator.normalizeElevatorCommand()
        );

        drivestick.leftStick().onTrue(
            Commands.runOnce(() ->drivetrain.disablePositionTargeting(), drivetrain)
        );

        // drivestick.rightTrigger(0.25).onTrue(
        //     Commands.sequence(
        //         superstructure.goToIntakePosition(),
        //         Commands.deadline(
        //             superstructure.intakeCoral(),
        //             intake.spasmIntakeCommand()
        //         )
        //     )
        // ).onFalse(
        //     superstructure.stowCommand()
        // );

        mechstick.leftBumper().onTrue(
            Commands.sequence(
                drivetrain.goToPoseCommand(),
                Commands.waitUntil(() -> drivetrain.atTargetPose() && drivetrain.atTargetVelocity()),
                Commands.runOnce(() -> drivetrain.disablePositionTargeting()),
                Commands.print("MMMMMMM TARGET POSE REACHED")
            )
        );

        mechstick.rightBumper().onTrue(
            Commands.sequence(
                drivetrain.goToPoseCommand(),
                // Commands.waitUntil(() -> drivetrain.settled()),
                superstructure.goToL2Command(),
                Commands.deadline(
                    new WaitCommand(2.5),
                    Commands.waitUntil(() -> superstructure.atTargets())
                ),

                // Commands.waitSeconds(5),
                Commands.runOnce(() -> drivetrain.disablePositionTargeting()),
                Commands.print("MMMMMMM TARGET POSE REACHED")
            )
        );

        mechstick.x().onTrue(
            Commands.sequence(
                superstructure.setEndEfffectorAlgaeRemovalSpeedCommand(),
                superstructure.goToL2RemoveCommand()
            )
        );

        mechstick.y().onTrue(
            Commands.sequence(
                superstructure.setEndEfffectorAlgaeRemovalSpeedCommand(),
                superstructure.goToL3RemoveCommand()
            )
        );

        // mechstick.a().onTrue(
            
        // );



        // mechstick.povUp().onTrue(
        //     intake.goL1OuttakeCommand()
        // ).onFalse(
        //     superstructure.stowCommand()
        // );

        // mechstick.y().onTrue(
        //     Commands.runOnce(
        //         () -> intake.setAlgaeStow(), intake)
        // ).onFalse(
        //     Commands.runOnce(
        //         () -> intake.setIntakeL1(), intake)
        // );

        // mechstick.b().onTrue(
        //     Commands.runOnce(() -> intake.setAlgaeOuttaekSpeed(), intake)
        // ).onFalse(
        //     intake.stopIntakeCommand()
        // );
    }

    // Pose2d targetPose = new Pose2d(5.70328981, 3.76387475, Rotation2d.fromDegrees(0));

    public void loop(){
        MaxSpeed = drivetrain.getMaxDriveSpeed();
        MaxAngularRate = RotationsPerSecond.of(drivetrain.getMaxTurnSpeed()).in(RadiansPerSecond);
    
        drive.Deadband = drivetrain.getDriveDeadBand();
        drive.RotationalDeadband = drivetrain.getTurnDeadBand();
    
        driveFacingAngle.Deadband = drivetrain.getDriveDeadBand();
        driveFacingAngle.RotationalDeadband = drivetrain.getTurnDeadBand();
        driveFacingAngle.HeadingController.setTolerance(Math.toRadians(drivetrain.getRequestedHeadingPIDTolerance()));

        superstructure.update();
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        // return Commands.sequence(
        //     superstructure.normalizeCommand(),
        //     m_chooser.getSelected()
        // );
        return m_chooser.getSelected();
    }

    public Command rumbleControllerCommand(double strength, double timeout) {
        // return Commands.print("rumble");
        return Commands.startEnd(
            () -> this.drivestick.getHID().setRumble(RumbleType.kBothRumble, strength), 
            () -> this.drivestick.getHID().setRumble(RumbleType.kBothRumble, 0)
        ).withTimeout(timeout);
    }

    /* Puts a progressive response curve on a normalized analog input
       by raising input to exponent while preserving the sign 
    */
    public static double progressiveInput(double input, double exponent)
    {
        if(input == 0) return 0;
        return Math.min(1,Math.max(-1,Math.abs(input)/input * Math.pow(Math.abs(input), exponent)));
    }

    public static double progressiveInput(double input, double exponent, boolean b) {
        return progressiveInput(input, exponent) * CommandSwerveDrivetrain.getInstance().getSingleAxisMultiplier();
    }

    public static double[] rotateBy(double x, double y, Rotation2d rotate) {
        return new double[] {
            x * rotate.getCos() - y * rotate.getSin(),
            x * rotate.getSin() + y * rotate.getCos()
        };
    }
}
