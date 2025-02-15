// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.doohickey.Doohickey;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.Position;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import redrocklib.logging.SmartDashboardNumber;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private static double progressiveDriveExponent = 1.4;
    private static double progressiveTurnExponent = 1.7;

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

    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private final Doohickey doohickey = Doohickey.getInstance();
    private final Elevator elevator = Elevator.getInstance();

    /* Path follower */
    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    private SendableChooser<Command> m_chooser = new SendableChooser<>();

    private SmartDashboardNumber targetPoseX = new SmartDashboardNumber("target/target-x", 0);
    private SmartDashboardNumber targetPoseY = new SmartDashboardNumber("target/target-y", 0);
    private SmartDashboardNumber targetPoseTheta = new SmartDashboardNumber("target/target-theta", 0);


    public RobotContainer() {
        drivetrain.setSwerveRequest(this.driveFacingAngle);

        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        autoChooser.addRoutine("TestPath Auto", autoRoutines::testpath2Auto);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
        configureSelector();
    }

    public void configureSelector(){
        m_chooser.setDefaultOption("no auto", Commands.print("good luck drivers!"));

        m_chooser.addOption("TEST AUTO 1", autoRoutines.testAuto1());
        
        SmartDashboard.putData("AUTO CHOOSER", m_chooser);
    }

    private void configureBindings() {
        configureDriveBindings();
        configureMechBindings();
    }
    
    private void configureDriveBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(
              () -> {
                if (drivetrain.isTargetingPosition()) {
                    return driveFacingAngle.withVelocityX(drivetrain.getPositionPIDValueX() * MaxSpeed)
                                            .withVelocityY(drivetrain.getPositionPIDValueY() * MaxSpeed)
                                            .withTargetDirection(Rotation2d.fromDegrees(drivetrain.getTargetHeadingDegrees()));
                }
                else if (!drivetrain.getUseHeadingPID() || Math.abs(drivestick.getRightX()) > drivetrain.getTurnDeadBand()) {
                  return drive.withVelocityX(progressiveInput(-drivestick.getLeftY(),progressiveDriveExponent) * MaxSpeed)
                              .withVelocityY(progressiveInput(-drivestick.getLeftX(),progressiveDriveExponent) * MaxSpeed)
                              .withRotationalRate(progressiveInput(-drivestick.getRightX(),progressiveTurnExponent) * MaxAngularRate);
                }
                else {
                  return driveFacingAngle.withVelocityX(progressiveInput(-drivestick.getLeftY(),progressiveDriveExponent) * MaxSpeed)
                                         .withVelocityY(progressiveInput(-drivestick.getLeftX(),progressiveDriveExponent) * MaxSpeed)
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

        drivestick.povUp().and(drivestick.back()).onTrue(
            new InstantCommand(() -> drivetrain.toggleHeadingPID(), drivetrain)
        );

        // drivestick.povLeft().onTrue(
        //     new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(90), drivetrain)
        // );

        // drivestick.povUp().onTrue(
        //     new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(0), drivetrain)
        // );

        // drivestick.povRight().onTrue(
        //     new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(-90), drivetrain)
        // );

        // drivestick.povDown().onTrue(
        //     new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(180), drivetrain)
        // );

        // drivestick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // drivestick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-drivestick.getLeftY(), -drivestick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // drivestick.back().and(drivestick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // drivestick.back().and(drivestick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // drivestick.start().and(drivestick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // drivestick.start().and(drivestick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        drivestick.start().and(drivestick.back()).onTrue(drivetrain.resetHeadingCommand());

        // drivestick.back().onTrue(
        //     new FunctionalCommand(
        //         () -> {
        //             drivetrain.setTargetPose(this.constructTestTargetPose());
        //             drivetrain.enablePositionTargeting();
        //         }, 
        //         () -> {}, 
        //         (interrupted) -> {
        //             drivetrain.disablePositionTargeting();
        //         }, 
        //         () -> !drivetrain.isTargetingPosition() || drivetrain.atTargetPose())
        // ).onFalse(
        //     Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain)
        // );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void configureMechBindings() {
        
        RobotModeTriggers.teleop().onTrue(
            Commands.parallel(
                elevator.normalizeElevatorCommand()
            )
        );


        // drivestick.leftBumper().onTrue(
        //     Commands.sequence(
        //         elevator.setSourceCommand(),
        //         doohickey.intakeCommand()
        //     )
        // );

        // drivestick.rightBumper().onTrue(
        //     doohickey.startOuttakeCommand()
        // ).onFalse(
        //     Commands.sequence(
        //         doohickey.stopCommand(),
        //         elevator.setGroundCommand()
        //     )
        // );

        drivestick.rightBumper().onTrue(
            doohickey.startOuttakeCommand()
        ).onFalse(
            doohickey.stopCommand()
        );

        drivestick.a().onTrue(
            elevator.setL1Command()
        );

        drivestick.x().onTrue(
            elevator.setL2Command()
        );

        drivestick.y().onTrue(
            elevator.setL3Command()
        );

        drivestick.b().onTrue(
            elevator.setL4Command()
        );

        // drivestick.povLeft().onTrue(
        //     elevator.setGroundCommand()
        // );

        drivestick.povDown().onTrue(
            elevator.setZeroCommand()
        );

        drivestick.back().and(drivestick.povLeft()).onTrue(
            new SequentialCommandGroup(
                elevator.setL3Command(),
                new WaitUntilCommand(() -> elevator.withinTargetRotation(Elevator.Position.L3)),
                doohickey.startOuttakeCommand(),
                new WaitCommand(1),
                doohickey.stopCommand(),
                elevator.setSourceCommand()
            )
        );

        drivestick.back().and(drivestick.povRight()).onTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> {drivetrain.setTargetPose(new Pose2d(5.70328981, 3.76387475, new Rotation2d(0))); drivetrain.enablePositionTargeting();}),
                new WaitUntilCommand(() -> !drivetrain.isTargetingPosition() || drivetrain.atTargetPose() && drivetrain.atTargetVelocity()),
                new InstantCommand(() -> drivetrain.disablePositionTargeting()),
                elevator.setL3Command(),
                new WaitUntilCommand(() -> elevator.withinTargetRotation(Elevator.Position.L3)),
                doohickey.startOuttakeCommand(),
                new WaitCommand(1),
                doohickey.stopCommand(),
                elevator.setSourceCommand()
            )
        );

        drivestick.rightStick().onTrue(
            elevator.normalizeElevatorCommand()
        );

        drivestick.leftStick().onTrue(
            Commands.runOnce(() ->drivetrain.disablePositionTargeting())
        );
    }

    public void loop(){
        MaxSpeed = drivetrain.getMaxDriveSpeed();
        MaxAngularRate = RotationsPerSecond.of(drivetrain.getMaxTurnSpeed()).in(RadiansPerSecond);
    
        drive.Deadband = drivetrain.getDriveDeadBand();
        drive.RotationalDeadband = drivetrain.getTurnDeadBand();
    
        driveFacingAngle.Deadband = drivetrain.getDriveDeadBand();
        driveFacingAngle.RotationalDeadband = drivetrain.getTurnDeadBand();
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        // return m_chooser.getSelected();
        return autoChooser.selectedCommand();
    }

    private Pose2d constructTestTargetPose() {
        return new Pose2d(targetPoseX.getNumber(), targetPoseY.getNumber(), Rotation2d.fromDegrees(targetPoseTheta.getNumber()));
    }
    /* Puts a progressive response curve on a normalized analog input
       by raising input to exponent while preserving the sign 
    */
    public static double progressiveInput(double input, double exponent)
    {
        if(input == 0) return 0;
        return Math.min(1,Math.max(-1,Math.abs(input)/input * Math.pow(Math.abs(input), exponent)));
    }
}
