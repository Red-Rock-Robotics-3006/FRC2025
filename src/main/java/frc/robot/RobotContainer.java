// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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

    /* Path follower */
    private final AutoFactory autoFactory;
    // private final AutoRoutines autoRoutines;
    // private final AutoChooser autoChooser = new AutoChooser();

    private SendableChooser<Command> m_chooser = new SendableChooser<>();


    public RobotContainer() {
        drivetrain.setSwerveRequest(this.driveFacingAngle);

        autoFactory = drivetrain.createAutoFactory();
        // autoRoutines = new AutoRoutines(autoFactory);

        // autoChooser.addRoutine("TestPath Auto", autoRoutines::simplePathAuto);
        // SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    public void configureSelector(){
        m_chooser.setDefaultOption("no auto", Commands.print("good luck drivers!"));
    }
    
    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(
              () -> {
                if (!drivetrain.getUseHeadingPID() || Math.abs(drivestick.getRightX()) > drivetrain.getTurnDeadBand()) {
                  return drive.withVelocityX(-drivestick.getLeftY() * MaxSpeed)
                              .withVelocityY(-drivestick.getLeftX() * MaxSpeed)
                              .withRotationalRate(-drivestick.getRightX() * MaxAngularRate);
                }
                else {
                  return driveFacingAngle.withVelocityX(-drivestick.getLeftY() * MaxSpeed)
                        .withVelocityY(-drivestick.getLeftX() * MaxSpeed)
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

        drivestick.leftBumper().onTrue(
            new InstantCommand(() -> drivetrain.toggleHeadingPID(), drivetrain)
        );

        drivestick.povLeft().onTrue(
            new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(90), drivetrain)
        );

        drivestick.povUp().onTrue(
            new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(0), drivetrain)
        );

        drivestick.povRight().onTrue(
            new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(-90), drivetrain)
        );

        drivestick.povDown().onTrue(
            new InstantCommand(() -> drivetrain.setTargetHeadingDegrees(180), drivetrain)
        );

        drivestick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        drivestick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-drivestick.getLeftY(), -drivestick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        drivestick.back().and(drivestick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        drivestick.back().and(drivestick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        drivestick.start().and(drivestick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        drivestick.start().and(drivestick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        drivestick.start().and(drivestick.back()).onTrue(drivetrain.resetHeadingCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
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
        return m_chooser.getSelected();
    }
}
