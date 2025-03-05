// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.Superstructure.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import redrocklib.logging.SmartDashboardBoolean;
import redrocklib.logging.SmartDashboardNumber;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
    private final CommandXboxController mechstick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();

    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    private SendableChooser<Command> m_chooser = new SendableChooser<>();

    private SmartDashboardBoolean inPIDTolerance = new SmartDashboardBoolean("dt/dt-in-pid-tolerance", false);


    public RobotContainer() {
        drivetrain.setSwerveRequest(this.driveFacingAngle);

        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new AutoRoutines(autoFactory);

        // autoChooser.addRoutine("TestPath2 Auto", autoRoutines::testpath2Auto);

        autoChooser.addRoutine("testlong1Auto Auto", autoRoutines::testlong1Auto);
        autoChooser.addRoutine("testlong1Paths Auto", autoRoutines::testlong1Paths);
        autoChooser.addRoutine("testlong1PathsPID Auto", autoRoutines::testlong1PathsPID);
        autoChooser.addRoutine("testcuts1Auto Auto", autoRoutines::testcuts1Auto);
        autoChooser.addRoutine("testcuts1Paths Auto", autoRoutines::testcuts1Paths);
        autoChooser.addRoutine("testcuts1PathsPID Auto", autoRoutines::testcuts1PathsPID);

        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
        configureSelector();

        configureElevatorTuning();
        // configureArmTuning();
        // configureEndEffectorTuning();
    }

    public void configureSelector(){
        m_chooser.setDefaultOption("no auto", Commands.print("good luck drivers!"));

        m_chooser.addOption("TEST AUTO 1", autoRoutines.testAuto1());
        
        SmartDashboard.putData("AUTO CHOOSER", m_chooser);
    }

    private void configureBindings() {
        configureDriveBindings();
        configureTestBindings();
    }
    
    private void configureDriveBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(
              () -> {
                double[] drivestickValues = this.rotateBy(-drivestick.getLeftY(), -drivestick.getLeftX(), drivetrain.getFieldCentricOffset());
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

        drivestick.rightStick().onTrue(
            new InstantCommand(drivetrain::toggleUsingSingleAxis, drivetrain)
        );

        // drivestick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        drivestick.start().and(drivestick.back()).onTrue(drivetrain.resetHeadingCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void configureTestBindings() {
        
        RobotModeTriggers.teleop().onTrue(
            Commands.parallel(
                superstructure.normalizeCommand()// elevator.normalizeElevatorCommand()
            )
        );

        drivestick.leftBumper().onTrue(
            Commands.sequence(
                superstructure.intakeCoral(),// doohickey.intakeCommand()
                superstructure.goToSourceIntakePosition()
            )
        );

        drivestick.rightBumper().onTrue(
            superstructure.outtakeCoral()// doohickey.startOuttakeCommand()
        ).onFalse(
            superstructure.stopEndEffector()// doohickey.stopCommand()
        );

        drivestick.a().onTrue(
            superstructure.goToL1Command()
        );

        drivestick.x().onTrue(
            superstructure.goToL2Command()
        );

        drivestick.y().onTrue(
            superstructure.goToL3Command()
        );

        drivestick.b().onTrue(
            superstructure.goToL4Command()
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
            Commands.runOnce(() ->drivetrain.disablePositionTargeting())
        );
    }

    private void configureCompBindings() {
        RobotModeTriggers.teleop().onTrue(
            Commands.parallel(
                superstructure.normalizeCommand()
            )
        );

        drivestick.leftBumper().onTrue(
            Commands.waitSeconds(0) //TODO make this the intake deploy
        ).onFalse(
            Commands.waitSeconds(0) //TODO make this the intake stow
        );

        drivestick.rightBumper().onTrue(
            superstructure.goToRequestedPositionCommand() //TODO add swerve thing
        ).onFalse(
            superstructure.stowCommand() //TODO add swerve thing
        );

        drivestick.leftTrigger(0.25).onTrue(
            superstructure.goToPosition(Position.SOURCE) //TODO add swerve thing
        ).onFalse(
            superstructure.stowCommand() //TODO add swerve thing
        );

        drivestick.rightTrigger(0.25).onTrue(
            Commands.sequence(
                Commands.waitSeconds(0), //TODO make this the intake L1 score thing
                superstructure.setRequestedScoringPositionCommand(Position.L1)
            )
        ).onFalse(
            Commands.waitSeconds(0) //TODO make this the intake stow
        );

        drivestick.x().onTrue(
            superstructure.outtakeCoral()
        );
        
        drivestick.y().onTrue(
            superstructure.goToRequestedPositionCommand()
        );

        drivestick.b().onTrue(
            superstructure.stowCommand()
        );
    }

    private void configureElevatorTuning() {
        drivestick.povUp().onTrue(
            Commands.runOnce(() -> Elevator.getInstance().increaseTarget(), Elevator.getInstance())
        );
        drivestick.povDown().onTrue(
            Commands.runOnce(() -> Elevator.getInstance().decreaseTarget(), Elevator.getInstance())
        );

        drivestick.a().onTrue(
            Commands.runOnce(() -> Elevator.getInstance().setTarget(), Elevator.getInstance())
        );
    }

    private void configureArmTuning() {
        drivestick.povUp().onTrue(
            Commands.runOnce(() -> Arm.getInstance().increaseTarget(), Arm.getInstance())
        );
        drivestick.povDown().onTrue(
            Commands.runOnce(() -> Arm.getInstance().decreaseTarget(), Arm.getInstance())
        );

        drivestick.a().onTrue(
            Commands.runOnce(() -> Arm.getInstance().setTarget(), Arm.getInstance())
        );
    }

    private void configureEndEffectorTuning() {
        drivestick.povUp().onTrue(
            Commands.runOnce(() -> EndEffector.getInstance().increaseTarget(), EndEffector.getInstance())
        );
        drivestick.povDown().onTrue(
            Commands.runOnce(() -> EndEffector.getInstance().decreaseTarget(), EndEffector.getInstance())
        );

        drivestick.a().onTrue(
            Commands.runOnce(() -> EndEffector.getInstance().setTarget(), EndEffector.getInstance())
        );
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
    }

    public Command getAutonomousCommand() {
        /* Run the routine selected from the auto chooser */
        // return m_chooser.getSelected();
        return autoChooser.selectedCommand();
    }

    

    private void drivetrainSetTargetPoseConstruct() {
        drivetrain.setTargetPose(drivetrain.constructTestTargetPose());
    }
    /* Puts a progressive response curve on a normalized analog input
       by raising input to exponent while preserving the sign 
    */
    public static double progressiveInput(double input, double exponent)
    {
        if(input == 0) return 0;
        return Math.min(1,Math.max(-1,Math.abs(input)/input * Math.pow(Math.abs(input), exponent)));
    }

    public double progressiveInput(double input, double exponent, boolean b) {
        return progressiveInput(input, exponent) * drivetrain.getSingleAxisMultiplier();
    }

    public double[] rotateBy(double x, double y, Rotation2d rotate) {
        return new double[] {
            x * rotate.getCos() - y * rotate.getSin(),
            x * rotate.getSin() + y * rotate.getCos()
        };
    }
}
