// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.nio.file.attribute.PosixFilePermission;
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

    private static double kDoubleRumbleWaitTime = 0.1;
    private static double kDoubleRumbleTime = 0.1;

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

        m_chooser.addOption("left444GroundLollipop", autos.left444GroundLollipop());
        m_chooser.addOption("right444GroundLollipop", autos.right444GroundLollipop());

        m_chooser.addOption("left442SixOClock", autos.left442SixOClock());
        m_chooser.addOption("right442SixOClock", autos.right442SixOClock());

        m_chooser.addOption("left444Source", autos.left444Source());
        m_chooser.addOption("right444Source", autos.right444Source());

        m_chooser.addOption("middle4BB", autos.middle4BB());
            
        SmartDashboard.putData("AUTO CHOOSER", m_chooser);
    }

    private void configureBindings() {
        configureDriveBindings();
        // configureTestBindings();
        configureCompBindings();
        // configureArmTuning();
        // configureEndEffectorTuning();
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

        drivestick.back().and(drivestick.povRight()).onTrue(
            Commands.runOnce(() -> drivetrain.setHeadingFromMegatag1(), drivetrain)
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
                intake.stopIntakeCommand(),
                Commands.runOnce(() -> drivetrain.enableVision(), drivetrain),
                Commands.runOnce(() -> drivetrain.resetKalaman(), drivetrain)
            )
        );

        drivestick.povDown().onTrue(
            superstructure.normalizeCommand()
        );

        drivestick.leftTrigger(0.25).onTrue(
            Commands.either(
                Commands.runOnce(() -> {}),
                Commands.sequence(
                    superstructure.goToGroundIntakePosition(),
                    Commands.deadline(
                        superstructure.intakeCoral(),
                        // intake.spasmIntakeCommand()
                        intake.startIntakeCommand()
                    ),
                    this.rumbleControllerCommand(1, 0.6)
                ),
                () -> superstructure.hasAlgae()
            )
        ).onFalse(
            Commands.either(
                superstructure.stowCommand(),
                superstructure.stowReefCommand(), 
                () -> superstructure.hasAlgae()
            )
        );

        drivestick.rightBumper().onTrue(
            Commands.either(
                Commands.sequence(
                    Commands.runOnce(() -> {drivetrain.setNearestBargePoseXTarget(); drivetrain.enablePositionTargeting();}, drivetrain),
                    Commands.parallel(
                        drivetrain.pidToPoseXOnlyContinous(),
                        Commands.sequence(
                            Commands.waitUntil(() -> drivetrain.atTargetPoseX()),
                            this.rumbleControllerCommand(1, kDoubleRumbleTime),
                            Commands.waitSeconds(kDoubleRumbleWaitTime),
                            this.rumbleControllerCommand(1, kDoubleRumbleTime)
                        )
                    )
                ),
                Commands.either(
                    Commands.sequence(
                        drivetrain.algaeRemovalPIDCommand(),
                        this.rumbleControllerCommand(1, kDoubleRumbleTime),
                        Commands.waitSeconds(kDoubleRumbleWaitTime),
                        this.rumbleControllerCommand(1, kDoubleRumbleTime)
                    ),
                    Commands.sequence(
                        Commands.runOnce(() -> {drivetrain.setNearestRequestedReefPoseTarget(); drivetrain.enablePositionTargeting();}, drivetrain),
                        drivetrain.setNearestRequestedReefPoseTargetCommand(),
                        this.rumbleControllerCommand(1, kDoubleRumbleTime),
                        Commands.waitSeconds(kDoubleRumbleWaitTime),
                        this.rumbleControllerCommand(1, kDoubleRumbleTime)
                    ),
                    () -> drivetrain.isTargetingAlgaeRemoval()
                ),
                () -> superstructure.hasAlgae() || !drivetrain.isTargetingAlgaeRemoval() && superstructure.getRequestedScoringPosition() == Position.BARGE
            )
        ).onFalse(
            Commands.sequence(
                Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain)
                // superstructure.stowCommand()
            )
        );

        drivestick.leftBumper().onTrue(
            Commands.either(
                Commands.runOnce(() -> {}),
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
                ),
                () -> superstructure.hasAlgae()
            )
        ).onFalse(
            superstructure.stowCommand()
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
            // superstructure.outtakeCoral()
            Commands.sequence(
                Commands.either(
                    superstructure.outtakeAlgae(),
                    superstructure.outtakeCoral(), 
                    () -> superstructure.hasAlgae()),
                superstructure.stowCommand()               
            )
        );


        drivestick.y().onTrue(
            Commands.select(
                Map.ofEntries(
                    Map.entry(Position.STOW, superstructure.stowCommand()),
                    Map.entry(Position.L1, superstructure.goToL1Command()),
                    Map.entry(Position.L2, superstructure.goToL2Command()),
                    Map.entry(Position.L3, superstructure.goToL3Command()),
                    Map.entry(Position.L4, superstructure.goToL4Command()),
                    Map.entry(Position.BARGE, superstructure.goToBargePosition())
                ),
                () -> superstructure.getRequestedScoringPosition())
            // superstructure.goToRequestedPositionCommand()
        );

        drivestick.b().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> drivetrain.disablePositionTargeting()),
                superstructure.stowCommand()
            )
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
                Commands.runOnce(() -> drivetrain.enableAlgaeRemovalTargeting()),
                superstructure.goToL2RemoveCommand(),
                superstructure.intakeGroundAlgaeEndeffector(),
                Commands.runOnce(() -> drivetrain.disableAlgaeRemovalTargeting()),
                rumbleBothControllersCommand(1, 0.3)
            )
        // ).onFalse(
        //     superstructure.stowCommand()
        );

        mechstick.povUp().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> drivetrain.enableAlgaeRemovalTargeting()),
                // superstructure.setEndEffectorAlgaeRemovalSpeedCommand(),
                // superstructure.goToL3RemoveCommand()
                superstructure.goToL3RemoveCommand(),
                superstructure.intakeGroundAlgaeEndeffector(),
                Commands.runOnce(() -> drivetrain.disableAlgaeRemovalTargeting()),
                rumbleBothControllersCommand(1, 0.3)
            )
        // ).onFalse(
        //     superstructure.stowCommand()
        );

        mechstick.leftBumper().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> Arm.getInstance().setClimbPosition(), Arm.getInstance()), //TODO Don't use Arm!
                Commands.runOnce(() -> intake.setIntakeClimb(), intake),
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
            Commands.sequence(
                superstructure.intakeAlgaeGroundCommand(),
                rumbleBothControllersCommand(1, 0.5)
            )
        ).onFalse(
            superstructure.stowAlgaeCommand()
        );

        // mechstick.back().onTrue(
        //     Commands.sequence(
        //         Commands.runOnce(() -> {drivetrain.setNearestSourcePose(); drivetrain.enablePositionTargeting();}, drivetrain),
        //         drivetrain.pidToPoseContinuousCommand()
        //     )
        // ).onFalse(
        //     Commands.sequence(
        //         Commands.runOnce(() -> drivetrain.disablePositionTargeting(), drivetrain)
        //     )
        // );

        mechstick.back().onTrue(
            superstructure.setRequestedScoringPositionCommand(Position.BARGE)
        );

        // mechstick.start().onTrue(
        //     superstructure.setRequestedScoringPositionCommand(Position.PROCESSOR)
        // );

        // mechstick.start().onTrue(
        //     Commands.runOnce(
        //         () -> EndEffector.getInstance().setCoralIntakeSpeed()
        //         , 
        //         EndEffector.getInstance())
        // ).onFalse(
        //     EndEffector.getInstance().stopCommand()
        // );

        mechstick.start().onTrue(
            this.superstructure.setEECoralIntakeSpeedCommand()
        ).onFalse(
            this.superstructure.stopEndEffector()
        );
    }

    private void configureTestBindings() {
        
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
                superstructure.setEndEffectorAlgaeRemovalSpeedCommand(),
                superstructure.goToL2RemoveCommand()
            )
        );

        mechstick.y().onTrue(
            Commands.sequence(
                superstructure.setEndEffectorAlgaeRemovalSpeedCommand(),
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

    // private void configureArmTuning() {
    //     drivestick.povUp().onTrue(
    //         Commands.runOnce(() -> Arm.getInstance().increaseTarget(), Arm.getInstance())
    //     );
    //     drivestick.povDown().onTrue(
    //         Commands.runOnce(() -> Arm.getInstance().decreaseTarget(), Arm.getInstance())
    //     );

    //     drivestick.a().onTrue(
    //         Commands.runOnce(() -> Arm.getInstance().setTarget(), Arm.getInstance())
    //     );

    //     drivestick.b().onTrue(
    //         Commands.runOnce(() -> EndEffector.getInstance().setWristToCurrentPositionForTunning(), EndEffector.getInstance())
    //     );
    // }

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
        drivestick.b().onTrue(
            superstructure.normalizeEFCommand()
        );

        drivestick.x().onTrue(
            Commands.runOnce(() -> Arm.getInstance().setArmToCurrentPositionForTuning(), Arm.getInstance())
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
        // return Commands.startEnd(
        //     () -> this.drivestick.getHID().setRumble(RumbleType.kBothRumble, strength), 
        //     () -> this.drivestick.getHID().setRumble(RumbleType.kBothRumble, 0)
        // ).withTimeout(timeout);
        return rumbleControllerCommand(strength, timeout, false);
    }

    public Command rumbleControllerCommand(double strength, double timeout, boolean isMechController) {
        // return Commands.print("rumble");
        if (isMechController) {
            return Commands.startEnd(
                () -> this.mechstick.getHID().setRumble(RumbleType.kBothRumble, strength), 
                () -> this.mechstick.getHID().setRumble(RumbleType.kBothRumble, 0)
            ).withTimeout(timeout);
        }
        return Commands.startEnd(
            () -> this.drivestick.getHID().setRumble(RumbleType.kBothRumble, strength), 
            () -> this.drivestick.getHID().setRumble(RumbleType.kBothRumble, 0)
        ).withTimeout(timeout);
    }

    public Command rumbleBothControllersCommand(double strength, double timeout) {
        return Commands.parallel(
            rumbleControllerCommand(strength, timeout, false),
            rumbleControllerCommand(strength, timeout, true)
        );
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
