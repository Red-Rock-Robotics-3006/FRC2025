package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.Superstructure.Position;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.subsystems.swerve.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.swerve.generated.TunerConstants2;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.Localization;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.logging.SmartDashboardBoolean;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
@Logged
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private SmartDashboardNumber rotateP = new SmartDashboardNumber("dt/dt-rotate-kp", 4);
    private SmartDashboardNumber rotateI = new SmartDashboardNumber("dt/dt-rotate-ki", 0.5); // 1.2
    private SmartDashboardNumber rotateD = new SmartDashboardNumber("dt/dt-rotate-d", 0);
    private SmartDashboardNumber rotateIRange = new SmartDashboardNumber("dt/dt-rotate-Irange", 0.2);
    private SmartDashboardNumber rotateTolerance = new SmartDashboardNumber("dt/dt-rotate-tolerance", 0.015);

    private SmartDashboardNumber rotationOmegaSignificance = new SmartDashboardNumber("dt/dt-rotation-rate-limit", 1).withTuningEnabled(false);
    private SmartDashboardNumber driveMaxSpeed = new SmartDashboardNumber("dt/dt-max-drive-speed", 6);
    private SmartDashboardNumber turnMaxSpeed = new SmartDashboardNumber("dt/dt-max-turn-speed", 1.5);
    private SmartDashboardNumber driveDeadBand = new SmartDashboardNumber("dt/dt-drive-deadband", 0.05);
    private SmartDashboardNumber turnDeadBand = new SmartDashboardNumber("dt/dt-turn-deadband", 0.05);
    private SmartDashboardNumber headingPIDTolerance = new SmartDashboardNumber("dt/dt-heading-pid-tolerance", 1.5).withTuningEnabled(false);

    private boolean enableHeadingPID = true;
    private boolean inPositionTargeting = false;
    private boolean isTargetingReef = true; //true is reef, false is source
    private boolean positionTargetOverride = false;

    private boolean usingSingleAxisDrive = false;

    private double targetHeadingDegrees = 0;

    private SwerveRequest.FieldCentricFacingAngle angleRequest;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private static CommandSwerveDrivetrain instance = null;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private Rotation2d fieldCentircOffset = Rotation2d.kZero;

    
    private SmartDashboardNumber kRejectionDistance = new SmartDashboardNumber("localization/rejection-distance", 3);
    private SmartDashboardNumber kRejectionRotationRate = new SmartDashboardNumber("localization/rejection-rotation-rate", 10);

    private SmartDashboardBoolean visionEnabled = new SmartDashboardBoolean("localization/vision-enabled", true);

    /** Swerve request to apply during field-centric path following */
    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();

    private SmartDashboardNumber pidScaleVelo = new SmartDashboardNumber("dt/dt-pid-scale-velo", 6).withTuningEnabled(false);
    

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    @Logged
    public Pose2d autoWantedPose2d;

    @Logged
    public Pose2d autoRealPose2d;

    private Pose2d targetPose2d = new Pose2d();

    private PIDController positionControllerX;
    private PIDController positionControllerY;

    private SlewRateLimiter positionRateLimiterX;
    private SlewRateLimiter positionRateLimiterY;

    private SmartDashboardNumber positionKp = new SmartDashboardNumber("dt/dt-position-kp", 0.55);
    private SmartDashboardNumber positionKi = new SmartDashboardNumber("dt/dt-position-ki", 0.7); // 15
    private SmartDashboardNumber positionKd = new SmartDashboardNumber("dt/dt-position-kd", 0);
    private SmartDashboardNumber positionIRange = new SmartDashboardNumber("dt/dt-position-Irange", 0.2);

    private SmartDashboardNumber sourcePositionKp = new SmartDashboardNumber("dt/dt-position-kp", 0.85);
    private SmartDashboardNumber sourcePositionKi = new SmartDashboardNumber("dt/dt-position-ki", 0); // 15
    private SmartDashboardNumber sourcePositionKd = new SmartDashboardNumber("dt/dt-position-kd", 0.013);
    private SmartDashboardNumber sourcePositionIRange = new SmartDashboardNumber("dt/dt-position-Irange", 0.2);

    private SmartDashboardNumber autoPositionKp = new SmartDashboardNumber("dt/dt-auto-position-kp", 6);
    private SmartDashboardNumber autoPositionKi = new SmartDashboardNumber("dt/dt-auto-position-ki", 0.4); // 15
    private SmartDashboardNumber autoPositionKd = new SmartDashboardNumber("dt/dt-auto-position-kd", 0);
    //private SmartDashboardNumber autoPositionIRange = new SmartDashboardNumber("dt/dt-auto-position-Irange", 0.2);

    private SmartDashboardNumber autoThetaKp = new SmartDashboardNumber("dt/dt-auto-theta-kp", 4);
    private SmartDashboardNumber autoThetaKi = new SmartDashboardNumber("dt/dt-auto-theta-ki", 0.3);
    private SmartDashboardNumber autoThetaKd = new SmartDashboardNumber("dt/dt-auto-theta-kd", 0);

    private SmartDashboardNumber xVelocity = new SmartDashboardNumber("dt/dt-x-velocity", 0);
    private SmartDashboardNumber yVelocity = new SmartDashboardNumber("dt/dt-y-velocity", 0);
    private SmartDashboardNumber velocityTolerance = new SmartDashboardNumber("dt/dt-velocity-tolerance", 0.3);

    private SmartDashboardNumber targetPoseX = new SmartDashboardNumber("target/target-x", 5.8);
    private SmartDashboardNumber targetPoseY = new SmartDashboardNumber("target/target-y", 3.83);
    private SmartDashboardNumber targetPoseTheta = new SmartDashboardNumber("target/target-theta", 0);


    private Field2d field2d = new Field2d();
    
    private SmartDashboardNumber positionTolerance = new SmartDashboardNumber("dt/dt-position-tolerance", 0.02);

    private final PIDController m_pathXController = new PIDController(autoPositionKp.getNumber(), autoPositionKi.getNumber(), autoPositionKd.getNumber());
    private final PIDController m_pathYController = new PIDController(autoPositionKp.getNumber(), autoPositionKi.getNumber(), autoPositionKd.getNumber());
    private final PIDController m_pathThetaController = new PIDController(autoThetaKp.getNumber(), autoThetaKi.getNumber(), autoThetaKd.getNumber());

    private SmartDashboardNumber pidMaxVelo = new SmartDashboardNumber("dt/dt-max-pid-velo", 1.7);
  
    private DriverStation.Alliance alliance = Alliance.Blue;

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public static enum ScorePose {A, B, C, D, E, F, G, H, I, J, K, L}

    private Pose2d[][] scorePosesBlue = new Pose2d[6][2], scorePosesRed = new Pose2d[6][2];


    private Pose2d blueCenter = new Pose2d(4.489323, 4.0259, new Rotation2d());
    private Pose2d redCenter = new Pose2d(13.066, 4.0259, new Rotation2d());

    private Pose2d blueSourceLeft = new Pose2d(1.133, 7.0218, Rotation2d.fromDegrees(126));
    private Pose2d blueSourceRight = new Pose2d(1.133, 1.03, Rotation2d.fromDegrees(-126));
    private Pose2d redSourceLeft = new Pose2d(16.421, 1.03, Rotation2d.fromDegrees(-54));
    private Pose2d redSourceRight = new Pose2d(16.421, 7.0218, Rotation2d.fromDegrees(54));

    private ArrayList<Pose2d> sourcePoses = new ArrayList<>();

    private Pose2d sourceOffset = new Pose2d(1.133-0.953, 1.131-1.03, Rotation2d.kZero);

    private Pose2d fieldCenter = new Pose2d(8.75665, 4.0259, Rotation2d.kZero);

    private Pose2d seedOffsetCW = new Pose2d(5.79 - 4.489323, -4.0259 + 3.86, Rotation2d.kZero);
    private Pose2d seedOffsetCCW = new Pose2d(5.79 - 4.489323, 4.0259 - 3.86, Rotation2d.kZero);

    private int reefClockSide = 0;
    
    ScorePose scorePose = ScorePose.A;

    private boolean inAuto = false;

    private CommandXboxController controller;

    private SwerveRequest.FieldCentric fieldCentricRequest;

    
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        initialize();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        initialize();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        initialize();
    }

    private void initialize() {
        // this.angleRequest.HeadingController = new PhoenixPIDController(this.rotateP.getNumber(), this.rotateI.getNumber(), this.rotateD.getNumber());
        // this.angleRequest.HeadingController.setTolerance(this.rotateTolerance.getNumber());
        // this.angleRequest.HeadingController.setIntegratorRange(-this.rotateIRange.getNumber(), this.rotateIRange.getNumber());

        positionControllerX = new PIDController(positionKp.getNumber(), positionKi.getNumber(), positionKd.getNumber());
        positionControllerY = new PIDController(positionKp.getNumber(), positionKi.getNumber(), positionKd.getNumber());

        positionControllerX.setTolerance(positionTolerance.getNumber());
        positionControllerY.setTolerance(positionTolerance.getNumber());

        positionControllerX.setIntegratorRange(-positionIRange.getNumber(), positionIRange.getNumber());
        positionControllerY.setIntegratorRange(-positionIRange.getNumber(), positionIRange.getNumber());

        positionRateLimiterX = new SlewRateLimiter(200);
        positionRateLimiterY = new SlewRateLimiter(200);

        m_pathXController.setTolerance(0.001);
        m_pathYController.setTolerance(0.001);
        m_pathThetaController.setTolerance(0.1);

        initializeReefPoses();
        
    }

    private void initializeReefPoses() {
        Pose2d blueSeedCCW = add(this.blueCenter, this.seedOffsetCCW);
        Pose2d blueSeedCW = add(this.blueCenter, this.seedOffsetCW);
        Pose2d redSeedCCW = add(this.redCenter, this.seedOffsetCCW);
        Pose2d redSeedCW = add(this.redCenter, this.seedOffsetCW);

        System.out.println(blueSeedCCW);
        System.out.println(redSeedCCW);
        for (int i = 0; i < 6; i++) {
            scorePosesBlue[i][0] = rotatePose(blueSeedCW, Rotation2d.fromDegrees(i * 60), blueCenter);
            scorePosesBlue[i][1] = rotatePose(blueSeedCCW, Rotation2d.fromDegrees(i * 60), blueCenter);
            scorePosesRed[i][0] = rotatePose(redSeedCW, Rotation2d.fromDegrees(i * 60), redCenter);
            scorePosesRed[i][1] = rotatePose(redSeedCCW, Rotation2d.fromDegrees(i * 60), redCenter);
        }

        for (int i = 0; i < 6; i++) {
            System.out.println(i);
            for (int j = 0; j < 2; j++) {
                System.out.println(scorePosesBlue[i][j]);
            }
            System.out.println();
        }

        for (int i = 0; i < 6; i++) {
            System.out.println(i);
            for (int j = 0; j < 2; j++) {
                System.out.println(scorePosesRed[i][j]);
            }
            System.out.println();
        }

        sourcePoses.add(blueSourceLeft);
        sourcePoses.add(blueSourceRight);
        sourcePoses.add(redSourceLeft);
        sourcePoses.add(redSourceRight);
    }

    public static Pose2d add(Pose2d a, Pose2d b) {
        return new Pose2d(a.getX() + b.getX(), a.getY() + b.getY(), a.getRotation().plus(b.getRotation()));
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {//TrajectoryLogger<SwerveSample> trajLogger
        return new AutoFactory(
            () -> getState().Pose,
            this::resetPose,
            this::followPath,
            true,
            this
            // trajLogger
        );
    }


    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        autoWantedPose2d = sample.getPose();
        autoRealPose2d = this.getState().Pose;
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        m_pathThetaController.setI(rotateI.getNumber());
        // if(m_pathThetaController.atSetpoint()) m_pathThetaController.reset();

        m_pathXController.setI(positionKi.getNumber());
        // if(m_pathXController.atSetpoint()) m_pathXController.reset();

        m_pathYController.setI(positionKi.getNumber());
        //if(m_pathYController.atSetpoint()) m_pathYController.reset();
        
        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        setControl(
            m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
                
        );
    }

    public void setSwerveRequest(SwerveRequest.FieldCentricFacingAngle request){
        this.angleRequest = request;
        angleRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        angleRequest.HeadingController.setTolerance(Math.toRadians(this.getRequestedHeadingPIDTolerance()));
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public void setDriveRequest(SwerveRequest.FieldCentric drive) {
        this.fieldCentricRequest = drive;
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                // setOperatorPerspectiveForward(
                //     allianceColor == Alliance.Red
                //         ? kRedAlliancePerspectiveRotation
                //         : kBlueAlliancePerspectiveRotation
                // );
                this.setAllianceColor(allianceColor);
                if (allianceColor == Alliance.Red) this.setFieldCentricOffset(kRedAlliancePerspectiveRotation);
                else this.setFieldCentricOffset(kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        this.angleRequest.HeadingController.setPID(this.rotateP.getNumber(), this.rotateI.getNumber(), this.rotateD.getNumber());
        this.angleRequest.HeadingController.setTolerance(this.rotateTolerance.getNumber());
        this.angleRequest.HeadingController.setIntegratorRange(-this.rotateIRange.getNumber(), this.rotateIRange.getNumber());
        
        if(this.angleRequest.HeadingController.atSetpoint())
        {
            this.angleRequest.HeadingController.setI(0);
            this.angleRequest.HeadingController.reset();
        }

        if (!isTargetingReef) {
            this.positionControllerX.setPID(sourcePositionKp.getNumber(), sourcePositionKi.getNumber(), sourcePositionKd.getNumber());
            this.positionControllerY.setPID(sourcePositionKp.getNumber(), sourcePositionKi.getNumber(), sourcePositionKd.getNumber());

            this.positionControllerX.setIntegratorRange(-sourcePositionIRange.getNumber(), sourcePositionIRange.getNumber());
            this.positionControllerY.setIntegratorRange(-sourcePositionIRange.getNumber(), sourcePositionIRange.getNumber());

            System.out.println("sigh aye guh ma");
        } else {
            this.positionControllerX.setPID(positionKp.getNumber(), positionKi.getNumber(), positionKd.getNumber());
            this.positionControllerY.setPID(positionKp.getNumber(), positionKi.getNumber(), positionKd.getNumber());
    
            positionControllerX.setIntegratorRange(-positionIRange.getNumber(), positionIRange.getNumber());
            positionControllerY.setIntegratorRange(-positionIRange.getNumber(), positionIRange.getNumber());
        }

        this.positionControllerX.setTolerance(positionTolerance.getNumber());
        this.positionControllerY.setTolerance(positionTolerance.getNumber());
        if(this.positionControllerX.atSetpoint() && this.positionKi.getNumber() > 0)
        {
            this.positionControllerX.setI(0);
            this.positionControllerX.reset();
            // System.out.println("reset integral");
        }
        if(this.positionControllerY.atSetpoint()) 
        {
            this.positionControllerY.setI(0);
            this.positionControllerY.reset();
            // System.out.println("reset integral");
        }

        
        SmartDashboard.putBoolean("dt/dt-at-target-pose", this.atTargetPose());
        SmartDashboard.putBoolean("dt/dt-at-target-velo", this.atTargetVelocity());
        SmartDashboard.putBoolean("dt/dt-settled", this.settled());
        SmartDashboard.putBoolean("dt/using heading pid", this.enableHeadingPID);
        SmartDashboard.putBoolean("dt/dt-is-targeting-pose", this.isTargetingPosition());
        // SmartDashboard.putNumber("dt/current heading", this.getHeadingDegrees());
        SmartDashboard.putNumber("dt/target heading", this.getTargetHeadingDegrees());

        // SmartDashboard.putBoolean("dt/heading-pid-at-setpoint", angleRequest.HeadingController.atSetpoint());
        // SmartDashboard.putNumber("dt/heading-pid-actual-tolerance", angleRequest.HeadingController.getPositionTolerance());
        // SmartDashboard.putNumber("dt/heading-pid-error", angleRequest.HeadingController.getPositionError());

        SmartDashboard.putNumber("dt/dt-position-x", this.getPose().getX());
        SmartDashboard.putNumber("dt/dt-position-y", this.getPose().getY());

        SmartDashboard.putBoolean("dt/dt-using-single-axis", this.usingSingleAxisDrive);
        SmartDashboard.putBoolean("dt/dt-position-target-override", this.positionTargetOverride);
        SmartDashboard.putBoolean("dt/dt-target-reef", this.isTargetingReef);

        SmartDashboard.putNumber("dt/dt-closest-side", getClosestReefSide((this.alliance == Alliance.Blue) ? blueCenter : redCenter, this.getPose()));
        SmartDashboard.putNumber("dt/dt-choose-side", this.reefClockSide);

        this.field2d.setRobotPose(this.targetPose2d);
        SmartDashboard.putData("dt/dt-target-pose", this.field2d);

        this.xVelocity.putNumber(this.positionControllerX.getErrorDerivative());
        this.yVelocity.putNumber(this.positionControllerY.getErrorDerivative());

        if (visionEnabled.getValue()) updateVisionMeasurements();
    }

    
    public void updateVisionMeasurements() {
        for (Localization.LimeLightPoseEstimateWrapper estimateWrapper : Localization.getPoseEstimates(this.getHeadingDegrees())) {
            if (estimateWrapper.tiv && poseEstimateIsValid(estimateWrapper.poseEstimate)) {
                this.addVisionMeasurement(estimateWrapper.poseEstimate.pose,
                                        // estimateWrapper.poseEstimate.timestampSeconds+SmartDashboard.getNumber("localization/timeoffset", Utils.getCurrentTimeSeconds()), 
                                        Utils.getCurrentTimeSeconds() - estimateWrapper.poseEstimate.latency * 0.01,
                                        estimateWrapper.getStdvs(estimateWrapper.poseEstimate.avgTagDist));
                estimateWrapper.field.setRobotPose(
                    estimateWrapper.poseEstimate.pose
                );
                SmartDashboard.putBoolean("localization/vision-accepted", true);
                SmartDashboard.putNumber(estimateWrapper.name + "/" + estimateWrapper.name + "-latency", estimateWrapper.poseEstimate.latency);
            }
            else
                SmartDashboard.putBoolean("localization/vision-accepted", false);
        }
    }

    public boolean settled()
    {
        return this.atTargetPose() && this.atTargetVelocity();
    }

    private void setFieldCentricOffset(Rotation2d offset) {
        this.fieldCentircOffset = offset;
    }

    public Rotation2d getFieldCentricOffset() {
        return this.fieldCentircOffset;
    }
    private void setAllianceColor(DriverStation.Alliance alliance) {
        this.alliance = alliance;
    }

    private boolean poseEstimateIsValid(LimelightHelpers.PoseEstimate e) {
        return e.avgTagDist < kRejectionDistance.getNumber() && Math.abs(this.getRotationRateDegrees()) < kRejectionRotationRate.getNumber();
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Command resetHeadingCommand(){
        return new InstantCommand(
            () -> {
                System.out.println("hi");
                this.resetPose(
                    new Pose2d(
                        this.getState().Pose.getX(),
                        this.getState().Pose.getY(),
                        new Rotation2d()
                    )
                );
                this.targetHeadingDegrees = 0;
            }
        );
    }

    public Command goToPoseCommand() {
        return new FunctionalCommand(
            () -> {this.setTargetPose(this.constructTestTargetPose()); this.enablePositionTargeting();}, 
            () -> {}, 
            (interrupted) -> System.out.println("@@@@@#@#@#@$U@#%(*&#(*%&@#(*$)))"), 
            () -> SmartDashboard.getBoolean("dt/dt-settled", false)
            // () -> true
        );
    }

    public Pose2d constructTestTargetPose() {
        return new Pose2d(targetPoseX.getNumber(), targetPoseY.getNumber(), Rotation2d.fromDegrees(targetPoseTheta.getNumber()));
    }

    public void setTargetHeadingDegrees(double degrees){
        this.targetHeadingDegrees = degrees;
    }

    public void setDriveController(CommandXboxController contoller) {
        this.controller = contoller;
    }

    public double getHeadingDegrees(){
        return this.getState().Pose.getRotation().getDegrees();
    }

    public double getTargetHeadingDegrees(){
        // double offset = (this.alliance == Alliance.Red) ? 180 : 0;
        double offset = 0;
        if (this.isTargetingPosition()) return targetPose2d.getRotation().getDegrees() + offset;
        return this.targetHeadingDegrees + offset;
    }

    public boolean isRotating(){
        return Math.abs(this.getPigeon2().getRate()) > this.rotationOmegaSignificance.getNumber();
    }

    public double getMaxDriveSpeed(){
        return this.driveMaxSpeed.getNumber();
    }

    public double getMaxTurnSpeed(){
        return this.turnMaxSpeed.getNumber();
    }

    public double getDriveDeadBand(){
        return this.driveDeadBand.getNumber();
    }

    public double getTurnDeadBand(){
        return this.turnDeadBand.getNumber();
    }

    public double getRequestedHeadingPIDTolerance() {
        return this.headingPIDTolerance.getNumber();
    }

    public void setUseHeadingPID(boolean b){
        this.enableHeadingPID = b;
    }

    public boolean getUseHeadingPID(){
        return this.enableHeadingPID;
    }

    public void toggleHeadingPID(){
        this.enableHeadingPID = !this.enableHeadingPID;
    }

    public void togglePositionTargetOverride() {
        this.positionTargetOverride = !this.positionTargetOverride;
    }

    /**
     * Returns drivetrain heading PID coefficients in the form of a double array with array.length == 3
     * 
     * @return Drivetrain Heading PID coeffs
     */
    public double[] getHeadingPIDCoeffs(){
        return new double[]{this.rotateP.getNumber(), this.rotateI.getNumber(), this.rotateD.getNumber()};
    }

    public static CommandSwerveDrivetrain getInstance(){
        if (instance == null) 
            // instance = TunerConstants2.createDrivetrain();
            instance = TunerConstants.createDrivetrain();
        return instance;
    }


    // TODO Remove stuff I added

    public void setBlueRightSourceTarget() {
        this.setTargetPose(blueSourceRight);
    }

    public void setBlueLeftSourceTarget() {
        this.setTargetPose(blueSourceLeft);
    }

    public void setNearestSourcePose() {
        this.setTargetPose(this.getClosestSourcePose(this.getPose()));
        this.isTargetingReef = false;
    }

    public void setNearestSourcePoseTargetHeading() {
        this.setTargetHeadingDegrees(this.getClosestSourcePose(this.getPose()).getRotation().getDegrees());
    }

    public double getRotationRateDegrees() {
        return this.getPigeon2().getRate();
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public void setTargetPose(Pose2d pose) {
        this.positionControllerX.reset();
        this.positionControllerY.reset();
        this.targetPose2d = pose;
        this.setTargetHeadingDegrees(pose.getRotation().getDegrees());
    }

    public double getMaxPIDVelocity() {
        return this.pidMaxVelo.getNumber();
    }

    public double getPositionPIDValueX() {
        return positionRateLimiterX.calculate(positionControllerX.calculate(getPose().getX(), targetPose2d.getX()));
    }

    public double getPositionPIDValueY() {
        return positionRateLimiterY.calculate(positionControllerY.calculate(getPose().getY(), targetPose2d.getY()));
    }

    // public boolean atTargetPose() {
    //     return positionControllerX.atSetpoint() && positionControllerY.atSetpoint() && this.angleRequest.HeadingController.atSetpoint();
    // }

    public boolean atTargetPose() {
        return Math.abs(this.targetPose2d.getX() - this.getPose().getX()) < positionTolerance.getNumber()
            && Math.abs(this.targetPose2d.getY() - this.getPose().getY()) < positionTolerance.getNumber();
            // && Math.abs(this.getHeadingDegrees() - this.getTargetHeadingDegrees()) < headingPIDTolerance.getNumber();
    }

    public boolean atTargetVelocity() {
        // SmartDashboard.putNumber("dt/velo-x-method", this.positionControllerX.getErrorDerivative());
        // SmartDashboard.putNumber("dt/velo-y-method", this.positionControllerY.getErrorDerivative());
        // return true;
        return Math.abs(this.positionControllerX.getErrorDerivative()) < this.velocityTolerance.getNumber() && Math.abs(this.positionControllerY.getErrorDerivative()) < this.velocityTolerance.getNumber();
    }

    public boolean isTargetingPosition() {
        return this.inPositionTargeting;
    }

    public void enablePositionTargeting() {
        this.inPositionTargeting = !positionTargetOverride;
    }

    public void disablePositionTargeting() {
        this.inPositionTargeting = false;
    }

    public boolean getPositionTargeting() {
        return this.inPositionTargeting;
    }

    public double getSingleAxisMultiplier() {
        if (usingSingleAxisDrive) return 0;
        return 1;
    }

    public void toggleUsingSingleAxis() {
        usingSingleAxisDrive = !usingSingleAxisDrive;
    }

    /**
     * 
     * @param reefSide 0 for clockwise, 1 for counter clockwise
     */
    public void setReefSide(int reefSide) {
        this.reefClockSide = MathUtil.clamp(reefSide, 0, 1);
    }

    public void setNearestRequestedReefPoseTarget() {
        boolean onBlue = (this.alliance == Alliance.Blue);
        int i = getClosestReefSide((onBlue) ? blueCenter : redCenter, this.getPose());
        if (onBlue) {
            this.setTargetPose(scorePosesBlue[i][this.reefClockSide]);
        }
        else {
            this.setTargetPose(scorePosesRed[i][this.reefClockSide]);
        }
        this.isTargetingReef = true;
    }

    public boolean getTargetingReef() {
        return this.isTargetingReef;
    }

    public Command setNearestRequestedReefPoseTargetCommand() {
        return Commands.deadline(
            new FunctionalCommand(
            () -> {},
            () -> {System.out.println("yalalallalalalala");}, 
            (interrupted) -> System.out.println("@@@@@#@#@#@$U@#%(*&#(*%&@#(*$)))"), 
            () -> SmartDashboard.getBoolean("dt/dt-settled", false)
            // () -> true
        ), 
        this.pidToPoseContinuousCommand());
    }

    public double getPIDScale() {
        return pidScaleVelo.getNumber();
    }

    public Pose2d getClosestSourcePose(Pose2d robotPose) {
        return robotPose.nearest(
            this.sourcePoses
        );
    }

    private Pose2d getClosestReefSideCenter() {
        return new Pose2d();
    }

    private Pose2d getClosestReefClockwise() {
        return Pose2d.kZero;
    }

    private Pose2d getClosestReefCounterClockWise() {
        return Pose2d.kZero;
    }

    public Command pidToPoseContinuousCommand() {
        return this.applyRequest(
            () -> angleRequest.withVelocityX(MathUtil.clamp(this.getPositionPIDValueX() * this.getPIDScale(), -this.getMaxPIDVelocity(), this.getMaxPIDVelocity()))
                                            .withVelocityY(MathUtil.clamp(this.getPositionPIDValueY() * this.getPIDScale(), -this.getMaxPIDVelocity(), this.getMaxPIDVelocity()))
                                            .withTargetDirection(Rotation2d.fromDegrees(this.getTargetHeadingDegrees()))
        );
    }

    public Command driveFacingAngleContinuousCommand() {
        return this.applyRequest(
            () -> {
                double[] drivestickValues = RobotContainer.rotateBy(-controller.getLeftY(), -controller.getLeftX(), this.getFieldCentricOffset());
            if (!this.getUseHeadingPID() || Math.abs(controller.getRightX()) > this.getTurnDeadBand())
                return fieldCentricRequest.withVelocityX(RobotContainer.progressiveInput(drivestickValues[0],RobotContainer.progressiveDriveExponent) * this.getMaxDriveSpeed())
                .withVelocityY(RobotContainer.progressiveInput(drivestickValues[1],RobotContainer.progressiveDriveExponent, true) * this.getMaxDriveSpeed())
                .withRotationalRate(RobotContainer.progressiveInput(-controller.getRightX(),RobotContainer.progressiveTurnExponent) * this.getMaxTurnSpeed() * Math.PI);
            else 
                return angleRequest.withVelocityX(RobotContainer.progressiveInput(drivestickValues[0],RobotContainer.progressiveDriveExponent) * this.getMaxDriveSpeed())
                .withVelocityY(RobotContainer.progressiveInput(drivestickValues[1], RobotContainer.progressiveDriveExponent, true) * this.getMaxDriveSpeed())
                .withTargetDirection(Rotation2d.fromDegrees(this.getTargetHeadingDegrees()));
        }
        );
    }

    public Command pidToPoseUntilCommand(BooleanSupplier bSupplier) {
        return Commands.deadline(
            Commands.waitUntil(bSupplier), 
            pidToPoseContinuousCommand());
    }

    /**
     * Finds the side of our reef we are clostest to as an int
     * <p>0 indexed, starting from the side in the +X direction, and increasing CCW</p>
     * @param reefCenter the center of our alliance's reef
     * @param robotCenter the center of the robot
     * @return the side of the reef we are closest to
     */
    public static int getClosestReefSide(Pose2d reefCenter, Pose2d robotCenter)
    {
        double rad = Math.atan2(robotCenter.getX()-reefCenter.getX(), reefCenter.getY()-robotCenter.getY()) - Math.PI/3;
        if(rad < 0) rad += 2*Math.PI;
        return (int)(rad*3/Math.PI);
    }

    public static Pose2d rotatePose(Pose2d pose, Rotation2d theta, Pose2d center) {
        double dx = pose.getX() - center.getX();
        double dy = pose.getY() - center.getY();

        return new Pose2d(
            center.getX() + dx * theta.getCos() - dy * theta.getSin(),
            center.getY() + dx * theta.getSin() + dy * theta.getCos(),
            pose.getRotation().plus(theta)
        );
    }
}
