package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
    private SmartDashboardNumber rotateP = new SmartDashboardNumber("dt/dt-rotate-kp", 8);
    private SmartDashboardNumber rotateI = new SmartDashboardNumber("dt/dt-rotate-ki", 1.2);
    private SmartDashboardNumber rotateD = new SmartDashboardNumber("dt/dt-rotate-d", 0.3);
    private SmartDashboardNumber rotateTolerance = new SmartDashboardNumber("dt/dt-rotate-tolerance", 0.1);

    private SmartDashboardNumber rotationOmegaSignificance = new SmartDashboardNumber("dt/dt-rotation-rate-limit", 1);
    private SmartDashboardNumber driveMaxSpeed = new SmartDashboardNumber("dt/dt-max-drive-speed", 6);
    private SmartDashboardNumber turnMaxSpeed = new SmartDashboardNumber("dt/dt-max-turn-speed", 1.5);
    private SmartDashboardNumber driveDeadBand = new SmartDashboardNumber("dt/dt-drive-deadband", 0.05);
    private SmartDashboardNumber turnDeadBand = new SmartDashboardNumber("dt/dt-turn-deadband", 0.05);

    private boolean enableHeadingPID = true;
    private boolean inPositionTargeting = false;

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

    
    private SmartDashboardNumber kRejectionDistance = new SmartDashboardNumber("localization/rejection-distance", 3);
    private SmartDashboardNumber kRejectionRotationRate = new SmartDashboardNumber("localization/rejection-rotation-rate", 3);

    private SmartDashboardBoolean visionEnabled = new SmartDashboardBoolean("localization/vision-enabled", false);

    /** Swerve request to apply during field-centric path following */
    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    

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

    private SmartDashboardNumber positionKp = new SmartDashboardNumber("dt/dt-position-kp", 2);
    private SmartDashboardNumber positionKi = new SmartDashboardNumber("dt/dt-position-ki", 20);
    private SmartDashboardNumber positionKd = new SmartDashboardNumber("dt/dt-position-kd", 1);
    private SmartDashboardNumber positionIRange = new SmartDashboardNumber("dt/dt-position-Irange", 3);
    
    private SmartDashboardNumber positionTolerance = new SmartDashboardNumber("dt/dt-position-tolerance", 0.02);

    private final PIDController m_pathXController = new PIDController(10, positionKi.getNumber(), positionKd.getNumber());
    private final PIDController m_pathYController = new PIDController(10, positionKi.getNumber(), positionKd.getNumber());
    private final PIDController m_pathThetaController = new PIDController(rotateP.getNumber(), rotateI.getNumber(), rotateD.getNumber());
  
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
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
            () -> getState().Pose,
            this::resetPose,
            this::followPath,
            true,
            this,
            trajLogger
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

        // m_pathThetaController.setI(rotateI.getNumber());
        // if(m_pathThetaController.atSetpoint()) m_pathThetaController.reset();

        // m_pathXController.setI(positionKi.getNumber());
        // if(m_pathXController.atSetpoint()) m_pathXController.reset();

        m_pathYController.setI(positionKi.getNumber());
        if(m_pathYController.atSetpoint()) m_pathYController.reset();
        
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
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        this.angleRequest.HeadingController.setPID(this.rotateP.getNumber(), this.rotateI.getNumber(), this.rotateD.getNumber());
        this.angleRequest.HeadingController.setTolerance(this.rotateTolerance.getNumber());
        if(this.angleRequest.HeadingController.atSetpoint())
        {
            this.angleRequest.HeadingController.setI(0);
            this.angleRequest.HeadingController.reset();
        }
        this.positionControllerX.setPID(positionKp.getNumber(), positionKi.getNumber(), positionKd.getNumber());
        this.positionControllerY.setPID(positionKp.getNumber(), positionKi.getNumber(), positionKd.getNumber());

        this.positionControllerX.setTolerance(positionTolerance.getNumber());
        this.positionControllerY.setTolerance(positionTolerance.getNumber());
        if(this.positionControllerX.atSetpoint())
        {
            this.positionControllerX.setI(0);
            this.positionControllerX.reset();
        }
        if(this.positionControllerY.atSetpoint()) 
        {
            this.positionControllerY.setI(0);
            this.positionControllerY.reset();
        }

        
        SmartDashboard.putBoolean("dt/dt-at-target-pose", this.atTargetPose());
        SmartDashboard.putBoolean("dt/using heading pid", this.enableHeadingPID);
        SmartDashboard.putBoolean("dt/dt-is-targeting-pose", this.isTargetingPosition());
        SmartDashboard.putNumber("dt/current heading", this.getHeadingDegrees());
        SmartDashboard.putNumber("dt/target heading", this.getTargetHeadingDegrees());

        if (visionEnabled.getValue()) updateVisionMeasurements();
    }

    public void updateVisionMeasurements() {
        for (Localization.LimeLightPoseEstimateWrapper estimateWrapper : Localization.getPoseEstimates(this.getHeadingDegrees())) {
            if (estimateWrapper.tiv && poseEstimateIsValid(estimateWrapper.poseEstimate)) {
                this.addVisionMeasurement(estimateWrapper.poseEstimate.pose,
                                        estimateWrapper.poseEstimate.timestampSeconds, 
                                        estimateWrapper.getStdvs(estimateWrapper.poseEstimate.avgTagDist));
                estimateWrapper.field.setRobotPose(
                    estimateWrapper.poseEstimate.pose
                );
            }
        }
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


    public void setTargetHeadingDegrees(double degrees){
        this.targetHeadingDegrees = degrees;
    }

    public double getHeadingDegrees(){
        return this.getState().Pose.getRotation().getDegrees();
    }

    public double getTargetHeadingDegrees(){
        if (this.isTargetingPosition()) return targetPose2d.getRotation().getDegrees();
        return this.targetHeadingDegrees;
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

    public void setUseHeadingPID(boolean b){
        this.enableHeadingPID = b;
    }

    public boolean getUseHeadingPID(){
        return this.enableHeadingPID;
    }

    public void toggleHeadingPID(){
        this.enableHeadingPID = !this.enableHeadingPID;
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
    

    public double getRotationRateDegrees() {
        return this.getPigeon2().getRate();
    }

    public Pose2d getPose() {
        return this.getState().Pose;
    }

    public void setTargetPose(Pose2d pose) {
        this.targetPose2d = pose;
        this.setTargetHeadingDegrees(pose.getRotation().getDegrees());
    }

    public double getPositionPIDValueX() {
        return positionRateLimiterX.calculate(positionControllerX.calculate(getPose().getX(), targetPose2d.getX()));
    }

    public double getPositionPIDValueY() {
        return positionRateLimiterY.calculate(positionControllerY.calculate(getPose().getY(), targetPose2d.getY()));
    }

    public boolean atTargetPose() {
        return positionControllerX.atSetpoint() && positionControllerY.atSetpoint();
    }

    public boolean isTargetingPosition() {
        return this.inPositionTargeting;
    }

    public void enablePositionTargeting() {
        this.inPositionTargeting = true;
    }

    public void disablePositionTargeting() {
        this.inPositionTargeting = false;
    }
}
