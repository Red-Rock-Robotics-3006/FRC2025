package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutputManager;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.wrappers.RedRockTalon;
import redrocklib.logging.SmartDashboardNumber;
import frc.robot.Superstructure.Position;


public class Arm extends SubsystemBase {
    public static final double kCANCoderOffset = -0.29052734375;
    public static final double kDiscontinuityPoint = 0.875;
    public static final double kRotorToSensorRatio = 68 / 10 * 68 / 16 * 48 / 9;
    public static final double kSensorToMechRatio = 1;

    private static final boolean kEnableMotorTuning = true;
    private static final boolean kEnablePositionTuning = true;
    private static final boolean kEnableLimitsTuning = false;

    private final RedRockTalon armMotor = new RedRockTalon(41, "arm-motor", "*");
    private final CANcoder cancoder = new CANcoder(42, "*");

    private SmartDashboardNumber minAngleDegrees = new SmartDashboardNumber("arm/min-angle", -90).withTuningEnabled(kEnableLimitsTuning);
    private SmartDashboardNumber minRotation = new SmartDashboardNumber("arm/minRotation", -0.25).withTuningEnabled(kEnableLimitsTuning);
    private SmartDashboardNumber maxAngleDegrees = new SmartDashboardNumber("arm/max-angle", 225).withTuningEnabled(kEnableLimitsTuning);
    private SmartDashboardNumber maxRotation = new SmartDashboardNumber("arm/maxRotation", 0.625).withTuningEnabled(kEnableLimitsTuning);

    private SmartDashboardNumber floorThreshold = new SmartDashboardNumber("arm/arm-threshold-floor", -0.03);
    private SmartDashboardNumber verticalThreshold = new SmartDashboardNumber("arm/arm-threshold-vertical", 0.25);
    private Position targetPosition = Position.STOW;

    private static Arm instance = null;

    private SmartDashboardNumber armTolerance = new SmartDashboardNumber("arm/arm-tolerance", 0.025);

    private SmartDashboardNumber l1Position = new SmartDashboardNumber("arm/position/arm-l1", 118.8).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber l2Position = new SmartDashboardNumber("arm/position/arm-l2", 118.8).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber l3Position = new SmartDashboardNumber("arm/position/arm-l3", 118.8).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber l4Position = new SmartDashboardNumber("arm/position/arm-l4", 114.57).withTuningEnabled(kEnablePositionTuning); //112 @ utah
    private SmartDashboardNumber sourcePosition = new SmartDashboardNumber("arm/position/arm-source", -57).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber coralGroundPosition = new SmartDashboardNumber("arm/position/arm-coral-ground", -52).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber algaeGroundPosition = new SmartDashboardNumber("arm/position/arm-algae-ground", 155).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber processorPosition = new SmartDashboardNumber("arm/position/arm-processor", 0).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber stowPosition = new SmartDashboardNumber("arm/position/arm-stow", 73).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber bargePosition = new SmartDashboardNumber("arm/position/arm-barge", 0).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber l2AlgaePosition = new SmartDashboardNumber("arm/position/arm-l2-algae", 145).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber l3AlgaePosition = new SmartDashboardNumber("arm/position/arm-l3-algae", 145).withTuningEnabled(kEnablePositionTuning);

    private SmartDashboardNumber climbPosition = new SmartDashboardNumber("arm/position/arm-climb", 105);
    private SmartDashboardNumber algaeOuttakePosition = new SmartDashboardNumber("arm/position/arm-algae-outtake", 155);

    private SmartDashboardNumber delta = new SmartDashboardNumber("arm/arm-tuning/delta", 5);
    private SmartDashboardNumber target = new SmartDashboardNumber("arm/arm-tuning/target", 0);
    

    private Arm(){
        super("Arm");

        AutoLogOutputManager.addObject(this);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withFeedbackRemoteSensorID(cancoder.getDeviceID())
        .withRotorToSensorRatio(kRotorToSensorRatio)
        .withSensorToMechanismRatio(kSensorToMechRatio);

        this.armMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKP(200)
            .withKI(0)
            .withKD(2)
            .withKG(0.4)
            .withGravityType(GravityTypeValue.Arm_Cosine)
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(4500)
            .withMotionMagicCruiseVelocity(45000)
            .withMotionMagicJerk(20000000)
        // ).withFeedbackConfigs(
        //     new FeedbackConfigs()
        //     .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        // ).withFeedbackConfigs(
        //     new FeedbackConfigs()
        //     .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        //     .withFeedbackRemoteSensorID(cancoder.getDeviceID())
        //     .withRotorToSensorRatio(kRotorToSensorRatio)
        //     .withSensorToMechanismRatio(kSensorToMechRatio)
        ).withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(false);

        this.cancoder.getConfigurator().apply(
            new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(kDiscontinuityPoint)
            .withMagnetOffset(kCANCoderOffset)
        );

        this.armMotor.motor.getConfigurator().apply(feedbackConfigs);

    }

    public void setAlgaeOuttakePosition() {
        this.goToAngle(this.algaeOuttakePosition.getNumber());
    }

    public void increaseTarget() {
        target.putNumber(target.getNumber() + delta.getNumber());
      }
    
    public void decreaseTarget() {
        target.putNumber(target.getNumber() - delta.getNumber());
    }

    public void setTarget() {
        this.setPosition(this.target.getNumber());
    }

    public void setPosition(double rotation) {
        this.armMotor.setMotionMagicPosition(MathUtil.clamp(rotation, minRotation.getNumber(), maxRotation.getNumber()));
    }

    public void setClimbPosition() {
        this.goToAngle(this.climbPosition.getNumber());
    }

    /**
     * Converts a Position to its corresponding value
     * @param pos the Position to convert
     * @return the numerical value
     */
    private double convertPosition(Position pos)
    {
        switch (pos) {
            case L1:
                return this.l1Position.getNumber();
            case L2:
                return this.l2Position.getNumber();
            case L3:
                return this.l3Position.getNumber();
            case L4:
                return this.l4Position.getNumber();
            case SOURCE:
                return this.sourcePosition.getNumber();
            case CORAL_GROUND:
                return this.coralGroundPosition.getNumber();
            case ALGAE_GROUND:
                return this.algaeGroundPosition.getNumber();
            case PROCESSOR:
                return this.processorPosition.getNumber();
            default: // Unreachable; Just to keep the compiler from complaining
            case STOW:
                return this.stowPosition.getNumber();
            case BARGE:
                return this.bargePosition.getNumber();
            case L2_ALGAE:
                return this.l2AlgaePosition.getNumber();
            case L3_ALGAE:
                return this.l3AlgaePosition.getNumber();
        }
    }

    /**
     * Check if the arm is at target position
     * @return true if the arm is on target
     */
    public boolean atTarget(){
        return //Math.abs(this.armMotor.motor.getClosedLoopError().getValueAsDouble()) < this.armTolerance.getNumber() ||
        Math.abs(this.angleToRotations(this.convertPosition(this.targetPosition))
        - this.armMotor.motor.getPosition().getValueAsDouble()) < this.armTolerance.getNumber();
    }

    /**
     * Move the arm to a clamped angle
     * @param angle the angle to move to
     */
    private void goToAngle(double angle)
    {
        this.setPosition(angleToRotations(angle));
    }

    private double angleToRotations(double degrees) {
        return ((maxRotation.getNumber() - minRotation.getNumber()) / (maxAngleDegrees.getNumber() - minAngleDegrees.getNumber())) * (degrees - minAngleDegrees.getNumber()) + minRotation.getNumber(); 
    }

    @Override
    public void periodic() {
        this.armMotor.update();
        SmartDashboard.putBoolean("arm/arm-in-safe-zone", this.inSafeZone());
        SmartDashboard.putNumber("arm/arm-closed-loop-error", this.armMotor.motor.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putBoolean("arm/arm-at-target", this.atTarget());
        // SmartDashboard.putNumber("arm/arm-cancoder-position", this.cancoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("arm/arm-error", Math.abs(this.angleToRotations(this.convertPosition(this.targetPosition)) - this.armMotor.motor.getPosition().getValueAsDouble()));
    }

    /**
     * Move the arm to a Position
     * @param pos the Position to move to
     * @return a Command to do so
     */
    public Command goToPosition(Position pos){
        return Commands.runOnce(
            () -> {this.setPosition(pos);}, this
        );
    }

    public void setPosition(Position pos) {
        this.targetPosition = pos;
        this.goToAngle(this.convertPosition(pos));
    }


    /**
     * Check if the arm is in danger of hitting the floor
     * @return true if the arm is too low
     */
    public boolean inSafeZone() {
        return this.armMotor.motor.getPosition().getValueAsDouble() > this.floorThreshold.getNumber() 
        && this.armMotor.motor.getPosition().getValueAsDouble() < this.verticalThreshold.getNumber();
    }

    public boolean belowFloorThreshold() {
        return this.armMotor.motor.getPosition().getValueAsDouble() < this.floorThreshold.getNumber();
    }

    /**
     * Swing arm to score in the Barge
     * @return a Command to do so
     */
    public Command scoreBarge(){
        return this.goToPosition(Position.L3); // TODO Consider deprecation and removal
    }

    /**
     * Get singleton instance
     * @return the Arm
     */
    public static Arm getInstance()
    {
        if(instance == null)
            instance = new Arm();
        return instance;
    }
}
