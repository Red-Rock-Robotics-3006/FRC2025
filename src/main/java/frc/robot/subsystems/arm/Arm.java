package frc.robot.subsystems.arm;

import java.util.Map;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.wrappers.RedRockTalon;
import redrocklib.logging.SmartDashboardNumber;
import frc.robot.Superstructure.Position;

/* TODO
 * Find pos values & combine accordingly
 * Tune Slot0
 * Tune MM
 * Tune tolerance
 * Tune scoreBarge
 */

public class Arm extends SubsystemBase {
    public static final double kCANCoderOffset = 0;
    public static final double kDiscontinuityPoint = 0.875;
    public static final double kRotorToSensorRatio = 68 / 10 * 68 / 16 * 48 / 9;
    public static final double kSensorToMechRatio = 1;

    private final RedRockTalon armMotor = new RedRockTalon(41, "arm-motor", "*");
    private final CANcoder cancoder = new CANcoder(42);

    private SmartDashboardNumber minRotation = new SmartDashboardNumber("arm/minRotation", 0);
    private SmartDashboardNumber maxRotation = new SmartDashboardNumber("arm/maxRotation", 0);

    private SmartDashboardNumber armTolerance = new SmartDashboardNumber("arm/arm-tolerance", 0);
    private Position targetPosition = Position.STOW;

    private static Arm instance = null;

    private SmartDashboardNumber l1Position = new SmartDashboardNumber("arm/position/arm-l1", 0);
    private SmartDashboardNumber l2Position = new SmartDashboardNumber("arm/position/arm-l2", 0);
    private SmartDashboardNumber l3Position = new SmartDashboardNumber("arm/position/arm-l3", 0);
    private SmartDashboardNumber l4Position = new SmartDashboardNumber("arm/position/arm-l4", 0);
    private SmartDashboardNumber sourcePosition = new SmartDashboardNumber("arm/position/arm-source", 0);
    private SmartDashboardNumber coralGroundPosition = new SmartDashboardNumber("arm/position/arm-coral-ground", 0);
    private SmartDashboardNumber algaeGroundPosition = new SmartDashboardNumber("arm/position/arm-algae-ground", 0);
    private SmartDashboardNumber processorPosition = new SmartDashboardNumber("arm/position/arm-processor", 0);
    private SmartDashboardNumber stowPosition = new SmartDashboardNumber("arm/position/arm-stow", 0);
    private SmartDashboardNumber bargePosition = new SmartDashboardNumber("arm/position/arm-barge", 0);
    

    private Arm(){
        super("Arm");

        this.cancoder.getConfigurator().apply(
            new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(kDiscontinuityPoint)
            .withMagnetOffset(kCANCoderOffset)
        );
    

        

        this.armMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withGravityType(GravityTypeValue.Arm_Cosine)
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(0)
            .withMotionMagicCruiseVelocity(0)
        ).withFeedbackConfigs(
            new FeedbackConfigs()
            .withFusedCANcoder(cancoder)
            .withRotorToSensorRatio(kRotorToSensorRatio)
            .withSensorToMechanismRatio(kSensorToMechRatio)
        );
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
        }
    }

    /**
     * Check if the arm is at target position
     * @return true if the arm is on target
     */
    public boolean atTarget(){
        return Math.abs(this.convertPosition(this.targetPosition)
        - this.armMotor.motor.getPosition().getValueAsDouble()) < this.armTolerance.getNumber();
    }

    /**
     * Move the arm to a clamped angle
     * @param angle the angle to mvoe to
     */
    private void goToAngle(double angle)
    {
        this.armMotor.setMotionMagicPosition(Math.max(this.minRotation.getNumber(), Math.min(this.maxRotation.getNumber(), angle)));
    }

    /**
     * Move the arm to a Position
     * @param pos the Position to move to
     * @return a Command to do so
     */
    public Command goToPosition(Position pos){ // TODO: Ensure no illegal movements
        this.targetPosition = pos;
        return Commands.runOnce(
            () -> {this.goToAngle(this.convertPosition(pos));}
        );
    }

    /**
     * Check if the arm is in danger of hitting the floor
     * @return true if the arm is too low
     */
    public boolean belowThreshold() {
        return this.armMotor.motor.getPosition().getValueAsDouble() < this.armTolerance.getNumber();
    }

    /**
     * Swing arm to score in the Barge
     * @return a Command to do so
     */
    public Command scoreBarge(){
        return goToPosition(Position.L3); // A bit jank but it should work
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
