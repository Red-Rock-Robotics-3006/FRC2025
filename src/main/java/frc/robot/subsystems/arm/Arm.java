package frc.robot.subsystems.arm;

import java.util.Map;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.wrappers.RedRockTalon;
import redrocklib.logging.SmartDashboardNumber;
import frc.robot.Superstructure.Position;

/* TODO
 * Find pos values
 * Tune Slot0
 * Tune MM
 * Tune tolerance
 * Tune scoreBarge
 */

public class Arm extends SubsystemBase {
    private final RedRockTalon armMotor = new RedRockTalon(41, "arm-motor", "*");
    private final CANcoder m_encoder = new CANcoder(42);

    private SmartDashboardNumber minRotation = new SmartDashboardNumber("arm/minRotation", 0);
    private SmartDashboardNumber maxRotation = new SmartDashboardNumber("arm/maxRotation", 0);

    private SmartDashboardNumber armTolerance = new SmartDashboardNumber("arm/arm-tolerance", 0);
    private Position targetPosition = Position.STOW;

    private static Arm instance = null;

    private static Map<Position, SmartDashboardNumber > POSITION_CONVERSIONS = Map.of(
        Position.L4, new SmartDashboardNumber("arm/position/arm-l4", 0),
        Position.L3, new SmartDashboardNumber("arm/position/arm-l3", 0),
        Position.L2, new SmartDashboardNumber("arm/position/arm-l2", 0),
        Position.L1, new SmartDashboardNumber("arm/position/arm-l1", 0),
        Position.SOURCE, new SmartDashboardNumber("arm/position/arm-source", 0),
        Position.CORAL_GROUND, new SmartDashboardNumber("arm/position/arm-coral-ground", 0),
        Position.ALGAE_GROUND, new SmartDashboardNumber("arm/position/arm-algae-ground", 0),
        Position.PROCESSOR, new SmartDashboardNumber("arm/position/arm-processor", 0),
        Position.STOW, new SmartDashboardNumber("arm/position/arm-stow", 0),
        Position.BARGE, new SmartDashboardNumber("arm/position/arm-barge", 0)
    );

    private Arm(){
        super("Arm");

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
            .withRemoteCANcoder(this.m_encoder)
        );
    }

    /**
     * Check if the arm is at target position
     * @return true if the arm is on target
     */
    public boolean atTarget(){
        return Math.abs(POSITION_CONVERSIONS.get(targetPosition).getNumber()
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
            () -> {this.goToAngle(POSITION_CONVERSIONS.get(pos).getNumber());}
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
