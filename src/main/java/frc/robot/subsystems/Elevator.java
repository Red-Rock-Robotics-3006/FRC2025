package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;
import frc.robot.Superstructure.Position;

public class Elevator extends SubsystemBase {
    private static Elevator instance = null;
    private static boolean kEnableMotorTuning = false;

    private static boolean kEnablePositionTuning = true;

    private SmartDashboardNumber minRotation = new SmartDashboardNumber("elevator/min-rotation", 0);
    private SmartDashboardNumber maxRotation = new SmartDashboardNumber("elevator/max-rotation", 62);

    private SmartDashboardNumber l1Position = new SmartDashboardNumber("elevator/elevator-positions/elevator-l1", 0).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber l2Position = new SmartDashboardNumber("elevator/elevator-positions/elevator-l2", 1.5).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber l3Position = new SmartDashboardNumber("elevator/elevator-positions/elevator-l3", 21).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber l4Position = new SmartDashboardNumber("elevator/elevator-positions/elevator-l4", 61).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber sourcePosition = new SmartDashboardNumber("elevator/elevator-positions/elevator-source", 50).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber coralGroundPosition = new SmartDashboardNumber("elevator/elevator-positions/elevator-coral-ground", 22.23).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber algaeGroundPosition = new SmartDashboardNumber("elevator/elevator-positions/elevator-algae-ground", 0).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber processorPosition = new SmartDashboardNumber("elevator/elevator-positions/elevator-processor", 0).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber stowPosition = new SmartDashboardNumber("elevator/elevator-positions/elevator-stow", 0).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber bargePosition = new SmartDashboardNumber("elevator/elevator-positions/elevator-barge", 60).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber l2AlgaePosition = new SmartDashboardNumber("elevator/elevator-positions/elevator-l2-algae", 0).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber l3AlgaePosition = new SmartDashboardNumber("elevator/elevator-positions/elevator-l3-algae", 17.5).withTuningEnabled(kEnablePositionTuning);

    private SmartDashboardNumber l1StowPosition = new SmartDashboardNumber("elevator/elevator-stows/elevator-stow-l1", 0).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber l2StowPosition = new SmartDashboardNumber("elevator/elevator-stows/elevator-stow-l2", 1.5).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber l3StowPosition = new SmartDashboardNumber("elevator/elevator-stows/elevator-stow-l3", 12.5).withTuningEnabled(kEnablePositionTuning);
    private SmartDashboardNumber l4StowPosition = new SmartDashboardNumber("elevator/elevator-stows/elevator--stow-l4", 12.5).withTuningEnabled(kEnablePositionTuning);

    private SmartDashboardNumber l1min = new SmartDashboardNumber("elevator/reef-safe-zones/l1-min", 0);
    private SmartDashboardNumber l1max = new SmartDashboardNumber("elevator/reef-safe-zones/l1-max", 0);
    private SmartDashboardNumber l2min = new SmartDashboardNumber("elevator/reef-safe-zones/l2-min", 0);
    private SmartDashboardNumber l2max = new SmartDashboardNumber("elevator/reef-safe-zones/l2-max", 0);
    private SmartDashboardNumber l3min = new SmartDashboardNumber("elevator/reef-safe-zones/l3-min", 0);
    private SmartDashboardNumber l3max = new SmartDashboardNumber("elevator/reef-safe-zones/l3-max", 0);
    private SmartDashboardNumber l4min = new SmartDashboardNumber("elevator/reef-safe-zones/l4-min", 0);
    private SmartDashboardNumber l4max = new SmartDashboardNumber("elevator/reef-safe-zones/l4-max", 0);
    

    private final RedRockTalon m_elevatorLeft = new RedRockTalon(31, "elevator-left", "*");
    private final RedRockTalon m_elevatorRight = new RedRockTalon(32, "elevator-right", "*");
    

    private SmartDashboardNumber delta = new SmartDashboardNumber("elevator/elevator-tuning/delta", 5);
    private SmartDashboardNumber target = new SmartDashboardNumber("elevator/elevator-tuning/target", 0);
    private SmartDashboardNumber tolerance = new SmartDashboardNumber("elevator/tolerance", 1.5);
    private SmartDashboardNumber normalizationSpeed = new SmartDashboardNumber("elevator/normalization-speed", -0.1);
    
    @AutoLogOutput 
    private Position targetPosition = Position.STOW;
    private SmartDashboardNumber armThreshold = new SmartDashboardNumber("elevator/elevator-arm-threshold", 40);

    private SmartDashboardNumber intakeArmThreshold = new SmartDashboardNumber("elevator/elevator-thresholds/intake-threshold", 8);
    private SmartDashboardNumber autoStowThreshold = new SmartDashboardNumber("elevator/elevator-thresholds/auto-stow-threshold", 40);
    private SmartDashboardNumber intakeSourceArmThreshold = new SmartDashboardNumber("elevator/elevator-thresholds/source-threshold", 20);
                                                                                                             

    private Elevator() {
        super("Elevator");

        AutoLogOutputManager.addObject(this);

        MotorOutputConfigs elevatorMotorOutputConfigs = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake);

        Slot0Configs elevatorSlot0Configs = new Slot0Configs()
            .withKS(0.22)
            .withKA(0)
            .withKV(0)
            .withKP(3)
            .withKI(0)
            .withKD(0)
            .withKG(0.5)
            .withGravityType(GravityTypeValue.Elevator_Static);

        MotionMagicConfigs elevatorMotionConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(200)
            .withMotionMagicAcceleration(1000)
            .withMotionMagicJerk(1000000);

        CurrentLimitsConfigs elevatorCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(50)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true);
        
        double currentThreshold = 50;
        
        this.m_elevatorLeft
        .withMotorOutputConfigs(elevatorMotorOutputConfigs)
        .withSlot0Configs(elevatorSlot0Configs)
        .withMotionMagicConfigs(elevatorMotionConfigs)
        .withCurrentLimitConfigs(elevatorCurrentLimitsConfigs)
        .withSpikeThreshold(currentThreshold)
        .withTuningEnabled(kEnableMotorTuning);
        
        this.m_elevatorRight
        .withMotorOutputConfigs(elevatorMotorOutputConfigs)
        .withSlot0Configs(elevatorSlot0Configs)
        .withMotionMagicConfigs(elevatorMotionConfigs)
        .withCurrentLimitConfigs(elevatorCurrentLimitsConfigs)
        .withSpikeThreshold(currentThreshold)
        .withTuningEnabled(kEnableMotorTuning);

        this.m_elevatorRight.motor.setControl(new Follower(m_elevatorLeft.motor.getDeviceID(), true)); // update

        this.m_elevatorLeft.motor.setPosition(0);
        this.m_elevatorRight.motor.setPosition(0);
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

    public double getStowPosition(Position pos) {
        switch(pos) {
            case L1: return this.l1StowPosition.getNumber();
            case L2: return this.l2StowPosition.getNumber();
            case L3: return this.l3StowPosition.getNumber();
            case L4: return this.l4StowPosition.getNumber();
            default: return this.stowPosition.getNumber();
        }
    }

    public boolean inReefSafeZone(Position position) {
        switch (position) {
            case L1: return this.getPosition() > l1min.getNumber() && this.getPosition() < l1max.getNumber();
            case L2: return this.getPosition() > l2min.getNumber() && this.getPosition() < l2max.getNumber();
            case L3: return this.getPosition() > l3min.getNumber() && this.getPosition() < l3max.getNumber();
            case L4: return this.getPosition() > l4min.getNumber() && this.getPosition() < l4max.getNumber();
            default: return false;
        }
    }

    /**
     * Move the elevator to a Position
     * @param pos the Position to move to
     * @return a Command to do so
     */
    public Command goToPosition(Position pos) {
        return Commands.runOnce(() ->this.setPosition(pos), this);
    }
    
    public void setPosition(Position pos) {
        this.targetPosition = pos;
        System.out.println("GOTO: " + this.convertPosition(pos));
        this.m_elevatorLeft.setMotionMagicPosition(MathUtil.clamp(this.convertPosition(pos), minRotation.getNumber(), maxRotation.getNumber()));
    }

    public void setPosition(double pos) {
        this.m_elevatorLeft.setMotionMagicPosition(MathUtil.clamp(pos, minRotation.getNumber(), maxRotation.getNumber()));
    }

    @Override
    public void periodic() {

        // SmartDashboard.putNumber("elevator/elevator-left-position", this.m_elevatorLeft.motor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("elevator/elevator-right-position", this.m_elevatorRight.motor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("elevator/elevator-left-spike", this.m_elevatorLeft.motor.getTorqueCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("elevator/elevator-right-spike", this.m_elevatorRight.motor.getTorqueCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("elevator/elevator-left-velocity", this.m_elevatorLeft.motor.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("elevator/elevator-right-velocity", this.m_elevatorRight.motor.getVelocity().getValueAsDouble());

        SmartDashboard.putNumber("elevator/error", Math.abs(this.convertPosition(this.targetPosition) - this.getPosition()));
        SmartDashboard.putNumber("elevator/pos", this.getPosition());
        SmartDashboard.putNumber("elevator/target", this.convertPosition(this.targetPosition));
        SmartDashboard.putString("elevator/tpos", this.targetPosition.name());
        SmartDashboard.putBoolean("elevator/above ground intake threshold", this.aboveGroundIntakeThreshold());

        this.m_elevatorLeft.update();
        this.m_elevatorRight.update();
    }

    public void increaseTarget() {
        target.putNumber(target.getNumber() + delta.getNumber());
      }
    
    public void decreaseTarget() {
        target.putNumber(target.getNumber() - delta.getNumber());
    }

    public void setTarget() {
        // this.setPosition(this.target.getNumber());
    }

    public double getPosition() {
        return m_elevatorLeft.motor.getPosition().getValueAsDouble();
    }

    public void resetMotors()
    {
        this.m_elevatorLeft.motor.setControl(new CoastOut());
        this.m_elevatorLeft.motor.setPosition(0);
        this.m_elevatorRight.motor.setPosition(0);
        this.m_elevatorLeft.motor.setControl(new DutyCycleOut(0));
    }

    public void setNormalizeSpeed()
    {
        this.m_elevatorLeft.motor.setControl(new DutyCycleOut(this.normalizationSpeed.getNumber()));
    }

    public boolean atCurrentSpike()
    {
        return this.m_elevatorLeft.aboveSpikeThreshold() || this.m_elevatorRight.aboveSpikeThreshold();
    }

    public void setL2Stow() {
        this.setPosition(this.getStowPosition(Position.L2));
    }

    public void setL3Stow() {
        this.setPosition(this.getStowPosition(Position.L3));
    }

    public void setL4Stow() {
        this.setPosition(this.getStowPosition(Position.L4));
    }

    public void setL1Stow() {
        this.setPosition(this.getStowPosition(Position.L1));
    }
    /**
     * Check if the elevator is at target position
     * @return true if the elevator is on target
     */
    public boolean atTarget(){
        return //Math.abs(this.m_elevatorLeft.motor.getClosedLoopError().getValueAsDouble()) < tolerance.getNumber() ||
        //this.m_elevatorRight.motor.getClosedLoopError().getValueAsDouble() < tolerance.getNumber() || 
         Math.abs(this.convertPosition(this.targetPosition) - this.getPosition()) < tolerance.getNumber();
    }

    /**
     * Check if a Position may drop the arm too low
     * @param pos the Position to check
     * @return true if the Position is below a threshold
     */
    public boolean posBelowThreshold(Position pos) {
        return this.convertPosition(pos) < this.armThreshold.getNumber();
    }

    public boolean aboveGroundIntakeThreshold() {
        return this.m_elevatorLeft.motor.getPosition().getValueAsDouble() > this.intakeArmThreshold.getNumber();
    }

    public boolean belowAutoStowGroundThreshold() {
        return this.m_elevatorLeft.motor.getPosition().getValueAsDouble() < this.autoStowThreshold.getNumber();
    }

    public boolean aboveSourceIntakeThreshold() {
        return this.m_elevatorLeft.motor.getPosition().getValueAsDouble() > this.intakeSourceArmThreshold.getNumber();
    }
    
    /**
     * Move the elevator to a normal position and zero it
     * @return a Command to do so
     */
    public Command normalizeElevatorCommand()
    {
        return new FunctionalCommand(
            () -> this.setNormalizeSpeed(), 
            () -> {}, 
            (interrupted) -> this.resetMotors(), 
            () -> this.atCurrentSpike(), this);
    }


    /**
     * Get singleton instance
     * @return the Elevator
     */
    public static Elevator getInstance() {
        if (instance == null)
            instance = new Elevator();
        return instance;
    }
}
