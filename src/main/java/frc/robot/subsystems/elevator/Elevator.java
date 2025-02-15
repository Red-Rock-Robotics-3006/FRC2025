package frc.robot.subsystems.elevator;

import java.util.Map;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;

public class Elevator extends SubsystemBase {
    private static Elevator instance = null;

    public static enum Position { // stores rotation values for different positions
        L4,
        L3,
        L2,
        L1,
        GROUND,
        SOURCE,
        ZERO
    }

    private static Map<Position, SmartDashboardNumber> POSITION_CONVERSIONS = Map.of(
            Position.L4, new SmartDashboardNumber("elevator/elevator-l4", 49),
            Position.L3, new SmartDashboardNumber("elevator/elevator-l3", 56),
            Position.L2, new SmartDashboardNumber("elevator/elevator-l2", 34),
            Position.L1, new SmartDashboardNumber("elevator/elevator-l1", 19),
            Position.GROUND, new SmartDashboardNumber("elevator/elevator-ground", 5),
            Position.SOURCE, new SmartDashboardNumber("elevator/elevator-source", 29),
            Position.ZERO, new SmartDashboardNumber("elevator/elevator-zero", 0));

    private final TalonFX m_elevatorLeft = new TalonFX(50, "*"); // update
    private final TalonFX m_elevatorRight = new TalonFX(51, "*"); // update

    private Slot0Configs elevatorSlot0Configs = new Slot0Configs();

    private MotionMagicConfigs elevatorMotionConfigs = new MotionMagicConfigs();

    private CurrentLimitsConfigs elevatorCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(50)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true);

    private SmartDashboardNumber elevatorMotionAccel = new SmartDashboardNumber("elevator/elevator-mm-accel", 100000); // update
    private SmartDashboardNumber elevatorMotionVel = new SmartDashboardNumber("elevator/elevator-mm-vel", 40);
    private SmartDashboardNumber elevatorMotionJerk = new SmartDashboardNumber("elevator/elevator-mm-jerk", 1000000);

    private SmartDashboardNumber elevatorKs = new SmartDashboardNumber("elevator/ks", 0.12);
    private SmartDashboardNumber elevatorKa = new SmartDashboardNumber("elevator/ka", 0);
    private SmartDashboardNumber elevatorKv = new SmartDashboardNumber("elevator/kv", 0); // to be tuned;
    private SmartDashboardNumber elevatorKp = new SmartDashboardNumber("elevator/kp", 2);
    private SmartDashboardNumber elevatorKi = new SmartDashboardNumber("elevator/ki", 0);
    private SmartDashboardNumber elevatorKd = new SmartDashboardNumber("elevator/kd", 0);

    private SmartDashboardNumber elevatorKg = new SmartDashboardNumber("elevator/kg", 0.022);

    private SmartDashboardNumber delta = new SmartDashboardNumber("elevator/delta", 5);
    private SmartDashboardNumber target = new SmartDashboardNumber("elevator/target", 0);
    private SmartDashboardNumber current = new SmartDashboardNumber("elevator/current", 0);
    private SmartDashboardNumber threshold = new SmartDashboardNumber("elevator/threshold", 0.2);
    private SmartDashboardNumber currentThreshold = new SmartDashboardNumber("elevator/current-threshold", 50);
    private SmartDashboardNumber normalizationSpeed = new SmartDashboardNumber("elevator/normalization-speed", -0.1);

                                                                                                             

    private Elevator() {
        super("Elevator");

        this.m_elevatorLeft.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withPeakForwardDutyCycle(1d)
                        .withPeakReverseDutyCycle(-1d)
                        .withNeutralMode(NeutralModeValue.Brake));

        this.m_elevatorRight.getConfigurator().apply(
                new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive)
                        .withPeakForwardDutyCycle(1d)
                        .withPeakReverseDutyCycle(-1d)
                        .withNeutralMode(NeutralModeValue.Brake));

        this.elevatorSlot0Configs = new Slot0Configs()
                .withKS(elevatorKs.getNumber())
                .withKA(elevatorKa.getNumber())
                .withKV(elevatorKv.getNumber())
                .withKP(elevatorKp.getNumber())
                .withKI(elevatorKi.getNumber())
                .withKD(elevatorKd.getNumber())
                .withKG(elevatorKg.getNumber())
                .withGravityType(GravityTypeValue.Elevator_Static);

        this.elevatorMotionConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(elevatorMotionVel.getNumber())
                .withMotionMagicAcceleration(elevatorMotionAccel.getNumber())
                .withMotionMagicJerk(elevatorMotionJerk.getNumber());

        this.m_elevatorLeft.getConfigurator().apply(elevatorSlot0Configs);
        this.m_elevatorRight.getConfigurator().apply(elevatorSlot0Configs);
        this.m_elevatorLeft.getConfigurator().apply(elevatorMotionConfigs);
        this.m_elevatorRight.getConfigurator().apply(elevatorMotionConfigs);
        this.m_elevatorLeft.getConfigurator().apply(elevatorCurrentLimitsConfigs);
        this.m_elevatorRight.getConfigurator().apply(elevatorCurrentLimitsConfigs);

        this.m_elevatorRight.setControl(new Follower(50, true)); // update

        this.m_elevatorLeft.setPosition(0);
        this.m_elevatorRight.setPosition(0);
    }

    public void setElevatorPosition(Position pos) {
        // this.m_elevatorLeft.setControl(
        //         new MotionMagicVoltage(Elevator.POSITION_CONVERSIONS.get(pos).getNumber())
        //                 .withSlot(0)
        //                 .withEnableFOC(true)
        //                 .withOverrideBrakeDurNeutral(false));
        this.setPosition(Elevator.POSITION_CONVERSIONS.get(pos).getNumber());
    }

    public void setPosition(double rotations) {
        System.out.println("GOTO: " + rotations);
        this.m_elevatorLeft.setControl(
            new MotionMagicVoltage(rotations)
                .withSlot(0)
                .withEnableFOC(true)
                .withOverrideBrakeDurNeutral(false)
        );
    }

    @Override
    public void periodic() {
        if (elevatorKs.hasChanged()
                || elevatorKa.hasChanged()
                || elevatorKv.hasChanged()
                || elevatorKp.hasChanged()
                || elevatorKi.hasChanged()
                || elevatorKd.hasChanged()
                || elevatorKg.hasChanged()) {
            elevatorSlot0Configs.kS = elevatorKs.getNumber();
            elevatorSlot0Configs.kA = elevatorKa.getNumber();
            elevatorSlot0Configs.kV = elevatorKv.getNumber();
            elevatorSlot0Configs.kP = elevatorKp.getNumber();
            elevatorSlot0Configs.kI = elevatorKi.getNumber();
            elevatorSlot0Configs.kD = elevatorKd.getNumber();
            elevatorSlot0Configs.kG = elevatorKg.getNumber();

            this.m_elevatorLeft.getConfigurator().apply(elevatorSlot0Configs);
            this.m_elevatorRight.getConfigurator().apply(elevatorSlot0Configs);

            System.out.println("applied"); // comment when necessary
        }

        if (elevatorMotionAccel.hasChanged() || elevatorMotionVel.hasChanged()) {
            elevatorMotionConfigs.MotionMagicAcceleration = elevatorMotionAccel.getNumber();
            elevatorMotionConfigs.MotionMagicCruiseVelocity = elevatorMotionVel.getNumber();
            this.m_elevatorLeft.getConfigurator().apply(elevatorMotionConfigs);
            this.m_elevatorRight.getConfigurator().apply(elevatorMotionConfigs);
        }

        if (elevatorMotionJerk.hasChanged()) {
            elevatorMotionConfigs.MotionMagicJerk = elevatorMotionJerk.getNumber();
            this.m_elevatorLeft.getConfigurator().apply(elevatorMotionConfigs);
            this.m_elevatorRight.getConfigurator().apply(elevatorMotionConfigs);
        }

        SmartDashboard.putNumber("elevator/elevator-left-position", m_elevatorLeft.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevator/elevator-right-position", m_elevatorRight.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevator/elevator-left-spike", m_elevatorLeft.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("elevator/elevator-right-spike", m_elevatorRight.getTorqueCurrent().getValueAsDouble());
        SmartDashboard.putNumber("elevator/elevator-left-velocity", m_elevatorLeft.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("elevator/elevator-right-velocity", m_elevatorRight.getVelocity().getValueAsDouble());
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

    public double getPosition() {
        return m_elevatorLeft.getPosition().getValueAsDouble();
    }

    public boolean withinTargetRotation(Position pos) {
        return Math.abs(Elevator.POSITION_CONVERSIONS.get(pos).getNumber() - this.getPosition()) < threshold.getNumber();
    }

    public boolean atCurrentSpike()
    {
        return Math.abs(this.m_elevatorLeft.getTorqueCurrent().getValueAsDouble()) > this.currentThreshold.getNumber() ||
                Math.abs(this.m_elevatorRight.getTorqueCurrent().getValueAsDouble()) > this.currentThreshold.getNumber();
    }

    public void resetMotors()
    {
        this.m_elevatorLeft.setControl(new CoastOut());
        this.m_elevatorLeft.setPosition(0);
        this.m_elevatorRight.setPosition(0);
        this.m_elevatorLeft.setControl(new DutyCycleOut(0));
    }

    public void setNormalizeSpeed()
    {
        this.m_elevatorLeft.setControl(new DutyCycleOut(this.normalizationSpeed.getNumber()));
    }

    public Command normalizeElevatorCommand()
    {
        return new FunctionalCommand(
            () -> this.setNormalizeSpeed(), 
            () -> {}, 
            (interrupted) -> this.resetMotors(), 
            () -> this.atCurrentSpike(), this);
    }

    public Command setL4Command() {
        return new InstantCommand(() -> this.setElevatorPosition(Position.L4), this);
    }

    public Command setL3Command() {
        return new InstantCommand(() -> this.setElevatorPosition(Position.L3), this);
    }

    public Command setL2Command() {
        return new InstantCommand(() -> this.setElevatorPosition(Position.L2), this);
    }

    public Command setL1Command() {
        return new InstantCommand(() -> this.setElevatorPosition(Position.L1), this);
    }

    public Command setSourceCommand() {
        return new InstantCommand(() -> this.setElevatorPosition(Position.SOURCE), this);
    }

    public Command setGroundCommand() {
        return new InstantCommand(() -> this.setElevatorPosition(Position.GROUND), this);
    }

    public Command setZeroCommand() {
        return new InstantCommand(() -> this.setElevatorPosition(Position.ZERO), this);
    }

    public static Elevator getInstance() {
        if (instance == null)
            instance = new Elevator();
        return instance;
    }
}
