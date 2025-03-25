package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardBoolean;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Intake extends SubsystemBase{
    private static Intake instance = null;
    
    public static final double kStallForwardTime = 0.45;
    public static final double kStallReverseTime = 0.16;

    private SmartDashboardNumber minPivotRotation = new SmartDashboardNumber("intake/intake-min-rotation", 0.2);
    private SmartDashboardNumber maxPivotRotation = new SmartDashboardNumber("intake/intake-max-rotation", 25);


    private RedRockTalon pivotMotor = new RedRockTalon(22, "intake-pivot-motor", "*");
    private RedRockTalon intakeMotor = new RedRockTalon(21, "intake-drive-motor", "*");

    private SmartDashboardNumber intakeDeployPosition = new SmartDashboardNumber("intake/intake-deploy-position", 22.3);
    private SmartDashboardNumber intakeStowPosition = new SmartDashboardNumber("intake/intake-stow-position", 0.2);

    private SmartDashboardNumber intakel1position = new SmartDashboardNumber("intake/intake-l1-position", 2.8);

    private SmartDashboardNumber intakeAlgaeStowPosition = new SmartDashboardNumber("intake/intake-algae/stow", 3);
    private SmartDashboardNumber intakeAlgaePosition = new SmartDashboardNumber("intake/intake-algae/pos", 8.5);
    private SmartDashboardNumber intakeAlgaeSpeed = new SmartDashboardNumber("intake/intake-algae/speed", -1400);
    private SmartDashboardNumber outtakeAlgaeSpeed = new SmartDashboardNumber("intake/intake-algae/outtake-speed", 1500);
    private SmartDashboardNumber algaeHoldSpeed = new SmartDashboardNumber("intake/intake-algae/hold-speed", -800);
    private SmartDashboardNumber algaeOuttakePosition = new SmartDashboardNumber("intake/intake-algae/outtake-position", 3);
    
    private SmartDashboardNumber intakeSpeed = new SmartDashboardNumber("intake/intake-speed", 5000);
    private SmartDashboardNumber tqIntakeSpeed = new SmartDashboardNumber("intake/tq-intake-speed", 3000);
    private SmartDashboardNumber outtakeSpeed = new SmartDashboardNumber("intake/outtake-speed", -1700);
    private SmartDashboardNumber resetSpeed = new SmartDashboardNumber("intake/reset-speed", -0.05);
    private SmartDashboardNumber currentStallOuttakeSpeed = new SmartDashboardNumber("intake/tq-current-outtake-speed", -900);
    private SmartDashboardNumber velocityTolerance = new SmartDashboardNumber("intake/intake-velocity-tolerance", 60);
    private SmartDashboardNumber positionTolerance = new SmartDashboardNumber("intake/intake-position-tolerance", 0.4);

    private SmartDashboardNumber delta = new SmartDashboardNumber("intake/tuning/delta", 5);
    private SmartDashboardNumber target = new SmartDashboardNumber("intake/tuning/target", 0);

    private SmartDashboardNumber tofThreshold = new SmartDashboardNumber("intake/intake-tof-threshold", 0.1);

    private SmartDashboardBoolean usingMotionMagic = new SmartDashboardBoolean("intake/intake-using-mm", true);

    private double veloictyTarget = 0;

    private SlewRateLimiter limiter = new SlewRateLimiter(180);

    private Intake() {
        super();
                
        this.pivotMotor.withMotorOutputConfigs(
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
            .withKP(1.5)
            .withKI(0)
            .withKD(0.08)
            .withGravityType(GravityTypeValue.Arm_Cosine)
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(850)
            .withMotionMagicCruiseVelocity(150)
            .withMotionMagicJerk(10000000)
        )
        .withSpikeThreshold(17)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(false);
                
        this.intakeMotor.withMotorOutputConfigs(
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
            .withKP(0.37)
            .withKI(0)
            .withKD(0)
            .withGravityType(GravityTypeValue.Arm_Cosine)
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()

            .withMotionMagicAcceleration(1300)
            .withMotionMagicCruiseVelocity(100)
        )
        .withSpikeThreshold(39)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
        );

        this.pivotMotor.motor.setPosition(0);
    }

    public void setPivotResetSpeed() {
        this.pivotMotor.motor.setControl(new DutyCycleOut(this.resetSpeed.getNumber()));
    }

    public void resetPivot() {
        this.pivotMotor.motor.setControl(new NeutralOut());
        this.pivotMotor.motor.setPosition(-1);
    }

    public void setOuttakeSpeed() {
        this.setIntakeSpeed(outtakeSpeed.getNumber());
    }

    public void increaseTarget() {
        target.putNumber(target.getNumber() + delta.getNumber());
    }
    
    public void decreaseTarget() {
        target.putNumber(target.getNumber() - delta.getNumber());
    }

    public void setAlgaeStow() {
        this.setPosition(intakeAlgaeStowPosition.getNumber());
    }

    public void setAlgaeStowSpeed() {
        this.setIntakeSpeed(algaeHoldSpeed.getNumber());
    }

    public void setTarget() {
        this.setPosition(this.target.getNumber());
    }

    public void setPosition(double rotations) {
        this.pivotMotor.setMotionMagicPosition(MathUtil.clamp(rotations, minPivotRotation.getNumber(), maxPivotRotation.getNumber()));
    }

    public void setIntakeSpeed(double speed) {
        this.veloictyTarget = speed;
        if (usingMotionMagic.getValue()) this.intakeMotor.setMotionMagicVelocity(speed);
        else this.intakeMotor.motor.setControl(
            new VelocityVoltage(speed / 60)
            .withSlot(0)
            .withEnableFOC(true)
            .withOverrideBrakeDurNeutral(true)
        );
    }

    public void setAlgaeOuttakePosition() {
        this.setPosition(algaeOuttakePosition.getNumber());
    }

    public void setAlgaeIntakeSpeed() {
        this.setIntakeSpeed(intakeAlgaeSpeed.getNumber());
    }

    public void setAlgaeOuttaekSpeed() {
        this.setIntakeSpeed(outtakeAlgaeSpeed.getNumber());
    }

    public void setIntakeDeploy() {
        this.setPosition(intakeDeployPosition.getNumber());
    }

    public void setIntakeStow() {
        this.setPosition(intakeStowPosition.getNumber());
    }

    public void startIntake() {
        this.setIntakeSpeed(intakeSpeed.getNumber());
    }

    public void setIntakeL1() {
        this.setPosition(intakel1position.getNumber());
    }

    public void setl1OuttakeSpeed() {
        this.setIntakeSpeed(outtakeSpeed.getNumber());
    }

    public void setAlgaeIntakePosition() {
        this.setPosition(intakeAlgaePosition.getNumber());
    }

    public void setTorqueIntakeSpeed() {
        this.setIntakeSpeed(tqIntakeSpeed.getNumber());
    }

    public void stopIntake() {
        this.veloictyTarget = 0;
        // this.intakeMotor.motor.setControl(new DutyCycleOut(0));
        this.setIntakeSpeed(0);
    }

    public void setTorqueCurrentOuttakeSpeed() {
        this.veloictyTarget = currentStallOuttakeSpeed.getNumber();
        this.intakeMotor.motor.setControl(
            new VelocityVoltage(this.currentStallOuttakeSpeed.getNumber() / 60)
            .withSlot(0)
            .withEnableFOC(true)
            .withOverrideBrakeDurNeutral(true)
        );
    }

    public boolean atVelocityTarget() {
        return Math.abs((this.veloictyTarget / 60) - this.intakeMotor.motor.getVelocity().getValueAsDouble()) < this.velocityTolerance.getNumber();
    }

    public boolean atPositionTarget() {
        return Math.abs(this.pivotMotor.motor.getPosition().getValueAsDouble()) < positionTolerance.getNumber();
    }

    public boolean atSlewSpikeThreshold() {
        return Math.abs(this.getTorqueCurrent()) > this.intakeMotor.getSpikeThreshold();
    }

    public double getTorqueCurrent() {
        return limiter.calculate(this.intakeMotor.motor.getTorqueCurrent().getValueAsDouble());
    }

    @Override
    public void periodic() {
        this.intakeMotor.update();
        this.pivotMotor.update();
        SmartDashboard.putNumber("intake/intake-slew-tq-current", this.getTorqueCurrent());
        SmartDashboard.putNumber("intake/intake-velo-target", this.veloictyTarget);
    }

    public Command resetIntakePivot() {
        return Commands.sequence(
            Commands.runOnce(this::setPivotResetSpeed, this),
            Commands.waitUntil(() -> this.pivotMotor.aboveSpikeThreshold()),
            Commands.runOnce(this::resetPivot, this)
        );
    }

    public Command deployIntakeCommand() {
        return Commands.runOnce(this::setIntakeDeploy, this);
    }

    public Command stowIntakeCommand() {
        return Commands.runOnce(this::setIntakeStow, this);
    }

    public Command startIntakeCommand() {
        return Commands.runOnce(this::startIntake, this);
    }

    public Command stopIntakeCommand() {
        return Commands.runOnce(this::stopIntake, this);
    }

    public Command startOutCommand() {
        return Commands.runOnce(this::setOuttakeSpeed, this);
    }

    public Command startCurrentStallOuttakeCommand() {
        return Commands.runOnce(this::setTorqueCurrentOuttakeSpeed, this);
    }

    public Command intakel1andHoldCommand() {
        return Commands.sequence(
            this.startIntakeCommand(),
            Commands.waitUntil(() -> this.atSlewSpikeThreshold()),
            this.stopIntakeCommand()
        );
    }

    public Command intakeAlgaeAndHoldCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> this.setAlgaeIntakePosition(), this),
            // Commands.waitUntil(() -> this.atPositionTarget()),
            Commands.runOnce(() -> this.setAlgaeIntakeSpeed(), this),
            Commands.waitUntil(() -> this.atSlewSpikeThreshold()),
            Commands.runOnce(() -> this.setAlgaeStowSpeed(), this),
            Commands.runOnce(() -> this.setAlgaeStow(), this)
        );
    }

    public Command outtakeAlgaeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> this.setAlgaeOuttakePosition(), this),
            // Commands.waitUntil(() -> this.atPositionTarget()),
            Commands.runOnce(() -> this.setAlgaeOuttaekSpeed(), this)
        );
    }

    public Command goL1OuttakeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> this.setIntakeL1(), this),
            // Commands.waitUntil(() -> this.atPositionTarget()),
            Commands.runOnce(() -> this.setl1OuttakeSpeed(), this)
        );
    }

    public Command spasmIntakeCommand() {
        return Commands.sequence(
            this.startIntakeCommand(),
            Commands.waitUntil(() -> this.atSlewSpikeThreshold()),
            Commands.repeatingSequence(
                this.startCurrentStallOuttakeCommand(),
                // Commands.waitUntil(() -> this.atVelocityTarget())
                Commands.waitSeconds(kStallReverseTime),
                Commands.runOnce(() -> this.setTorqueIntakeSpeed(), this),
                Commands.waitSeconds(kStallForwardTime),
                Commands.waitUntil(() -> this.atSlewSpikeThreshold())
            )
        );
    }


    public static Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }
}
