package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import redrocklib.wrappers.RedRockTalon;
import redrocklib.logging.SmartDashboardBoolean;
import redrocklib.logging.SmartDashboardNumber;
import frc.robot.Superstructure.Position;


public class EndEffector extends SubsystemBase {
    public static final double kCoralOuttakeWaitTime = 2;
    public static final double kAlgaeOuttakeWaitTime = 2;
    public static final double kCoralGroundIntakeTime = 0.25;
    public static final double kAlgaeGroundIntakeTime = 0.25;
    public static final double kAlgaeRemoveTime = 0.25;

    public static boolean kEnableMotorTuning = false;

    private final RedRockTalon coralDriveMotor = new RedRockTalon(51,"endeffector-coral-drive","*");
    private final RedRockTalon algaeDriveMotor = new RedRockTalon(52,"endeffector-algae-drive","*");
    private final RedRockTalon wristMotor = new RedRockTalon(53,"endeffector-wrist","*");
    private final CANrange coralTOF = new CANrange(54, "*");
    private final CANrange algaeTOF = new CANrange(55, "*"); //TODO uncomment on rebuild

    private SmartDashboardNumber minRotation = new SmartDashboardNumber("endeffector/min-rotation", 0);
    private SmartDashboardNumber maxRotation = new SmartDashboardNumber("endeffector/max-rotation", 22);

    private SmartDashboardNumber coralIntakeSpeed = new SmartDashboardNumber("endeffector/coral-intake-speed-ef", 0.5);
    private SmartDashboardNumber coralOuttakeSpeed = new SmartDashboardNumber("endeffector/coral-outtake-speed", -1);
    private SmartDashboardNumber coralTOFThreshold = new SmartDashboardNumber("endeffector/coral-threshold", 0.09);
    private SmartDashboardNumber algaeTOFThreshold = new SmartDashboardNumber("endeffector/algae-threshold", 0.2);
    private SmartDashboardNumber normalizeSpeed = new SmartDashboardNumber("endeffector/normalize-speed", -0.05);
    private SmartDashboardNumber algaeIntakeSpeed = new SmartDashboardNumber("endeffector/algae-intake-speed", 0.45);
    private SmartDashboardNumber algaeOuttakeSpeed = new SmartDashboardNumber("endeffector/algae-outtake-speed", -1);
    private SmartDashboardNumber algaeRemovalSpeed = new SmartDashboardNumber("endeffector/algae-removal-speed", 0.6);
    private SmartDashboardNumber wristTolerance = new SmartDashboardNumber("endeffector/wrist-tolerance", 1.3);
    private SmartDashboardNumber holdSpeed = new SmartDashboardNumber("endeffector/hold-speed", 20);
    private SmartDashboardNumber algaeHoldSpeed = new SmartDashboardNumber("endeffector/algae-hold", 35);
    
    private Position targetPosition = Position.STOW;

    private static EndEffector instance = null;

    private SmartDashboardNumber l1Position = new SmartDashboardNumber("endeffector/position/endeffector-l1", 0);
    private SmartDashboardNumber l2Position = new SmartDashboardNumber("endeffector/position/endeffector-l2", 2.5);
    private SmartDashboardNumber l3Position = new SmartDashboardNumber("endeffector/position/endeffector-l3", 2.5);
    private SmartDashboardNumber l4Position = new SmartDashboardNumber("endeffector/position/endeffector-l4", 0);
    private SmartDashboardNumber sourcePosition = new SmartDashboardNumber("endeffector/position/endeffector-source", 10);
    private SmartDashboardNumber coralGroundPosition = new SmartDashboardNumber("endeffector/position/endeffector-coral-ground", 10.89);
    private SmartDashboardNumber algaeGroundPosition = new SmartDashboardNumber("endeffector/position/endeffector-algae-ground", 17.31);
    private SmartDashboardNumber processorPosition = new SmartDashboardNumber("endeffector/position/endeffector-processor", 0);
    private SmartDashboardNumber stowPosition = new SmartDashboardNumber("endeffector/position/endeffector-stow", 5);
    private SmartDashboardNumber bargePosition = new SmartDashboardNumber("endeffector/position/endeffector-barge", 22);
    private SmartDashboardNumber l2AlgaePosition = new SmartDashboardNumber("endeffector/position/endeffector-l2-algae", 10.91);
    private SmartDashboardNumber l3AlgaePosition = new SmartDashboardNumber("endeffector/position/endeffector-l3-algae", 10.91);

    private SmartDashboardNumber algaeStowPosition = new SmartDashboardNumber("endeffector/position/algae-stow", 8);
    private SmartDashboardNumber algaeInBetween = new SmartDashboardNumber("endeffector/position/barge-inbetween", 10);

    private SmartDashboardNumber delta = new SmartDashboardNumber("endeffector/ef-tuning/delta", 5);
    private SmartDashboardNumber target = new SmartDashboardNumber("endeffector/ef-tuning/target", 0);

    private SmartDashboardNumber lowPassConstant = new SmartDashboardNumber("endeffector/ef-low-pass-constant", 0.2);

    private double lowPassCanrangeVal = 0;

    private EndEffector(){
        
        super("End Effector");
        AutoLogOutputManager.addObject(this);
        this.coralDriveMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0.008)
            .withKV(0)
            .withKP(3)
            .withKI(0)
            .withKD(0)
        )
        .withSpikeThreshold(40)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(25)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(40)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(false);
        
        this.algaeDriveMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0.008) // TODO tune
            .withKV(0)
            .withKP(3)
            .withKI(0)
            .withKD(0)
        )
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(25)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(false);
                
        this.wristMotor.withMotorOutputConfigs(
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
            .withKP(5)
            .withKI(0)
            .withKD(0.1)
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(400)
            .withMotionMagicCruiseVelocity(75)
            .withMotionMagicJerk(10000000)
        )
        .withSpikeThreshold(25)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(30)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(45)
            .withStatorCurrentLimitEnable(true)
        )
        .withTuningEnabled(kEnableMotorTuning);

        this.wristMotor.motor.setPosition(0);
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
        this.wristMotor.setMotionMagicPosition(MathUtil.clamp(rotation, minRotation.getNumber(), maxRotation.getNumber()));
    }

    public void setPosition(Position pos) {
        this.targetPosition = pos;
        this.wristMotor.setMotionMagicPosition(convertPosition(pos));
    }

    public void setAlgaeStowPosition() {
        this.setPosition(this.algaeStowPosition.getNumber());
    }   

    public void setBargeInbetweenPosition() {
        this.setPosition(this.algaeInBetween.getNumber());
    }

    public void setWristToCurrentPositionForTunning() {
        double pos = this.wristMotor.motor.getPosition().getValueAsDouble();
        this.wristMotor.motor.setControl(
            new PositionVoltage(pos)
            .withSlot(0)
            .withEnableFOC(true)
            .withOverrideBrakeDurNeutral(true)
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
            case L2_ALGAE:
                return this.l2AlgaePosition.getNumber();
            case L3_ALGAE:
                return this.l3AlgaePosition.getNumber();
        }
    }

    /**
     * Set the drive speed to a specified power
     * @param speed the power to drive at
     */
    private void setCoralSpeed(double speed){
        this.coralDriveMotor.motor.setControl(
            new DutyCycleOut(speed)
        );
    }


    private void setAlgaeSpeed(double speed){
        this.algaeDriveMotor.motor.setControl(
            new DutyCycleOut(speed)
        );
    }

    public void setCoralIntakeSpeed() {
        this.setCoralSpeed(this.coralIntakeSpeed.getNumber());
    }

    public void setCoralOuttakeSpeed() {
        this.setCoralSpeed(this.coralOuttakeSpeed.getNumber());
    }

    public void setAlgaeIntakeSpeed() {
        this.setAlgaeSpeed(this.algaeIntakeSpeed.getNumber());
    }

    public void setAlgaeOuttakeSpeed() {
        this.setAlgaeSpeed(this.algaeOuttakeSpeed.getNumber());
    }

    public void setAlgaeRemoveSpeed() {
        this.setAlgaeSpeed(this.algaeRemovalSpeed.getNumber());
    }

    public void setNormalizeSpeed() {
        this.wristMotor.motor.setControl(new DutyCycleOut(this.normalizeSpeed.getNumber()));
    }

    public void stopCoral() {
        this.coralDriveMotor.motor.setControl(new NeutralOut());
    }

    public void setHoldSpeed() {
        this.coralDriveMotor.motor.setControl(new TorqueCurrentFOC(holdSpeed.getNumber()));
    }

    public void setAlgaeHoldSpeed() {
        this.algaeDriveMotor.motor.setControl(new TorqueCurrentFOC(algaeHoldSpeed.getNumber()));
    }

    public void stop() {
        // this.coralDriveMotor.motor.setControl(new TorqueCurrentFOC(holdSpeed.getNumber()));
        this.coralDriveMotor.motor.setControl(new NeutralOut());
        
        double coralPos = this.coralDriveMotor.motor.getPosition().getValueAsDouble();
        
        // this.coralDriveMotor.motor.setControl(
        //     new PositionVoltage(coralPos + 0.1)
        //     .withEnableFOC(true)
        //     .withSlot(0)
        //     .withOverrideBrakeDurNeutral(true)
        // );

        // this.algaeDriveMotor.motor.setControl(new NeutralOut());
            
        // double algaePos = this.algaeDriveMotor.motor.getPosition().getValueAsDouble();

        // this.algaeDriveMotor.motor.setControl(
        //     new PositionVoltage(algaePos)
        //     .withEnableFOC(true)
        //     .withSlot(0)
        //     .withOverrideBrakeDurNeutral(true)
        // );
        // this.algaeDriveMotor.motor.setControl(new TorqueCurrentFOC(this.algaeHoldSpeed.getNumber()));
        this.algaeDriveMotor.motor.setControl(new NeutralOut());
    }

    public void resetWrist() {
        this.wristMotor.motor.setControl(new NeutralOut());
        this.wristMotor.motor.setPosition(-1);
    }
    
    /**
     * Check if the endeffector is at target position
     * @return true if the endeffector is on target
     */
    public boolean atTarget(){
        // return Math.abs(this.convertPosition(this.targetPosition)
        //     - this.wristMotor.motor.getPosition().getValueAsDouble()) < this.wristTolerance.getNumber();
        return // Math.abs(this.wristMotor.motor.getClosedLoopError().getValueAsDouble()) < this.wristTolerance.getNumber() ||
            // Math.abs(this.convertPosition(this.targetPosition) - this.wristMotor.motor.getPosition().getValueAsDouble()) < this.wristTolerance.getNumber();
            true;
    }

    @Override
    public void periodic() {
        this.coralDriveMotor.update();
        this.algaeDriveMotor.update();
        this.wristMotor.update();
        SmartDashboard.putNumber("endeffector/coral-canrange-val", this.coralTOF.getDistance().getValueAsDouble());
        SmartDashboard.putNumber("endeffector/algae-canrange-val", this.algaeTOF.getDistance().getValueAsDouble()); //TODO uncomment on rebuild
        SmartDashboard.putBoolean("endeffector/coral-detected", this.coralDetected());
        SmartDashboard.putBoolean("endeffector/algae-detected", this.algaeDetected());
        SmartDashboard.putBoolean("endeffector/ef-at-target", this.atTarget());
        double constant = lowPassConstant.getNumber();
        lowPassCanrangeVal = constant * this.algaeTOF.getDistance().getValueAsDouble() + (1 - constant) * lowPassCanrangeVal;
        SmartDashboard.putNumber("endeffector/low-pass-algae-canrange", lowPassCanrangeVal);
        Logger.recordOutput("endeffector/wrist-error", Math.abs(this.convertPosition(this.targetPosition) - this.wristMotor.motor.getPosition().getValueAsDouble()));
    }

    /**
     * Move the endeffector to a Position
     * @param pos the Position to move to
     * @return a Command to do so
     */
    public Command goToPosition(Position pos){
        return Commands.runOnce(
            () -> this.setPosition(convertPosition(pos)),
            this);
    }

    /**
     * Detect if Coral is present in the endeffector
     * @return true if coral is present
     */
    public boolean coralDetected(){
        return this.coralTOF.getDistance().getValueAsDouble() < this.coralTOFThreshold.getNumber();
    }

    /**
     * Detect if Algae is present in the endeffector
     * @return true if algae is present
     */
    public boolean algaeDetected(){ 
        return this.lowPassCanrangeVal < this.algaeTOFThreshold.getNumber(); //TODO uncomment on rebuild
        // return false;
    }

    public Command setAlgaeRemovalSpeedCommand() {
        return Commands.runOnce(() -> this.setAlgaeRemoveSpeed());
    }

    /**
     * Auto intake Coral
     * @return a Command to do so
     */
    public Command intakeCoral() {
        return Commands.sequence(
            Commands.runOnce(() -> this.setCoralIntakeSpeed(), this),
            Commands.waitUntil(() -> this.coralDetected()),
            Commands.waitSeconds(kCoralGroundIntakeTime),
            Commands.runOnce(() -> this.setHoldSpeed(), this)
        );
    }

    /**
     * Auto intake Algae
     * @return a Command to do so
     */
    public Command intakeGroundAlgae(){
        return Commands.sequence(
            // Commands.runOnce(() -> this.setAlgaeIntakeSpeed(), this),
            // Commands.waitUntil(() -> this.algaeDetected()),
            // Commands.waitSeconds(kAlgaeGroundIntakeTime),
            // Commands.runOnce(() -> this.setAlgaeHoldSpeed(), this)
            this.runOnce(() -> this.setAlgaeHoldSpeed())
        );
    }

    /**
     * Auto remove Algae
     * @return a Command to do so
     */
    public Command removeAlgae(){
        return Commands.sequence(
            Commands.runOnce(() -> this.setAlgaeRemoveSpeed(), this),
            Commands.waitUntil(() -> this.algaeDetected()),
            Commands.waitSeconds(kAlgaeRemoveTime),
            Commands.runOnce(() -> this.stop(), this)
        );
    }

    /**
     * Auto dispense Coral
     * @return a Command to do so
     */
    public Command outtakeCoral(){
        return new SequentialCommandGroup(
            new InstantCommand(this::setCoralOuttakeSpeed, this),
            // new WaitUntilCommand(() -> !this.coralDetected()),
            new WaitCommand(kCoralOuttakeWaitTime),
            this.stopCommand()
        );
    }

    /**
     * Auto dispense Algae
     * @return a Command to do so
     */
    public Command outtakeAlgae(){
        return Commands.sequence(
            new InstantCommand(this::setAlgaeOuttakeSpeed, this),
            // new WaitUntilCommand(() -> !this.algaeDetected()),
            new WaitCommand(kAlgaeOuttakeWaitTime),
            this.stopCommand()
        );
    }

    public Command stopCommand(){
        return Commands.either(
            this.runOnce(() -> this.setHoldSpeed()),
            // this.runOnce(() -> this.stop()), 
            Commands.either(
                this.runOnce(() -> this.setAlgaeHoldSpeed()), 
                this.runOnce(() -> this.stop()), 
                () -> this.algaeDetected()),
            () -> this.coralDetected());
    }

    /**
     * Move the endeffector to a normal position and zero it
     * @return a Command to do so
     */
    public Command normalizeEndEffectorCommand(){
        return new SequentialCommandGroup(
            new FunctionalCommand(
                () -> this.setNormalizeSpeed(),
                () -> {},
                (interrupted) -> this.resetWrist(),
                () -> this.wristMotor.aboveSpikeThreshold(),
                this
            )
        );
    }

    /**
     * Get singleton instance
     * @return the EndEffector
     */
    public static EndEffector getInstance()
    {
        if(instance == null)
            instance = new EndEffector();
        return instance;
    }
}
