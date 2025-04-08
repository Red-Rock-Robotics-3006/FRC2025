package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutputManager;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
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
    public static final double kCoralOuttakeWaitTime = 0.0;
    public static final double kAlgaeOuttakeWaitTime = 0.2;
    public static final double kCoralGroundIntakeTime = 0.25;
    public static final double kAlgaeGroundIntakeTime = 0.25;
    public static final double kAlgaeRemoveTime = 0.25;

    public static boolean kEnableMotorTuning = false;

    private final RedRockTalon coralDriveMotor = new RedRockTalon(51,"endeffector-drive","*");
    // private final RedRockTalon algaeDriveMotor = new RedRockTalon(52,"endeffector-drive","*"); // TODO uncomment on rebuild
    private final RedRockTalon wristMotor = new RedRockTalon(52,"endeffector-wrist","*"); // TODO re-id on rebuild 53
    private final CANrange coralTOF = new CANrange(53, "*"); // TODO re-id on rebuild 54
    // private final CANrange algaeTOF = new CANrange(55, "*"); TODO uncomment on rebuild

    private SmartDashboardNumber minRotation = new SmartDashboardNumber("endeffector/min-rotation", 0);
    private SmartDashboardNumber maxRotation = new SmartDashboardNumber("endeffector/max-rotation", 40);

    private SmartDashboardNumber coralIntakeSpeed = new SmartDashboardNumber("endeffector/coral-intake-speed-ef", -0.45);
    private SmartDashboardNumber coralOuttakeSpeed = new SmartDashboardNumber("endeffector/coral-outtake-speed", 0.2);
    private SmartDashboardNumber coralTOFThreshold = new SmartDashboardNumber("endeffector/coral-threshold", 0.09);
    private SmartDashboardNumber algaeTOFThreshold = new SmartDashboardNumber("endeffector/algae-threshold", 0.09);
    private SmartDashboardNumber normalizeSpeed = new SmartDashboardNumber("endeffector/normalize-speed", -0.1);
    private SmartDashboardNumber algaeIntakeSpeed = new SmartDashboardNumber("endeffector/algae-intake-speed", -0.35);
    private SmartDashboardNumber algaeOuttakeSpeed = new SmartDashboardNumber("endeffector/algae-outtake-speed", 0.45);
    private SmartDashboardNumber algaeRemovalSpeed = new SmartDashboardNumber("endeffector/algae-removal-speed", 0.6);
    private SmartDashboardNumber wristTolerance = new SmartDashboardNumber("endeffector/wrist-tolerance", 0.5);
    
    private Position targetPosition = Position.STOW;

    private static EndEffector instance = null;

    private SmartDashboardNumber l1Position = new SmartDashboardNumber("endeffector/position/endeffector-l1", 0);
    private SmartDashboardNumber l2Position = new SmartDashboardNumber("endeffector/position/endeffector-l2", 28.5);
    private SmartDashboardNumber l3Position = new SmartDashboardNumber("endeffector/position/endeffector-l3", 30);
    private SmartDashboardNumber l4Position = new SmartDashboardNumber("endeffector/position/endeffector-l4", 17);
    private SmartDashboardNumber sourcePosition = new SmartDashboardNumber("endeffector/position/endeffector-source", 29);
    private SmartDashboardNumber coralGroundPosition = new SmartDashboardNumber("endeffector/position/endeffector-coral-ground", 39.34);
    private SmartDashboardNumber algaeGroundPosition = new SmartDashboardNumber("endeffector/position/endeffector-algae-ground", 17.1);
    private SmartDashboardNumber processorPosition = new SmartDashboardNumber("endeffector/position/endeffector-processor", 0);
    private SmartDashboardNumber stowPosition = new SmartDashboardNumber("endeffector/position/endeffector-stow", 0);
    private SmartDashboardNumber bargePosition = new SmartDashboardNumber("endeffector/position/endeffector-barge", 0);
    private SmartDashboardNumber l2AlgaePosition = new SmartDashboardNumber("endeffector/position/endeffector-l2-algae", 35.5);
    private SmartDashboardNumber l3AlgaePosition = new SmartDashboardNumber("endeffector/position/endeffector-l3-algae", 35.5);

    private SmartDashboardNumber delta = new SmartDashboardNumber("endeffector/ef-tuning/delta", 5);
    private SmartDashboardNumber target = new SmartDashboardNumber("endeffector/ef-tuning/target", 0);

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
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
        );
        
        // this.algaeDriveMotor.withMotorOutputConfigs( // TODO uncomment on rebuild
        //     new MotorOutputConfigs()
        //     .withInverted(InvertedValue.CounterClockwise_Positive)
        //     .withPeakForwardDutyCycle(1d)
        //     .withPeakReverseDutyCycle(-1d)
        //     .withNeutralMode(NeutralModeValue.Brake)
        // )
        // .withSlot0Configs(
        //     new Slot0Configs()
        //     .withKA(0)
        //     .withKS(0.008) // TODO tune
        //     .withKV(0)
        //     .withKP(3)
        //     .withKI(0)
        //     .withKD(0)
        // )
        // .withCurrentLimitConfigs(
        //     new CurrentLimitsConfigs()
        //     .withSupplyCurrentLimit(25)
        //     .withSupplyCurrentLimitEnable(true)
        //     .withStatorCurrentLimit(60)
        //     .withStatorCurrentLimitEnable(true)
        // );
                
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
            .withKP(6)
            .withKI(0)
            .withKD(0)
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(1000)
            .withMotionMagicCruiseVelocity(200)
            .withMotionMagicJerk(10000000)
        )
        .withSpikeThreshold(30)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(60)
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
        // this.algaeDriveMotor.motor.setControl( TODO uncomment on rebuild
        //     new DutyCycleOut(speed)
        // );
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

    public void stop() {
        this.coralDriveMotor.motor.setControl(new NeutralOut());
        
        double coralPos = this.coralDriveMotor.motor.getPosition().getValueAsDouble();
        
        this.coralDriveMotor.motor.setControl(
            new PositionVoltage(coralPos)
            .withEnableFOC(true)
            .withSlot(0)
            .withOverrideBrakeDurNeutral(true)
        );

        // TODO uncomment on rebuild
        // this.algaeDriveMotor.motor.setControl(new NeutralOut());
            
        // double algaePos = this.algaeDriveMotor.motor.getPosition().getValueAsDouble();

        // this.algaeDriveMotor.motor.setControl(
        //     new PositionVoltage(algaePos)
        //     .withEnableFOC(true)
        //     .withSlot(0)
        //     .withOverrideBrakeDurNeutral(true)
        // );
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
            Math.abs(this.convertPosition(this.targetPosition) - this.wristMotor.motor.getPosition().getValueAsDouble()) < this.wristTolerance.getNumber();
    }

    @Override
    public void periodic() {
        this.coralDriveMotor.update();
        // this.algaeDriveMotor.update(); TODO uncomment on rebuild
        this.wristMotor.update();
        SmartDashboard.putNumber("endeffector/coral-canrange-val", this.coralTOF.getDistance().getValueAsDouble());
        // SmartDashboard.putNumber("endeffector/algae-canrange-val", this.algaeTOF.getDistance().getValueAsDouble()); TODO uncomment on rebuild
        SmartDashboard.putBoolean("endeffector/coral-detected", this.coralDetected());
        SmartDashboard.putBoolean("endeffector/algae-detected", this.algaeDetected());
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
        // return this.algaeTOF.getDistance().getValueAsDouble() < this.algaeTOFThreshold.getNumber(); TODO uncomment on rebuild
        return false;
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
            Commands.runOnce(() -> this.stop(), this)
        );
    }

    /**
     * Auto intake Algae
     * @return a Command to do so
     */
    public Command intakeGroundAlgae(){
        return Commands.sequence(
            Commands.runOnce(() -> this.setAlgaeIntakeSpeed(), this),
            Commands.waitUntil(() -> this.algaeDetected()),
            Commands.waitSeconds(kAlgaeGroundIntakeTime),
            Commands.runOnce(() -> this.stop(), this)
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
            new WaitUntilCommand(() -> !this.coralDetected()),
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
            new InstantCommand(this::setAlgaeIntakeSpeed, this),
            new WaitUntilCommand(() -> !this.algaeDetected()),
            new WaitCommand(kAlgaeOuttakeWaitTime),
            this.stopCommand()
        );
    }

    public Command stopCommand(){
        return this.runOnce(() -> this.stop());
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
