package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class LED extends SubsystemBase{

    private static LED instance = null;

    private AddressableLED control = new AddressableLED(9);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(299);
    private AddressableLEDBufferView elevatorLeftView = this.buffer.createView(0, 0);
    private AddressableLEDBufferView elevatorRightView = this.buffer.createView(0, 0);
    private AddressableLEDBufferView intakeView = this.buffer.createView(0, 0);

    private CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();

    private final Color INIT_YELLOW = new Color(255, 165, 0);
    private final Color NOTE_ORANGE = new Color(255, 15, 0);
    private final Color WHITE = new Color(255, 255, 255);
    private final Color GREEN = new Color(0, 255, 0);
    private final Color BLUE = new Color(0, 0, 255);
    private final Color RED = new Color(255, 0, 0);
    private final Color MAGENTA = new Color(255, 0, 255);
    private final Color OFF = new Color(0, 0, 0);

    private int blinkControl = 0;
    private boolean swerveIsHoming = false;

    private Star[] stars = new Star[this.buffer.getLength()];
    private final float starFreq = 0.002f;
    // private final Color starColor = new Color(255, 40, 0);
    private final float starLowTemp = 800f;
    private final float starHighTemp = 5800f;
    private final float starCycleSpeed = 250f;
    private final float starCycleVariation = 150f;

    private LED() {
        super("LED");
        this.control.setLength(this.buffer.getLength());
        this.control.setColorOrder(AddressableLED.ColorOrder.kRGB);

        this.setLights(INIT_YELLOW);
        this.control.setData(buffer);
        
        this.control.start();

        SmartDashboard.putNumber("huecontrol", huethingcontrol);
        initStars();
    }

    public enum LEDState {
        SOURCE_INTAKE_HOMING,
        SOURCE_INTAKE_READY,
        REEF_HOMING,
        REEF_READY,
        HAS_CORAL,
        IDLE
    }

    private LEDState state = LEDState.IDLE;

    // public void setState(LEDState s) {
    //     state = s;
    // }

    // public Command setLEDStateCommand(LEDState s) {
    //     return Commands.runOnce(() -> this.setState(s));
    // }

    public void setLights(int r, int g, int b) {
        if (r > 255 || g > 255 || b > 255) {
            for (int i = 0; i < buffer.getLength(); i++) {
                this.buffer.setRGB(i, 255, 255, 255);
            }
        }
        else {
            for (int i = 0; i < buffer.getLength(); i++) {
                this.buffer.setRGB(i, r, g, b);
            }
        }
    }

    public void setLights(int start, int end, int r, int g, int b) {
        if (r > 255 || g > 255 || b > 255) {
            for (int i = start; i < end; i++) {
                this.buffer.setRGB(i, 255, 255, 255);
            }
        }
        else {
            for (int i = start; i < end; i++) {
                this.buffer.setRGB(i, r, g, b);
            }
        }
    }

    public void setLights(Color c) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, c);
        }
    }

    public void setLights(int start, int end, Color c) {
        for (int i = start; i < end; i++) {
            buffer.setLED(i, c);
        }
    }

    int huething = 0;
    int huethingcontrol = 3;
    private void rainbow() {
        for (var i = 0; i < buffer.getLength(); i++) {
          final var hue = (huething + (i * 180 / buffer.getLength())) % 180;
          buffer.setHSV(i, hue, 255, 50);
        }
        huething += huethingcontrol;
        huething %= 180;
    }

    private void initStars() {
        for(int i = 0; i < this.stars.length; i++) {
            this.stars[i] = new Star(starLowTemp,starHighTemp,starCycleSpeed,starCycleVariation,starFreq,i);
        }
    }

    private void processStars() {
        for(int i = 0; i < this.stars.length; i++) {
            buffer.setLED(i, stars[i].getTemperatureColor());
        }
    }

    public void increaseHueControl() {huethingcontrol++;SmartDashboard.putNumber("huecontrol", huethingcontrol);}
    public void decreaseHueControl() {huethingcontrol--;SmartDashboard.putNumber("huecontrol", huethingcontrol);}

    public void blink(Color c, int freq) {
        if (blinkControl % freq * 2 < freq) this.setLights(c);
        else this.setLights(OFF);
    }
    
    // public void setSwerveIsHoming(boolean b) {
    //     swerveIsHoming = b;
    // }

    public void periodic() {
        blinkControl++;

        if (swerve.getTargetingReef() && swerve.getPositionTargeting()) state = LEDState.REEF_HOMING;
        else if (EndEffector.getInstance().coralDetected()) state = LEDState.HAS_CORAL;
        else if (!swerve.getTargetingReef() && swerve.getPositionTargeting()) state = LEDState.SOURCE_INTAKE_HOMING;
        else state = LEDState.IDLE;

        switch(state) {
            case SOURCE_INTAKE_HOMING:
                blink(BLUE, 7);
                break;
            case SOURCE_INTAKE_READY:
                setLights(BLUE);
                break;
            case REEF_HOMING:
                blink(GREEN, 7);
                break;
            case REEF_READY:
                setLights(GREEN);
                break;
            case HAS_CORAL:
                blink(NOTE_ORANGE, 14);
                break;
            case IDLE:
                rainbow();
                break;
        }

        this.control.setData(buffer);
    }

    /**
     * Singleton architecture which returns the singular instance of LED
     * @return the instance (which is instantiated when first called)
     */
    public static LED getInstance(){
        if (instance == null) instance = new LED();
        return instance;
    }
}