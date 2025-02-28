package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Superstructure.Position;

public class Elevator extends SubsystemBase {
    /**
     * Check if the elevator is at target position
     * @return true if the elevator is on target
     */
    public boolean atTarget(){return true;}
    /**
     * Move the elevator to a Position
     * @param pos the Position to move to
     * @return a Command to do so
     */
    public Command goToPosition(Position pos){return new Command() {};}
    /**
     * Check if a Position may drop the arm too low
     * @param pos the Position to check
     * @return true if the Position is below a threshold
     */
    public boolean posBelowThreshold(Position pos) {return true;}
    /**
     * Move the endeffector to a normal position and zero it
     * @return a Command to do so
     */
    public Command normalizeCommand() {return new Command() {};}
    /**
     * Get singleton instance
     * @return the Elevator
     */
    public static Elevator getInstance(){return new Elevator();}
}
