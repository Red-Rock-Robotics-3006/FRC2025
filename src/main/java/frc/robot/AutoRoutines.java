package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public Command testAuto1() {
        // return m_factory.trajectoryCmd("New Path");
        Command testpath2Command = m_factory.trajectoryCmd("testpath2");
        return Commands.sequence(
            m_factory.resetOdometry("testpath2"),
            new InstantCommand(() -> System.out.println("hi")),
            m_factory.trajectoryCmd("testpath2"),
            testpath2Command);
    }
}