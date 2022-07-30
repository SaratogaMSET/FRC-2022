package test;

import static org.junit.Assert.*;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotCodeTest {
    @Test
    public void checkRobotInit() {
        RobotContainer m_container = new RobotContainer();
        assertNotNull("Failed to initialize RobotContainer.", m_container);

        ArrayList<SubsystemBase> m_subsystems = new ArrayList<>();
        m_subsystems.add(DrivetrainSubsystem.getInstance());
        m_subsystems.add(FeederSubsystem.getInstance());
        m_subsystems.add(HangSubsystem.getInstance());
        m_subsystems.add(IntakeSubsystem.getInstance());
        m_subsystems.add(LEDSubsystem.getInstance());
        m_subsystems.add(ShooterSubsystem.getInstance());
        m_subsystems.add(VisionSubsystem.getInstance());

        for (int i = 0; i < m_subsystems.size(); ++i) {
            assertNotNull("Failed to initialize " + m_subsystems.get(i).getName(), m_subsystems.get(i));
        }
    }
}
