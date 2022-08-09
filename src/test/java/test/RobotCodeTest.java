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

        // There's probably a way better way to do this
        // TODO: that ^
        ArrayList<SubsystemBase> m_subsystems = new ArrayList<>();
        try {
            m_subsystems.add(DrivetrainSubsystem.getInstance());
        } catch (Exception e) {
            fail("Failed to initialize " + m_subsystems.get(0).getName());
        }
        try {
            m_subsystems.add(FeederSubsystem.getInstance());
        } catch (Exception e) {
            fail("Failed to initialize " + m_subsystems.get(1).getName());
        }
        try {
            m_subsystems.add(HangSubsystem.getInstance());
        } catch (Exception e) {
            fail("Failed to initialize " + m_subsystems.get(2).getName());
        }
        try {
            m_subsystems.add(IntakeSubsystem.getInstance());
        } catch (Exception e) {
            fail("Failed to initialize " + m_subsystems.get(3).getName());
        }
        try {
            m_subsystems.add(LEDSubsystem.getInstance());
        } catch (Exception e) {
            fail("Failed to initialize " + m_subsystems.get(4).getName());
        }
        try {
            m_subsystems.add(ShooterSubsystem.getInstance());
        } catch (Exception e) {
            fail("Failed to initialize " + m_subsystems.get(5).getName());
        }
        try {
            m_subsystems.add(VisionSubsystem.getInstance());
        } catch (Exception e) {
            fail("Failed to initialize " + m_subsystems.get(6).getName());
        }
        
        for (int i = 0; i < m_subsystems.size(); ++i) {
            assertNotNull("Failed to initialize " + m_subsystems.get(i).getName(), m_subsystems.get(i));
        }
    }
}
