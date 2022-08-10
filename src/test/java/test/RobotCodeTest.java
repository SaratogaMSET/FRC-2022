package test;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.fail;

import org.junit.jupiter.api.Test;

import frc.robot.RobotContainer;

public class RobotCodeTest {
    @Test
    public void checkRobotInit() {
        RobotContainer m_container = null;
        try {
            m_container = new RobotContainer();
        } catch (Exception e) {
            fail("Failed to initialize RobotContainer.");
            System.out.println();
            System.out.println("Stacktrace: ");
            System.out.println(e);
        }
        assertNotNull("Failed to initialize RobotContainer.", m_container);

        // There's probably a way better way to do this
        // TODO: that ^
        /* ArrayList<SubsystemBase> m_subsystems = new ArrayList<>();
        try {
            m_subsystems.add(new VisionSubsystem());
        } catch (Exception e) {
            System.out.println();
            System.out.println("Stacktrace: ");
            System.out.println(e);
            fail("Failed to initialize " + m_subsystems.get(0).getName());
        }
        try {
            m_subsystems.add(new FeederSubsystem());
        } catch (Exception e) {
            System.out.println();
            System.out.println("Stacktrace: ");
            System.out.println(e);
            fail("Failed to initialize " + m_subsystems.get(1).getName());
        }
        try {
            m_subsystems.add(new HangSubsystem());
        } catch (Exception e) {
            System.out.println();
            System.out.println("Stacktrace: ");
            System.out.println(e);
            fail("Failed to initialize " + m_subsystems.get(2).getName());
        }
        try {
            m_subsystems.add(new IntakeSubsystem());
        } catch (Exception e) {
            System.out.println();
            System.out.println("Stacktrace: ");
            System.out.println(e);
            fail("Failed to initialize " + m_subsystems.get(3).getName());
        }
        try {
            m_subsystems.add(new LEDSubsystem());
        } catch (Exception e) {
            System.out.println();
            System.out.println("Stacktrace: ");
            System.out.println(e);
            fail("Failed to initialize " + m_subsystems.get(4).getName());
        }
        try {
            m_subsystems.add(new ShooterSubsystem());
        } catch (Exception e) {
            System.out.println();
            System.out.println("Stacktrace: ");
            System.out.println(e);
            fail("Failed to initialize " + m_subsystems.get(5).getName());
        }
        try {
            m_subsystems.add(new DrivetrainSubsystem((VisionSubsystem) m_subsystems.get(0)));
        } catch (Exception e) {
            System.out.println();
            System.out.println("Stacktrace: ");
            System.out.println(e);
            fail("Failed to initialize " + m_subsystems.get(6).getName());
        }
        
        for (int i = 0; i < m_subsystems.size(); ++i) {
            assertNotNull("Failed to initialize " + m_subsystems.get(i).getName(), m_subsystems.get(i));
        } */
    }
}
