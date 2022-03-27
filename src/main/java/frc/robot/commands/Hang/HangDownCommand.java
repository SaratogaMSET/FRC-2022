package frc.robot.commands.Hang;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangSubsystem;

/**
 * This command is used to pull the hang arms down (or pull the robot up)
 */
public class HangDownCommand extends CommandBase {
    private final HangSubsystem m_hangSubsystem;
    private double hangSpeed;

    public HangDownCommand(HangSubsystem hang, double speed) {
        hangSpeed = -speed;
        m_hangSubsystem = hang;
        addRequirements(m_hangSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void execute() {
        if (m_hangSubsystem.hangRightLimitSwitch.get() || m_hangSubsystem.triggeredRightSwitch) {
            m_hangSubsystem.triggeredRightSwitch = true;
            m_hangSubsystem.setHangRightSpeed(0);
        } else if (!m_hangSubsystem.triggeredRightSwitch) {
            m_hangSubsystem.setHangRightSpeed(hangSpeed);
        }

        if (m_hangSubsystem.hangLeftLimitSwitch.get() || m_hangSubsystem.triggeredLeftSwitch) {
            m_hangSubsystem.triggeredLeftSwitch = true;
            m_hangSubsystem.setHangLeftSpeed(0);
        } else if (!m_hangSubsystem.triggeredLeftSwitch) {
            m_hangSubsystem.setHangLeftSpeed(hangSpeed);
        }

        m_hangSubsystem.maxHeightRight = false;
        m_hangSubsystem.maxHeightLeft = false;
        m_hangSubsystem.halfHeightLeft = false;
        m_hangSubsystem.halfHeightLeft = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_hangSubsystem.triggeredLeftSwitch && m_hangSubsystem.triggeredRightSwitch) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_hangSubsystem.setHangLeftSpeed(0);
        m_hangSubsystem.setHangRightSpeed(0);
    }
}