package frc.robot.commands.Hang;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HangSubsystem;

/**
 * This command is used to move the hang arms up (to reach to a rung and latch on to it)
 */
public class HalfHangUpCommand extends CommandBase {
    private final HangSubsystem m_hangSubsystem;
    private double hangSpeed;

    public HalfHangUpCommand(HangSubsystem hang, double speed) {
        hangSpeed = speed;
        m_hangSubsystem = hang;
        addRequirements(m_hangSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void execute() {

        //right
        if (m_hangSubsystem.getRightEncoderValue() > Constants.HangConstants.HANG_HALF_ENCODER_COUNTS) {
            m_hangSubsystem.halfHeightRight = true;
            m_hangSubsystem.setHangRightSpeed(0); //not necessary
        } else {
            m_hangSubsystem.setHangRightSpeed(hangSpeed); 
        }

        //left
        if (m_hangSubsystem.getLeftEncoderValue() > Constants.HangConstants.HANG_HALF_ENCODER_COUNTS) {
            m_hangSubsystem.halfHeightLeft = true;
            m_hangSubsystem.setHangLeftSpeed(0); //not necessary
        } else {
            m_hangSubsystem.setHangLeftSpeed(hangSpeed);
        }

        m_hangSubsystem.triggeredRightSwitch = false;
        m_hangSubsystem.triggeredLeftSwitch = false;
        m_hangSubsystem.triggeredRightSoftStop = false;
        m_hangSubsystem.triggeredLeftSoftStop = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_hangSubsystem.halfHeightRight && m_hangSubsystem.halfHeightLeft) {
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