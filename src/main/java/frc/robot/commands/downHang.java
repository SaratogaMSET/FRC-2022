package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.HangSubsystem;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class downHang extends CommandBase {
    private final HangSubsystem m_hangSubsystem;
    private double hangSpeed;

    public downHang(HangSubsystem hang, double speed) {
        hangSpeed = -speed;
        m_hangSubsystem = hang;
        addRequirements(m_hangSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void execute() {
        
        if (m_hangSubsystem.hangRightLimitSwitch.get()) {
            m_hangSubsystem.triggeredRightSwitch = true;
            m_hangSubsystem.setHangRightSpeed(0);
            m_hangSubsystem.rightResetEncoders(); 
        } else if (!m_hangSubsystem.hangRightLimitSwitch.get()) {
            m_hangSubsystem.setHangRightSpeed(hangSpeed); 
        }

        if (m_hangSubsystem.hangLeftLimitSwitch.get()) {
            m_hangSubsystem.triggeredLeftSwitch = true;
            m_hangSubsystem.setHangLeftSpeed(0);
            m_hangSubsystem.leftResetEncoders(); 
        } else if (!m_hangSubsystem.hangLeftLimitSwitch.get()) {
            m_hangSubsystem.setHangLeftSpeed(hangSpeed);
        }
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
