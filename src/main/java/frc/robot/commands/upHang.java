package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HangSubsystem;

/**
 * This command is used to move the hang arms up (to reach to a rung and latch on to it)
 */
public class upHang extends CommandBase {
    private final HangSubsystem m_hangSubsystem;
    private double hangSpeed;

    private double encoderValue;
    private boolean heightLeft;
    private boolean heightRight;


    public upHang(HangSubsystem hang, double speed, double value) {
        hangSpeed = speed;
        m_hangSubsystem = hang;

        encoderValue = value; 
        addRequirements(m_hangSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void execute() {

        //right
        if (m_hangSubsystem.getRightEncoderValue() >= encoderValue) {
            heightRight = true;
            m_hangSubsystem.setHangRightSpeed(0); //not necessary
        } else {
            m_hangSubsystem.setHangRightSpeed(hangSpeed); 
        }

        //left
        if (m_hangSubsystem.getLeftEncoderValue() >= encoderValue) {
            heightLeft = true;
            m_hangSubsystem.setHangLeftSpeed(0); //not necessary
        } else {
            m_hangSubsystem.setHangLeftSpeed(hangSpeed);
        }

        m_hangSubsystem.triggeredRightSwitch = false;
        m_hangSubsystem.triggeredLeftSwitch = false;
        
        heightRight = false;
        heightLeft = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (heightLeft && heightRight);
    }

    @Override
    public void end(boolean interrupted) {
        m_hangSubsystem.setHangLeftSpeed(0);
        m_hangSubsystem.setHangRightSpeed(0);
    }
}