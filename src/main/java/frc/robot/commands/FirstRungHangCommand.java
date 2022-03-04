package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.HangSubsystem;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FirstRungHangCommand extends CommandBase {
    private final HangSubsystem m_hangSubsystem;
    private double hangSpeed;

    private final DigitalInput hangRightLimitSwitch; 
    private final DigitalInput hangLeftLimitSwitch; 

    public FirstRungHangCommand(double speed) {

        m_hangSubsystem = new HangSubsystem ();
        hangSpeed = speed;
        addRequirements(m_hangSubsystem);

        hangRightLimitSwitch = new DigitalInput(4);
        hangLeftLimitSwitch = new DigitalInput(5);
    }
    // Called when the command is initially scheduled.
    @Override
    public void execute() {
        SmartDashboard.putString("limit switch right ", hangRightLimitSwitch.get() + "");
        SmartDashboard.putString("limit switch left", hangLeftLimitSwitch.get() + "");

        m_hangSubsystem.getLeftEncoder();
        m_hangSubsystem.getRightEncoder();

        if (hangRightLimitSwitch.get() && !m_hangSubsystem.triggeredRightSwitch) {
            m_hangSubsystem.triggeredRightSwitch = true;
            m_hangSubsystem.setHangRightSpeed(0);
            // if (m_hangSubsystem.getRightEncoder() != m_hangSubsystem.metersToNativeUnits(0.0762)) {
            //     m_hangSubsystem.setHangRightSpeed(hangSpeed);
            // } else m_hangSubsystem.setHangRightSpeed(0);
            m_hangSubsystem.rightResetEncoders(); 
        } else if (!m_hangSubsystem.triggeredRightSwitch) {
            m_hangSubsystem.setHangRightSpeed(hangSpeed); 
        }

        if (hangLeftLimitSwitch.get() && !m_hangSubsystem.triggeredLeftSwitch) {
            m_hangSubsystem.triggeredLeftSwitch = true;
            m_hangSubsystem.setHangLeftSpeed(0);
            // if (m_hangSubsystem.getLeftEncoder() != m_hangSubsystem.metersToNativeUnits(0.0762)) {
            //     m_hangSubsystem.setHangLeftSpeed(hangSpeed);
            // } else m_hangSubsystem.setHangLeftSpeed(0);
            m_hangSubsystem.leftResetEncoders(); 
        } else if (!m_hangSubsystem.triggeredLeftSwitch) {
            m_hangSubsystem.setHangLeftSpeed(hangSpeed);
        }
    }
}