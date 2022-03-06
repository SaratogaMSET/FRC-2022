package frc.robot.commands.Hang;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.HangSubsystem;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DeployHangCommand extends CommandBase {
    private final HangSubsystem m_hangSubsystem;
    private boolean m_position;

    public DeployHangCommand(HangSubsystem hang, Boolean position) {
        m_position = position;
        m_hangSubsystem = hang;
        addRequirements(m_hangSubsystem);

        m_hangSubsystem.deployHang(m_position);
    }
    // Called when the command is initially scheduled.
    @Override
    public void execute() {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}