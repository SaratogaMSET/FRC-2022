package frc.robot.commands.Hang;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangSubsystem;

public class DeployHangCommand extends CommandBase {
    
    private final HangSubsystem m_hangSubsystem;
    private boolean m_position;

    public DeployHangCommand(HangSubsystem hang, boolean position) {
        m_position = position;
        m_hangSubsystem = hang;
        addRequirements(m_hangSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void execute() {
        m_hangSubsystem.deployHang(m_position);

        SmartDashboard.putBoolean("Hang deployed ", m_hangSubsystem.isHangDeployed());
    }

    @Override
    public boolean isFinished() {
        // This is one time only command, so we return true after execute is complete.
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing to clean up.
    }
}