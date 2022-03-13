package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualShootCommand extends CommandBase {
    private final ShooterSubsystem m_shooter;
    private double m_rpm;

    public ManualShootCommand(ShooterSubsystem shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_rpm == 0) {
            m_shooter.setRPM(m_shooter.getLastNonZeroRPM());
        } else {
            m_shooter.setRPM(m_rpm);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.setRPM(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}