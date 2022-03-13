package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ToggleShooterAngleCommand extends CommandBase {
    private final ShooterSubsystem m_shooter;

    public ToggleShooterAngleCommand(ShooterSubsystem shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_shooter.toggleShooterAngle();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}