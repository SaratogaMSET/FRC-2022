package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.ShooterSubsystem.ShooterZone;
import frc.robot.subsystems.VisionSubsystem;

public class ShootCommand extends CommandBase {
    private final ShooterSubsystem m_shooter;
    private ShooterZone m_zone;
    private double m_rpm;
    private VisionSubsystem m_vision;

    public ShootCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        addRequirements(shooter);
        addRequirements(vision);
        m_shooter = shooter;
        m_vision = vision;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_zone = m_shooter.getShooterZone(m_vision.getDistance());
        m_rpm = m_shooter.getDesiredShooterRPM(m_zone);
        ShooterAngle desiredAngle = m_shooter.getDesiredShooterAngle(m_zone);

        // Set shooter angle and rpm
        m_shooter.setAngle(desiredAngle);
        m_shooter.setRPM(m_rpm);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Nothing to do here.
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.setRPM(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}