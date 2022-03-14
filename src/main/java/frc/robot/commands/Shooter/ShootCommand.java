package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.ShooterSubsystem.ShooterZone;
import frc.robot.subsystems.VisionSubsystem;

public class ShootCommand extends CommandBase {
    private final ShooterSubsystem m_shooter;
    private VisionSubsystem m_vision;

    private ShooterZone m_zone = ShooterZone.ZONE_1;

    private boolean m_shooterAngle;
    private double m_rpm;

    /** Creates a new ShooterCommand. */
    public ShootCommand(ShooterSubsystem shooter, ShooterZone zone) {
        m_shooter = shooter;
        m_zone = zone;
        addRequirements(m_shooter);
    }

    public ShootCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        m_shooter = shooter;
        m_vision = vision;
        addRequirements(shooter);
        addRequirements(vision);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_vision != null) {
            double distance = m_vision.getDistanceFromTarget();
            m_zone = m_shooter.getShooterZone(distance);
        }
        m_rpm = m_shooter.getShooterStateRPM(m_zone);
        m_shooterAngle = m_shooter.getShooterAngle(m_zone);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putString("Shooter Zone", m_zone.toString());
        SmartDashboard.putNumber("RPM Setpoint", m_rpm);
        SmartDashboard.putBoolean("Hood Setpoint", m_shooterAngle);
        m_shooter.setRPM(m_rpm);
        m_shooter.setAngle(m_shooterAngle);
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