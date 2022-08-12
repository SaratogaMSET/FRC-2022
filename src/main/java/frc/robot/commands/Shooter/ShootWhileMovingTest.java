package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Vision.Distance;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterZone;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.RobotContainer;

public class ShootWhileMovingTest extends CommandBase {
    private final ShooterSubsystem m_shooter;
    private Compressor m_compressor;
    private VisionSubsystem m_vision = null;

    private ShooterZone m_zone = ShooterZone.ZONE_1;
    private double distance;
    private boolean m_shooterAngle;
    private double m_rpm;

    /** Creates a new ShooterCommand. */
    // public ShootWhileMovingTest(ShooterSubsystem shooter, ShooterZone zone, Compressor compressor) {
    //     m_shooter = shooter;
    //     m_compressor = compressor;
    //     m_zone = zone;
    //     addRequirements(m_shooter);
    // }
    public ShootWhileMovingTest(ShooterSubsystem shooter, double distance, Compressor compressor) {
        m_shooter = shooter;
        m_compressor = compressor;
        this.distance = distance;
        addRequirements(m_shooter);
    }
    public ShootWhileMovingTest(ShooterSubsystem shooter, VisionSubsystem vision, Compressor compressor) {
        m_shooter = shooter;
        m_compressor = compressor;
        m_vision = vision;
        addRequirements(shooter);
        addRequirements(vision);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("RPM Setpoint", m_rpm);
        // SmartDashboard.putBoolean("Hood Setpoint", m_shooterAngle);

        m_zone = m_shooter.getShooterZone(distance); //need to figure out how to continously update distance
        m_rpm = m_shooter.getShooterStateRPM(m_zone, distance);
        if (m_vision != null) {
            double distance = m_vision.getDistanceFromTarget();
            m_zone = m_shooter.getShooterZone(distance);
            m_rpm = m_shooter.getShooterStateRPM(m_zone, distance);
        }
        // m_rpm = m_shooter.getShooterStateRPM(m_zone, distance);
        m_shooterAngle = m_shooter.getShooterAngle(m_zone);

        m_shooter.setAngle(m_shooterAngle);

        m_compressor.disable();
        m_shooter.setRPM(m_rpm);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooter.setRPM(0);
        m_shooter.setAngle(true);
        m_compressor.enableDigital();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}