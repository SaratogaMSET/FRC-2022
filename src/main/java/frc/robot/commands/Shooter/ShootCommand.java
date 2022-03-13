package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.ShooterSubsystem.ShooterZone;
import frc.robot.subsystems.VisionSubsystem;

public class ShootCommand extends CommandBase {
    private final ShooterSubsystem m_shooter;
    private VisionSubsystem m_vision;

    private ShooterZone m_zone;

    private boolean m_shooterAngle;
    private double m_rpm;

    /** Creates a new ShooterCommand. */
    public ShootCommand(ShooterSubsystem shooter, ShooterZone zone) {
        m_shooter = shooter;
        m_zone = zone;
        addRequirements(m_shooter);
    }

    public ShootCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        addRequirements(shooter);
        addRequirements(vision);
        m_shooter = shooter;
        m_vision = vision;
        double distance = m_vision.getDistanceFromTarget();
        m_zone = m_shooter.getShooterZone(distance);
    }

    // public void shoot(double rpm, double timeDelay, double voltageIncrease, double duration){
    //     double voltage1 = m_shooter.shooterMotor1.getBusVoltage();
    //     double voltage2 = m_shooter.shooterMotor2.getBusVoltage();
    //     m_shooter.setRPM(rpm);
    //     //TimeDelayInSeconds
    //     double timeStart = Timer.getFPGATimestamp();
        
    //     double timeStartIncrease = timeStart + timeDelay;
    //     double timeEndIncrease = timeStartIncrease + duration;

    //     if(Timer.getFPGATimestamp() >= timeStartIncrease && Timer.getFPGATimestamp() <= timeEndIncrease){
    //         m_shooter.shooterMotor1.setVoltage(feedforward.calculate(leftVelocity)); //voltage1 + voltageIncrease);
    //         m_shooter.shooterMotor2.setVoltage(voltage2 + voltageIncrease);
    //     }
    // }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_rpm = m_shooter.getShooterStateRPM(m_zone);
        m_shooterAngle = m_shooter.getShooterAngle(m_zone);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
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