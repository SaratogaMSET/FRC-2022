package frc.robot.commands.Shooter;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterZone;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.VisionSubsystem;

public class ShootCommand extends CommandBase {
    private final ShooterSubsystem m_shooter;
    private ShooterZone target;
    private double rpm;
    private VisionSubsystem m_vision;
    private ShooterAngle shooterAngle;



    /** Creates a new ShooterCommand. */
    public ShootCommand(ShooterSubsystem shooter, ShooterZone target) {
        this.m_shooter = shooter;
        this.target = target;
        this.rpm = m_shooter.getShooterStateRPM(target);
        addRequirements(m_shooter);
    }


    public ShootCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        addRequirements(shooter);
        addRequirements(vision);
        this.m_shooter = shooter;
        this.m_vision = vision;
        this.target = m_vision.getShooterStateFromDistance();
        
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
        this.rpm = m_shooter.getShooterStateRPM(target);
        this.shooterAngle = m_shooter.getAngleState(target);
        m_shooter.setRPM(rpm);

        // if (angleState == ShooterStateAngle.FOURZERO)
        //     m_shooter.deploy(true);
        // else
        //     m_shooter.deploy(false);
    }
    

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    
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