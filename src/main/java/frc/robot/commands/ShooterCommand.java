package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterZone;
import frc.robot.subsystems.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.VisionSubsystem;

public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem m_shooter;
    private ShooterZone target;
    private double rpm;
    private boolean isFinished;
    private VisionSubsystem m_vision;
    private ShooterAngle angleState;



    /** Creates a new ShooterCommand. */
    public ShooterCommand(ShooterSubsystem shooter, ShooterZone target) {
        this.m_shooter = shooter;
        this.target = target;
        this.rpm = m_shooter.getShooterStateRPM(target);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_shooter);
    }


    public ShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
        addRequirements(shooter);
        addRequirements(vision);
        this.m_shooter = shooter;
        this.m_vision = vision;
        this.target = m_vision.getShooterStateFromDistance();
        
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isFinished = false;
        
        this.rpm = m_shooter.getShooterStateRPM(target);
        this.angleState = m_shooter.getAngleState(target);
        m_shooter.setRPM(rpm);
        SmartDashboard.putNumber("Shooter Rpm Setpoint", rpm);


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
        if (interrupted) {
            m_shooter.setRPM(0);
        }
    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}