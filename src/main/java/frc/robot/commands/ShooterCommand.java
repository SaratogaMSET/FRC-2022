package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem m_shooter;
    public ShooterCommand(ShooterSubsystem shooterSub) {
        m_shooter = shooterSub;
        addRequirements(shooterSub);
    }
    @Override
    public void initialize(){
        m_shooter.run(0.65);
    }
    @Override
    public void execute() {
        
    }
    @Override
    public void end(boolean interrupted) {
        m_shooter.run(0.0);
    }
}