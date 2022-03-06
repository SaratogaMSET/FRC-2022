package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
// import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem m_shooter;
    private final FeederSubsystem m_feeder;


    public ShooterCommand(FeederSubsystem feeder, ShooterSubsystem shooterSub) {
        m_feeder = feeder;
        m_shooter = shooterSub;
        addRequirements(shooterSub);
        addRequirements(m_feeder);
    }
    @Override
    public void initialize(){
        m_shooter.run(0.65);
        m_feeder.setRawShooterFeeder(.3);
    }
    @Override
    public void execute() {
        
    }
    @Override
    public void end(boolean interrupted) {
        m_shooter.run(0.0);
    }
}