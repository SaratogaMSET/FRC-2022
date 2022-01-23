package frc.robot.commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.DoubleSupplier;
public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem m_shooter;
    public ShooterCommand(ShooterSubsystem shooterSub) {
        m_shooter = shooterSub;
        addRequirements(shooterSub);
    }
    @Override
    public void initialize(){
        m_shooter.run(0.3);
    }
    @Override
    public void execute() {
        
    }
    @Override
    public void end(boolean interrupted) {
        m_shooter.run(0.0);
    }
}