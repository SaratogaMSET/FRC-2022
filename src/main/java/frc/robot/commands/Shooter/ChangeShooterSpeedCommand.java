package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ChangeShooterSpeedCommand extends CommandBase {
    private static final double INCREMENT = 0.01;
    
    public static final boolean INCREASE = true;
    public static final boolean DECREASE = false;

    private ShooterSubsystem m_shooter;
    private boolean m_increase;

    public ChangeShooterSpeedCommand(ShooterSubsystem shooter, boolean increase) {
        m_shooter = shooter;
        m_increase = increase;
        addRequirements(m_shooter);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double newRPM = m_shooter.getLastNonZeroRPM();
        if (m_increase) {
            newRPM += INCREMENT;
        } else {
            newRPM -= INCREMENT;
        }
        m_shooter.setRPM(newRPM);

        SmartDashboard.putNumber("RPM manual setpoint", newRPM);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}