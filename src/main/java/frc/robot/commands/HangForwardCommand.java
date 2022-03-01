package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HangSubsystem;

public class HangForwardCommand extends CommandBase {
    private final HangSubsystem m_hangSubsystem;
    private double hangSpeed;

    public HangForwardCommand(int motorType, double speed) {
        m_hangSubsystem = new HangSubsystem (motorType);
        hangSpeed = speed;
        addRequirements(m_hangSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void execute() {
        m_hangSubsystem.setHangSpeed(hangSpeed);
    }
}