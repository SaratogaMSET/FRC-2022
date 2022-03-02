package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;

public class DeployIntakeCommand extends CommandBase {
    private final IntakeSubsystem m_intake;
    private IntakeState state;

    public DeployIntakeCommand(IntakeSubsystem intakeSub, IntakeState state) {
        m_intake = intakeSub;
        this.state = state;
        addRequirements(intakeSub);
    }

    public void initialize() {
        if (state == IntakeState.TEST)
            m_intake.diagnostics();
        else if (state == IntakeState.DOWN)
            m_intake.deploy(true);
        else
            m_intake.deploy(false);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) { // stops the intake and them moves the intake up before stoping everything

    }

    // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}