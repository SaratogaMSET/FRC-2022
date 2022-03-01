package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeCommand extends CommandBase {
    private final Intake m_intake;
    private IntakeState state;

    public IntakeCommand(Intake intakeSub, IntakeState state) {
        m_intake = intakeSub;
        this.state = state;
        addRequirements(intakeSub);
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        if (state == IntakeState.TEST)
            m_intake.diagnostics();
        else if (state == IntakeState.DOWN)
            m_intake.deploy(true);
        else
            m_intake.deploy(false);
    }

    @Override
    public void end(boolean interrupted) { // stops the intake and them moves the intake up before stoping everything
                                           // again
        m_intake.deploy(false);
    }

    // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}