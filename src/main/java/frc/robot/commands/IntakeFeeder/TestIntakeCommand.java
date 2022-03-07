package frc.robot.commands.IntakeFeeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;

public class TestIntakeCommand extends CommandBase {
    private IntakeSubsystem m_intake;

    public TestIntakeCommand(IntakeSubsystem intakeSubsystem) {
        m_intake = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        SmartDashboard.putString(m_intake.getIntakeState().toString(), "Intake state 1");

        // Let's get the current position of intake and reverse that position.
        if (m_intake.getIntakeState() == IntakeState.UP) {
            m_intake.updateIntakeState(IntakeState.DOWN);
        } else {
            m_intake.updateIntakeState(IntakeState.UP);
        }

        SmartDashboard.putString(m_intake.getIntakeState().toString(), "Intake state 2");

        // Let's do it again, take the current position of intake and reverse that.
        if (m_intake.getIntakeState() == IntakeState.UP) {
            m_intake.updateIntakeState(IntakeState.DOWN);
        } else {
            m_intake.updateIntakeState(IntakeState.UP);
        }

        SmartDashboard.putString(m_intake.getIntakeState().toString(), "Intake state 3");
    }

    public boolean isFinished() {
        // Since we just want the command to execute just one.
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // There is nothing to clean up.
    }    
}
