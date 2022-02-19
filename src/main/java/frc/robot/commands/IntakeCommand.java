package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

public class IntakeCommand extends CommandBase {
    private final Intake m_intake;
    private IntakeState state;
    private double speed;

    public IntakeCommand(Intake intakeSub, IntakeState state, double speed) {
        m_intake = intakeSub;
        this.state = state;
        this.speed = speed;
        addRequirements(intakeSub);
    }

    public void initialize() {}

    public void stopAll() {
        m_intake.stopAll();
    }

    @Override
    public void execute() {
        if(state == IntakeState.TEST){
            m_intake.diagnostics();
       }
        else {
           m_intake.deploy(true);
        }
    }

    @Override
    public void end(boolean interrupted) { // stops the intake and them moves the intake up before stoping everything again
        m_intake.deploy(false);
        m_intake.stopAll();
    }

    // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}