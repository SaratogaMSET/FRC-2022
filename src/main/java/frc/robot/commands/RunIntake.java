package frc.robot.commands;

import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

public class RunIntake extends CommandBase {
    private final Intake m_intake;
    private IntakeState state;
    private double speed;
    private String smart;

    public RunIntake(Intake intakeSub, IntakeState state, double speed) {
        m_intake = intakeSub;
        this.state = state;
        this.speed = speed;
        addRequirements(intakeSub);
    }

    public RunIntake(Intake intakeSub, IntakeState state) {
        m_intake = intakeSub;
        this.state = state;
        this.speed = 0;
        addRequirements(intakeSub);
    }


    public void initialize() {
        //m_intake.run(speed);
        SmartDashboard.putString("Intake Status:", smart);
        if(state == IntakeState.IDLE) {
            m_intake.stopAll();
            smart = "IDLE";
        }
        else if(state == IntakeState.FLIP_UP) {
            m_intake.deploy(false);
            smart = "UP (NO SPIN)";
        }
        else if(state == IntakeState.FLIP_DOWN) {
            m_intake.deploy(true);
            smart = "DOWN (NO SPIN)";
        }
        else { // Should never occur
            m_intake.stopAll();
            smart = "STOP (STATE NOT FOUND)";
        }
    }

    public void stopAll() {
        m_intake.stopAll();
        smart = "ALL-STOP (E-STOP)";
    }

    @Override
    public void execute() {
        /**
        * The action to take when the command ends. Called when either the command finishes normally, or
        * when it interrupted/canceled.
        **/
    }

    @Override
    public void end(boolean interrupted) { // stops the intake and them moves the intake up before stoping everything again
        m_intake.stopAll();
        m_intake.deploy(false);
        m_intake.stopAll();
    }

    // Returns true when the command should end.
    public boolean isFinished() {
        if (RobotState.intakeState == IntakeState.FLIP_UP) {
            return true;
        }
        return false;
    }
}