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
    private boolean enableCompressor;

    public RunIntake(Intake intakeSub, IntakeState state, double speed, boolean enableCompressor) {
        m_intake = intakeSub;
        this.state = state;
        this.speed = speed;
        this.enableCompressor = enableCompressor;
        addRequirements(intakeSub);
    }

    public RunIntake(Intake intakeSub, IntakeState state) {
        m_intake = intakeSub;
        this.state = state;
        this.speed = 0;

        addRequirements(intakeSub);
    }


    public void initialize() {
        m_intake.run(speed);
        m_intake.getPSI();
        if(state == IntakeState.IDLE) {
            m_intake.stopAll();
            SmartDashboard.putBoolean("IDLE", true);
        }
        else if(state == IntakeState.INTAKE) {
            m_intake.deploy(true);
            m_intake.run(speed);
            SmartDashboard.putBoolean("DOWN, INTAKING", true);
        }
        else if(state == IntakeState.OUTTAKE) {
            m_intake.deploy(true);
            m_intake.run(-speed);
            SmartDashboard.putBoolean("DOWN, OUTTAKING", true);
        }
        else if(state == IntakeState.FLIP_UP) {
            m_intake.deploy(false);
            m_intake.run(0);
            SmartDashboard.putBoolean("UP (NO SPIN)", true);
        }
        else if(state == IntakeState.FLIP_DOWN) {
            m_intake.enableCompressor(true);
            m_intake.deploy(true);
            m_intake.run(0);
            SmartDashboard.putBoolean("DOWN (NO SPIN)", true);
        }
        else if(enableCompressor){
            m_intake.enableCompressor(true);
        }
        // else if (state == IntakeState.INTAKE) {
        //     m_intake.run(speed);
        //     SmartDashboard.putBoolean("INTAKING", true);
        // }

        // else if (state == IntakeState.OUTTAKE) {
        //     m_intake.run(speed);
        //     SmartDashboard.putBoolean("OUTTAKING", true);
        // }
        else { // Should never occur
            m_intake.stopAll();
            SmartDashboard.putBoolean("STOP (STATE NOT FOUND)", true);
        }
        
    }

    public void stopAll() {
        m_intake.stopAll();
        SmartDashboard.putBoolean("ALL-STOP (E-STOP)", true);
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