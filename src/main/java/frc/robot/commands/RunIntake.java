package frc.robot.commands;

import frc.robot.RobotState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

public class RunIntake extends CommandBase {
    private final Intake m_intake;
    private IntakeState state;

    public RunIntake(Intake intakeSub) {
        m_intake = intakeSub;
        addRequirements(intakeSub);

        m_intake.run(0.1);
    }

    public void run() {
        if(state == IntakeState.IDLE) {
            m_intake.deploy(true);
            SmartDashboard.putBoolean("DEPLOY", true);
        }
        else if(state == IntakeState.DOWN) {
            m_intake.deploy(true);
            m_intake.run(0.0);
            SmartDashboard.putBoolean("DOWN", true);
        }
        else if(state == IntakeState.UP) {
            m_intake.deploy(false);
            m_intake.run(0.0);
            SmartDashboard.putBoolean("UP", true);
        }
        else if (state == IntakeState.INTAKE) {
            m_intake.run(0.1);
            SmartDashboard.putBoolean("INTAKING", true);
        }
        else if (state == IntakeState.OUTTAKE) {
            m_intake.run(-0.1);
            SmartDashboard.putBoolean("OUTTAKING", true);
        }
        else { // IntakeState must be IDLE
            m_intake.stopAll();
            SmartDashboard.putBoolean("STOP", true);
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
        */
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.run(0.0);
        m_intake.stopAll();
    }
}
