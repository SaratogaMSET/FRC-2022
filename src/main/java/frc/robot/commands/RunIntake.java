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

    @Override
    public void execute() {
        if(state == IntakeState.IDLE) {
            m_intake.deploy(true);
            SmartDashboard.putBoolean("DEPLOY", true);
        }
        else if(state == IntakeState.DOWN) {
            m_intake.deploy(true);
            m_intake.run(0.1);
            SmartDashboard.putBoolean("DOWN", true);
        }
        else if(state == IntakeState.UP) {
            m_intake.deploy(false);
            m_intake.run(0.0);
            SmartDashboard.putBoolean("UP", true);
        }
        else {
            m_intake.run(0.0);
            SmartDashboard.putBoolean("ALL-STOP", true);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.run(0.0);
        m_intake.deploy(false);
    }
}
