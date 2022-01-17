package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

public class RunIntake extends CommandBase {
    private final Intake m_intake;

    public RunIntake(Intake intakeSub) {
        m_intake = intakeSub;
        addRequirements(intakeSub);

        m_intake.run(0.1);
    }

    @Override
    public void execute() {
        // if() {
        //     m_intake.deploy(true);
        //     m_intake.run(0.1);
        // }
        // else {
        //     m_intake.deploy(false);
        //     m_intake.run(0.0);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.run(0.0);
        m_intake.deploy(false);
    }
}
