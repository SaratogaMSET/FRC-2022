package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ZeroGyroCommand extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;

    public ZeroGyroCommand(DrivetrainSubsystem dt) {
        this.m_drivetrainSubsystem = dt;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.zeroGyroscope();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}