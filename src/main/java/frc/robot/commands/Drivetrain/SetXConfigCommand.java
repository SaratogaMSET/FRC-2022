package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SetXConfigCommand extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;

    public SetXConfigCommand(DrivetrainSubsystem dt) {
        this.m_drivetrainSubsystem = dt;
        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.m_backRightModule.set(0, Math.PI * 1/4);
        m_drivetrainSubsystem.m_backLeftModule.set(0, Math.PI * 3/4);

        m_drivetrainSubsystem.m_frontRightModule.set(0, Math.PI * 3/4);
        m_drivetrainSubsystem.m_frontLeftModule.set(0, Math.PI * 1/4);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}