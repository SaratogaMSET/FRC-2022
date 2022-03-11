package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoRunCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private double m_velocity;
    private final PIDController pid;
    public double pidValue = 0;

    public AutoRunCommand(DrivetrainSubsystem drivetrainSubsystem, double velocity) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_velocity = velocity;

        pid = new PIDController(Constants.Drivetrain.kPThetaController, Constants.Drivetrain.kIThetaController, 0);
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(m_velocity, 0, 0));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
