package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoRunCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private double m_velocity;
    private double m_initialAngle;
    private final PIDController pid;
    public double pidValue = 0;

    public AutoRunCommand(DrivetrainSubsystem drivetrainSubsystem, double velocity) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_velocity = velocity;
        this.m_initialAngle = m_drivetrainSubsystem.getNavHeading();

        pid = new PIDController(Constants.Drivetrain.kPThetaController+.02, Constants.Drivetrain.kIThetaController, 0);
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double currentAngle = m_drivetrainSubsystem.getNavHeading();
        double diff = m_initialAngle-currentAngle;
        pidValue = pid.calculate(diff, 0);

        SmartDashboard.putNumber("PID Valuee", pidValue);
        m_drivetrainSubsystem.drive(new ChassisSpeeds(m_velocity, 0, pidValue));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
