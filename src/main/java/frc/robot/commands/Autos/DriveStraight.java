package frc.robot.commands.Autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveStraight extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private double m_velocityX;
    private double angle;

    public double pidValue = 0;

    public DriveStraight(DrivetrainSubsystem drivetrainSubsystem, double velocityX, double angle) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_velocityX = velocityX;
        this.angle = angle;
        //+0.02
    }

    @Override
    public void execute() {
      
        m_drivetrainSubsystem.m_backRightModule.set(m_velocityX, angle);
        m_drivetrainSubsystem.m_backLeftModule.set(m_velocityX, angle);
        m_drivetrainSubsystem.m_frontRightModule.set(m_velocityX, angle);
        m_drivetrainSubsystem.m_frontLeftModule.set(m_velocityX, angle);
        // SmartDashboard.putNumber("PID Value", pidValue);

    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
