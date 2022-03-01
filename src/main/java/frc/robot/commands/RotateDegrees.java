package frc.robot.commands;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RotateDegrees extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private final PIDController pid;


    public CANCoder backRightCanCoder = new CANCoder(Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER);
    public CANCoder backLeftCanCoder = new CANCoder(Constants.Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER);
    public CANCoder frontRightCanCoder = new CANCoder(Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER);
    public CANCoder frontLeftCanCoder = new CANCoder(Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER);

    public RotateDegrees(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_visionSubsystem = visionSystem;

        pid = new PIDController(Constants.Drivetrain.kPThetaController, Constants.Drivetrain.kIThetaController, 0);

        addRequirements(m_drivetrainSubsystem);
        addRequirements(m_visionSubsystem);
    }

    @Override
    public void execute() {
        

        double pidValue = 0;
        if (pid.calculate(m_visionSubsystem.getRawAngle(), 0) > 9) {
            pidValue = 9;
        } else if (pid.calculate(m_visionSubsystem.getRawAngle(), 0) < -9) {
            pidValue = -9;
        } else {
            pidValue = pid.calculate(m_visionSubsystem.getRawAngle(), 0);
        }
        
        m_drivetrainSubsystem.drive(
            new ChassisSpeeds(
                0,
                0,
                pidValue
            )
        );

        SmartDashboard.putNumber("PID value", pidValue);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(0 - m_visionSubsystem.getRawAngle()) < 5) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
