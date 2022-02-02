package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.CANCoder;

public class RotateDegrees extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final double m_setpoint;
    private final PIDController pid;


    public CANCoder backRightCanCoder = new CANCoder(Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER);
    public CANCoder backLeftCanCoder = new CANCoder(Constants.Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER);
    public CANCoder frontRightCanCoder = new CANCoder(Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER);
    public CANCoder frontLeftCanCoder = new CANCoder(Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER);

    public RotateDegrees(DrivetrainSubsystem drivetrainSubsystem, double setpoint) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_setpoint = setpoint;

        pid = new PIDController(Constants.Drivetrain.kPThetaController, Constants.Drivetrain.kIThetaController, 0);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        

        double pidValue = 0;
        if (pid.calculate(m_drivetrainSubsystem.getPose().getRotation().getDegrees(), m_setpoint) > 9) {
            pidValue = 9;
        } else if (pid.calculate(m_drivetrainSubsystem.getPose().getRotation().getDegrees(), m_setpoint) < -9) {
            pidValue = -9;
        } else {
            pidValue = pid.calculate(m_drivetrainSubsystem.getPose().getRotation().getDegrees(), m_setpoint);
        }
        
        m_drivetrainSubsystem.drive(
            new ChassisSpeeds(
                0,
                0,
                -pidValue
            )
        );

        SmartDashboard.putNumber("PID value", pidValue);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(m_setpoint - m_drivetrainSubsystem.getPose().getRotation().getDegrees()) < 5) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}