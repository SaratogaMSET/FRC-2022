package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private double m_translationXTrapezoidal = 0;
    private double m_translationYTrapezoidal = 0;


    public CANCoder backRightCanCoder = new CANCoder(Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER);
    public CANCoder backLeftCanCoder = new CANCoder(Constants.Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER);
    public CANCoder frontRightCanCoder = new CANCoder(Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER);
    public CANCoder frontLeftCanCoder = new CANCoder(Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER);

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {

        m_translationXTrapezoidal = (m_translationXSupplier.getAsDouble()-m_translationXTrapezoidal)/1 + m_translationXTrapezoidal;
        m_translationYTrapezoidal = (m_translationYSupplier.getAsDouble()-m_translationYTrapezoidal)/1 + m_translationYTrapezoidal;

        double magnitude = Math.hypot(m_translationXTrapezoidal, m_translationYTrapezoidal);

        double joyAngle = Math.atan2(m_translationYTrapezoidal, m_translationXTrapezoidal);
        double roboAngle = (m_drivetrainSubsystem.getNavHeading() + joyAngle);

        double resultX = Math.cos(roboAngle) * magnitude;
        double resultY = Math.sin(roboAngle) * magnitude;

        // double multiplier = Math.pow(Math.abs(m_rotationSupplier.getAsDouble()/9.1), 0.8);
        double rotation = m_rotationSupplier.getAsDouble();
        SmartDashboard.putNumber("m_translationXSupplier", resultX);
        SmartDashboard.putNumber("m_translationYSupplier", resultY);
        SmartDashboard.putNumber("m_rotationSupplier", m_rotationSupplier.getAsDouble());
        
        if(Math.abs(resultX) < 0.18){
            resultX = 0;
        }
        if(Math.abs(rotation) < 0.6){
            rotation = 0;
        }
        // m_drivetrainSubsystem.drive(
        //     new ChassisSpeeds(
        //         m_translationXSupplier.getAsDouble(),
        //         m_translationYSupplier.getAsDouble(),
        //         m_rotationSupplier.getAsDouble() * multiplier
        //     )
        // );

        m_drivetrainSubsystem.drive(
            new ChassisSpeeds(
                resultX,
                resultY,
                rotation
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
