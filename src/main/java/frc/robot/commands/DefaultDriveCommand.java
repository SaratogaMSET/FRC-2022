package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.CANCoder;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

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
        double magnitude = Math.hypot(m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble());
        // double roboAngle = (Math.atan2(m_translationYSupplier.getAsDouble(), m_translationXSupplier.getAsDouble()));
        double joyAngle = Math.atan2(m_translationYSupplier.getAsDouble(), m_translationXSupplier.getAsDouble());
        double roboAngle = (m_drivetrainSubsystem.getNavHeading() + joyAngle - m_drivetrainSubsystem.getGyroOffset());
        // roboAngle -= m_drivetrainSubsystem.getNavHeading();
        double resultX = Math.cos(roboAngle) * magnitude;
        double resultY = Math.sin(roboAngle) * magnitude;

        

        // double resultX = m_translationXSupplier.getAsDouble() * Math.cos(roboAngle) + m_translationYSupplier.getAsDouble() * Math.sin(roboAngle);
        // double resultY = -m_translationXSupplier.getAsDouble() * Math.sin(roboAngle) + m_translationYSupplier.getAsDouble() * Math.cos(roboAngle);

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        
        m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                    resultX,
                    resultY,
                    // m_translationXSupplier.getAsDouble(),
                    // m_translationYSupplier.getAsDouble(),
                    m_rotationSupplier.getAsDouble()
                )
        );
        
        // m_drivetrainSubsystem.drive(
        //         ChassisSpeeds.fromFieldRelativeSpeeds(
        //             // resultX,
        //             // resultY,
        //             m_translationXSupplier.getAsDouble(),
        //             m_translationYSupplier.getAsDouble(),
        //             m_rotationSupplier.getAsDouble(),
        //             // Rotation2d.fromDegrees(Math.abs(Math.toDegrees(initialAngle)))
        //             m_drivetrainSubsystem.getGyroscopeRotation()
        //         )
        // );

        // SmartDashboard.putNumber("heading", Math.toDegrees(m_drivetrainSubsystem.getNavHeading()));
        SmartDashboard.putNumber("joystickAngle", (Math.toDegrees(joyAngle)));

        SmartDashboard.putNumber("XSupplier", m_translationXSupplier.getAsDouble());
        SmartDashboard.putNumber("YSupplier", m_translationYSupplier.getAsDouble());

        // SmartDashboard.putNumber("backRightCanCoder", backRightCanCoder.getPosition());
        // SmartDashboard.putNumber("backLeftCanCoder", backLeftCanCoder.getPosition());
        // SmartDashboard.putNumber("frontRightCanCoder", frontRightCanCoder.getPosition());
        // SmartDashboard.putNumber("frontLeftCanCoder", frontLeftCanCoder.getPosition());

        
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
