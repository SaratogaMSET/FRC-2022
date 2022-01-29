package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.CANCoder;

public class AutonomousCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    public CANCoder backRightCanCoder = new CANCoder(Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER);
    public CANCoder backLeftCanCoder = new CANCoder(Constants.Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER);
    public CANCoder frontRightCanCoder = new CANCoder(Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER);
    public CANCoder frontLeftCanCoder = new CANCoder(Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    public AutonomousCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, m_drivetrainSubsystem.getRotation2d());

        // double magnitude = Math.hypot(m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble());

        // double joyAngle = Math.atan2(m_translationYSupplier.getAsDouble(), m_translationXSupplier.getAsDouble());
        // double roboAngle = (m_drivetrainSubsystem.getNavHeading() + joyAngle);

        // double resultX = Math.cos(roboAngle) * magnitude;
        // double resultY = Math.sin(roboAngle) * magnitude;
        // double resultRotation = 3.0;


        // m_drivetrainSubsystem.drive(
        //     new ChassisSpeeds(
        //         resultX,
        //         resultY,
        //         resultRotation
        //     )
        // );

    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
