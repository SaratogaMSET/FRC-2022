package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSystem;
import frc.robot.subsystems.VisionSystem.VisionState;
import edu.wpi.first.math.controller.PIDController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.CANCoder;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionSystem m_visionSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final BooleanSupplier m_turnmodeSupplier;

    private boolean maintainGyroAngle = false;
    private double gyroAngleToMaintain = 0;

    private boolean enabledVisionHoming = false;

    private double m_translationXTrapezoidal = 0;
    private double m_translationYTrapezoidal = 0;


    public CANCoder backRightCanCoder = new CANCoder(Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER);
    public CANCoder backLeftCanCoder = new CANCoder(Constants.Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER);
    public CANCoder frontRightCanCoder = new CANCoder(Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER);
    public CANCoder frontLeftCanCoder = new CANCoder(Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER);

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               VisionSystem VisionSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               BooleanSupplier turnModeBooleanSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_visionSubsystem = VisionSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_turnmodeSupplier = turnModeBooleanSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {

        m_translationXTrapezoidal = (m_translationXSupplier.getAsDouble()-m_translationXTrapezoidal)/5 + m_translationXTrapezoidal;
        m_translationYTrapezoidal = (m_translationYSupplier.getAsDouble()-m_translationYTrapezoidal)/5 + m_translationYTrapezoidal;

        double magnitude = Math.hypot(m_translationXTrapezoidal, m_translationYTrapezoidal);

        double joyAngle = Math.atan2(m_translationYTrapezoidal, m_translationXTrapezoidal);
        double roboAngle = (m_drivetrainSubsystem.getNavHeading() + joyAngle);

        double resultX = Math.cos(roboAngle) * magnitude;
        double resultY = Math.sin(roboAngle) * magnitude;

        double multiplier = Math.pow(Math.abs(m_rotationSupplier.getAsDouble()/9.1), 0.8);

        double resultW = m_rotationSupplier.getAsDouble();
        if(maintainGyroAngle && m_turnmodeSupplier.getAsBoolean()) {
            if(Math.abs(m_rotationSupplier.getAsDouble()) < 0.01){
                PIDController pid = new PIDController(Constants.Drivetrain.kPThetaController, Constants.Drivetrain.kIThetaController, 0);
                resultW = pid.calculate(m_drivetrainSubsystem.getNavHeading(), gyroAngleToMaintain);
            }else{
                gyroAngleToMaintain = m_drivetrainSubsystem.getNavHeading();
            }
        }

        if(enabledVisionHoming && m_turnmodeSupplier.getAsBoolean() && m_visionSubsystem.updateVisionState() == VisionState.TARGET_VISIBLE){
            PIDController pid = new PIDController(Constants.Drivetrain.kPThetaController, Constants.Drivetrain.kIThetaController, 0);
            double pidValue = 0;
            if (pid.calculate(m_visionSubsystem.getTx(), 0) > 9) {
                pidValue = 9;
            } else if (pid.calculate(m_visionSubsystem.getTx(), 0) < -9) {
                pidValue = -9;
            } else {
                pidValue = pid.calculate(m_visionSubsystem.getTx(), 0);
            }
            resultW = pidValue;
        }

        SmartDashboard.putNumber("m_translationXSupplier", m_translationXTrapezoidal);
        SmartDashboard.putNumber("m_translationYSupplier", m_translationYTrapezoidal);
        SmartDashboard.putNumber("m_rotationSupplier", m_rotationSupplier.getAsDouble());
        
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
                resultW * multiplier
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
