package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Swerve;

public class ConstantAim extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Swerve swerve;
    private final DoubleSupplier m_rawAngle;
    private final PIDController pid;

    public double pidValue = 0;

    private DoubleSupplier m_translationXSupplier;
    private DoubleSupplier m_translationYSupplier;

    private DoubleSupplier m_rot;

    private double m_translationXTrapezoidal = 0;
    private double m_translationYTrapezoidal = 0;


    // public CANCoder backRightCanCoder = new CANCoder(Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER);
    // public CANCoder backLeftCanCoder = new CANCoder(Constants.Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER);
    // public CANCoder frontRightCanCoder = new CANCoder(Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER);
    // public CANCoder frontLeftCanCoder = new CANCoder(Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER);

    public ConstantAim(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier rawAngle) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_rawAngle = rawAngle;
        this.m_translationXSupplier = x;
        this.m_translationYSupplier = y;
        this.m_rot = rot;

        pid = new PIDController(Constants.Drivetrain.kPThetaAimLock, Constants.Drivetrain.kIThetaAimLock, 0);
        this.swerve = null;
        addRequirements(m_drivetrainSubsystem);
        // addRequirements(m_visionSubsystem);
    }
    public ConstantAim(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot, Swerve drivetrainSubsystem, DoubleSupplier rawAngle) {
        this.swerve = drivetrainSubsystem;
        this.m_rawAngle = rawAngle;
        this.m_translationXSupplier = x;
        this.m_translationYSupplier = y;

        this.m_rot = rot;
        this.m_drivetrainSubsystem = null;
        pid = new PIDController(Constants.Drivetrain.kPThetaAimLock, Constants.Drivetrain.kIThetaAimLock, 0);

        addRequirements(swerve);
        // addRequirements(m_visionSubsystem);
    }



    @Override
    public void execute() {
        // m_visionSubsystem.refresh();
        if (pid.calculate(m_rawAngle.getAsDouble(), 0) > 9) {
            pidValue = 9;
        } else if (pid.calculate(m_rawAngle.getAsDouble(), 0) < -9) {
            pidValue = -9;
        } else {
            pidValue = pid.calculate(m_rawAngle.getAsDouble(), 0);
        }
        
        // VisionSubsystem.VisionState visionState = m_visionSubsystem.updateVisionState();

        // failsafe code to make sure it does not keep spinning
        // if(visionState==VisionSubsystem.VisionState.NO_TARGET){
        //     pidValue = 0;
        // }


        m_translationXTrapezoidal = (m_translationXSupplier.getAsDouble()-m_translationXTrapezoidal)/1 + m_translationXTrapezoidal;
        m_translationYTrapezoidal = (m_translationYSupplier.getAsDouble()-m_translationYTrapezoidal)/1 + m_translationYTrapezoidal;

        double magnitude = Math.hypot(m_translationXTrapezoidal, m_translationYTrapezoidal);

        double joyAngle = Math.atan2(m_translationYTrapezoidal, m_translationXTrapezoidal);

        if(this.swerve == null){

            double roboAngle = (m_drivetrainSubsystem.getNavHeading() + joyAngle);

            double resultX = Math.cos(roboAngle) * magnitude;
            double resultY = Math.sin(roboAngle) * magnitude;        

            m_drivetrainSubsystem.drive(
                new ChassisSpeeds(
                    resultX,
                    resultY,
                    m_rot.getAsDouble() + pidValue
                )
            );

        }
        else{
            double roboAngle = (swerve.getNavHeading() + joyAngle);

            double resultX = Math.cos(roboAngle) * magnitude;
            double resultY = Math.sin(roboAngle) * magnitude;        
            /* Deadbands */
            resultY = (Math.abs(resultY) < Constants.Drivetrain.stickDeadband) ? 0 : resultY;
            resultX = (Math.abs(resultX) < Constants.Drivetrain.stickDeadband) ? 0 : resultX;
            swerve.drive(
                new Translation2d(resultX,resultY),
                roboAngle,
                true, //check this
                true
            );
        }


        // SmartDashboard.putNumber("PID value", pidValue);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
