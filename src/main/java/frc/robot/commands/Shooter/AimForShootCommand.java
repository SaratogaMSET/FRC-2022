package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimForShootCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private final PIDController pid;
    public double pidValue = 0;


    // public CANCoder backRightCanCoder = new CANCoder(Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER);
    // public CANCoder backLeftCanCoder = new CANCoder(Constants.Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER);
    // public CANCoder frontRightCanCoder = new CANCoder(Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER);
    // public CANCoder frontLeftCanCoder = new CANCoder(Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER);

    public AimForShootCommand(DrivetrainSubsystem drivetrainSubsystem, double angle) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_visionSubsystem = null;

        pid = new PIDController(Constants.Drivetrain.kPThetaController, Constants.Drivetrain.kIThetaController, 0);

        addRequirements(m_drivetrainSubsystem);
    }

    public AimForShootCommand(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_visionSubsystem = visionSystem;

        pid = new PIDController(Constants.Drivetrain.kPThetaController, Constants.Drivetrain.kIThetaController, 0);

        addRequirements(m_drivetrainSubsystem);
        addRequirements(m_visionSubsystem);
    }




    @Override
    public void execute() {
        // m_visionSubsystem.refresh();
        if (pid.calculate(m_visionSubsystem.getRawAngle(), 0) > 9) {
            pidValue = 9;
        } else if (pid.calculate(m_visionSubsystem.getRawAngle(), 0) < -9) {
            pidValue = -9;
        } else {
            pidValue = pid.calculate(m_visionSubsystem.getRawAngle(), 0);
        }
        
        VisionSubsystem.VisionState visionState = m_visionSubsystem.updateVisionState();

        // failsafe code to make sure it does not keep spinning
        if(visionState==VisionSubsystem.VisionState.NO_TARGET){
            pidValue = 0;
        }

        m_drivetrainSubsystem.drive(
            new ChassisSpeeds(
                0,
                0,
                pidValue
            )
        );

        // SmartDashboard.putNumber("PID value", pidValue);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(0 - m_visionSubsystem.getRawAngle()) < 3) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
