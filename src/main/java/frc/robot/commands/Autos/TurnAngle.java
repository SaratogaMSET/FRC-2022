package frc.robot.commands.Autos;

import com.fasterxml.jackson.databind.deser.SettableBeanProperty.Delegating;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TurnAngle extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private double m_DeltaTheta;
    private double m_initialAngle;
    private final PIDController pid;
    public double pidValue = 0;

    public TurnAngle(DrivetrainSubsystem drivetrainSubsystem, double deltaTheta) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;

        this.m_DeltaTheta = deltaTheta;
        

        pid = new PIDController(Constants.Drivetrain.kPThetaController+0.0225, Constants.Drivetrain.kIThetaController, 0);
        //+0.02
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        this.m_initialAngle = m_drivetrainSubsystem.getNavHeading();
    }

    @Override
    public void execute() {
        double currentAngle = m_drivetrainSubsystem.getNavHeading();
        double diff = (currentAngle-m_initialAngle);
        pidValue = pid.calculate(diff, m_DeltaTheta * Math.PI/180) * 30;
        
        SmartDashboard.putNumber("Diff", diff * 180/Math.PI);
        SmartDashboard.putNumber("currentAngle", currentAngle * 180/Math.PI);
        SmartDashboard.putNumber("initialAngle", m_initialAngle * 180/Math.PI);
        SmartDashboard.putNumber("PID Setpoint", m_DeltaTheta);
        SmartDashboard.putNumber("PID Value", pidValue);


        m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, pidValue));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(pidValue) < 0.5){
            return true;
        }
        return false;
    }
}
