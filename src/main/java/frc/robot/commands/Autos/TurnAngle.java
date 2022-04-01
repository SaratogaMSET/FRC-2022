package frc.robot.commands.Autos;

import com.fasterxml.jackson.databind.deser.SettableBeanProperty.Delegating;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
        this.m_initialAngle = m_drivetrainSubsystem.getNavHeading();

        pid = new PIDController(Constants.Drivetrain.kPThetaController+0.0225, Constants.Drivetrain.kIThetaController, 0);
        //+0.02
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double currentAngle = m_drivetrainSubsystem.getNavHeading();
        double diff = (currentAngle-m_initialAngle) * 180/Math.PI;
        pidValue = pid.calculate(diff, m_DeltaTheta);

        // SmartDashboard.putNumber("PID Value", pidValue);
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, m_DeltaTheta));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
