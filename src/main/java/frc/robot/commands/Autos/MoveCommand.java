package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class MoveCommand extends CommandBase {
    private final DrivetrainSubsystem m_dt = DrivetrainSubsystem.getInstance();

    private ArrayList<CurvePoint> m_allPoints = null;

    public MoveCommand(ArrayList<CurvePoint> allPoints, double followAngle) {
        
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        m_dt.drive(new ChassisSpeeds());
    }
}
