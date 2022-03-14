package frc.robot.commands.Hang;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotoelectricSystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This command is used to move the hang arms up (to reach to a rung and latch on to it)
 */
public class HangAutoAlign extends CommandBase {
    private final PhotoelectricSystem m_photoSubsystem;
    private final DrivetrainSubsystem m_driveSubsystem;
    private boolean done = false;

    public HangAutoAlign(PhotoelectricSystem photo, DrivetrainSubsystem drive) {
        m_photoSubsystem = photo;
        m_driveSubsystem = drive;

        addRequirements(m_photoSubsystem);
        addRequirements(m_driveSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void execute() {
        PhotoelectricSystem.PhotoelectricState state_left = m_photoSubsystem.updatePhotoStateLeft();
        PhotoelectricSystem.PhotoelectricState state_right = m_photoSubsystem.updatePhotoStateRight();
        if(state_left== PhotoelectricSystem.PhotoelectricState.NOT_LINE && state_right== PhotoelectricSystem.PhotoelectricState.NOT_LINE){
            m_driveSubsystem.drive(new ChassisSpeeds(1, 0, 0));
        }
        else if(state_left== PhotoelectricSystem.PhotoelectricState.LINE && state_right== PhotoelectricSystem.PhotoelectricState.LINE){
            done = true;
        }
        else if(state_left== PhotoelectricSystem.PhotoelectricState.LINE && state_right== PhotoelectricSystem.PhotoelectricState.NOT_LINE){
            m_driveSubsystem.drive(new ChassisSpeeds(0.5, 0, 0.75)); // counterclockwise
        }
        else{
            m_driveSubsystem.drive(new ChassisSpeeds(0.5, 0, -0.75)); // clockwise
        }
      
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }


    @Override
    public void end(boolean interrupted) {
        
    }
}