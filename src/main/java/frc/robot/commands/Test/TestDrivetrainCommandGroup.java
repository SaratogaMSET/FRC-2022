package frc.robot.commands.Test;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Drivetrain.DefaultDriveCommand;
import frc.robot.commands.IntakeFeeder.RunFeederCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;

public class TestDrivetrainCommandGroup extends SequentialCommandGroup {
    public TestDrivetrainCommandGroup(DrivetrainSubsystem drivetrainSubsystem,
            double driveX, double driveY, double turnSpeed) {
        try {
            addCommands(
                    (new DefaultDriveCommand(drivetrainSubsystem, () -> driveX, () -> driveY, () -> turnSpeed))
                            .withTimeout(2));
            SmartDashboard.putString("Drivetrain", "Success");
        } catch (Exception e) {
            SmartDashboard.putString("Drivetrain", "Failed");
        }
    }
}