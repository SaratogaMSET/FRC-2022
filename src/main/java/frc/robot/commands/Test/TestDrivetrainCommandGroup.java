package frc.robot.commands.Test;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TestDrivetrainCommandGroup extends SequentialCommandGroup {
    public TestDrivetrainCommandGroup(DrivetrainSubsystem drivetrainSubsystem,
            double driveX, double driveY, double turnSpeed) {
        try {
            addCommands(
                    (new DefaultDriveCommand(drivetrainSubsystem, () -> (driveX * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI), () -> (driveY * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI), () -> turnSpeed))
                            .withTimeout(1));
            SmartDashboard.putString("Drivetrain", "Success");
        } catch (Exception e) {
            SmartDashboard.putString("Drivetrain", "Failed");
        }
    }
}