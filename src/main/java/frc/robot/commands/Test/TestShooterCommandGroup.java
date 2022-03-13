package frc.robot.commands.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterZone;
import frc.robot.commands.Shooter.ShootCommand;

public class TestShooterCommandGroup extends SequentialCommandGroup {
    public TestShooterCommandGroup(ShooterSubsystem shooter, VisionSubsystem vision) {
        try {
            addCommands(
                    new ShootCommand(shooter, ShooterZone.TEST).withTimeout(2));
            SmartDashboard.putString("Shooter Motors and Solenoid", "Success");
        } catch (Exception e) {
            SmartDashboard.putString("Shooter Motors and Solenoid", "Failed");
        }
    }
}