package frc.robot.commands.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.commands.Shooter.ToggleShooterAngleCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterZone;
import frc.robot.subsystems.VisionSubsystem;

public class TestShooterCommandGroup extends SequentialCommandGroup {
    public TestShooterCommandGroup(ShooterSubsystem shooter, VisionSubsystem vision) {
        try {
            addCommands(
                    new ShootCommand(shooter, ShooterZone.TEST).withTimeout(1),
                    new ToggleShooterAngleCommand(shooter),
                    new ShootCommand(shooter, ShooterZone.ZONE_1).withTimeout(1),
                    new ToggleShooterAngleCommand(shooter));
            SmartDashboard.putString("Shooter Motors and Solenoid", "Success");
        } catch (Exception e) {
            SmartDashboard.putString("Shooter Motors and Solenoid", "Failed");
        }
    }
}