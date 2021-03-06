package frc.robot.commands.Test;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterZone;
import frc.robot.commands.Shooter.ShootCommand;

public class TestShooterCommandGroup extends SequentialCommandGroup {
    public TestShooterCommandGroup(ShooterSubsystem shooter, ShooterZone zone, Compressor compressor) {
        try {
            addCommands(
                    new ShootCommand(shooter, zone, compressor).withTimeout(1));
            SmartDashboard.putString("Shooter Motors and Solenoid", "Success");
        } catch (Exception e) {
            SmartDashboard.putString("Shooter Motors and Solenoid", "Failed");
        }
    }
}