package frc.robot.commands.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Hang.DeployHangCommand;
import frc.robot.commands.Hang.HangDownCommand;
import frc.robot.commands.Hang.HangUpCommand;
import frc.robot.subsystems.HangSubsystem;

public class TestHangCommandGroup extends SequentialCommandGroup {
    public TestHangCommandGroup(HangSubsystem hang, double speed) {
        try {
            addCommands(
                    new HangDownCommand(hang, speed),
                    new HangUpCommand(hang, speed),
                    new DeployHangCommand(hang, HangSubsystem.POSITION_FORWARD),
                    new DeployHangCommand(hang, HangSubsystem.POSITION_NORMAL),
                    new HangDownCommand(hang, speed));
            SmartDashboard.putString("Intake Solenoid", "Success");
        } catch (Exception e) {
            SmartDashboard.putString("Intake Solenoid", "Failed");
        }
    }
}