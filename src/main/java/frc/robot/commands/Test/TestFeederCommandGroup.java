package frc.robot.commands.Test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeFeeder.RunFeederCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;

public class TestFeederCommandGroup extends SequentialCommandGroup {
    public TestFeederCommandGroup(FeederSubsystem feeder,
            double shooterFeederSpeed, double intakeFeederSpeed) {
        try {
            if (feeder.getFeederState() == FeederState.IDLE) {
                addCommands(
                        (new RunFeederCommand(feeder, FeederState.INTAKE, shooterFeederSpeed, intakeFeederSpeed))
                                .withTimeout(2),
                        (new RunFeederCommand(feeder, FeederState.OUTTAKE, shooterFeederSpeed, intakeFeederSpeed))
                                .withTimeout(2));
            }
            SmartDashboard.putString("Intake Solenoid", "Success");
        } catch (Exception e) {
            SmartDashboard.putString("Intake Solenoid", "Failed");
        }
    }
}