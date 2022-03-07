package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeFeeder.DeployIntakeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;

public class TestIntakeCommandGroup extends SequentialCommandGroup {
    public TestIntakeCommandGroup(IntakeSubsystem intake) {
        if (intake.getIntakeState() == IntakeState.UP) {
            addCommands(
                new DeployIntakeCommand(intake, IntakeState.DOWN),
                new DeployIntakeCommand(intake, IntakeState.UP)
            );
        } else {
            addCommands(
                new DeployIntakeCommand(intake, IntakeState.UP),
                new DeployIntakeCommand(intake, IntakeState.DOWN),
                new DeployIntakeCommand(intake, IntakeState.UP)
            );
        }
    }
}