package frc.robot.commands.IntakeFeeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;

public class RunFeederCommand extends CommandBase {
    private final FeederSubsystem feeder;
    private FeederState feederState;
    private double shooterFeederSpeed, intakeFeederSpeed;

    public RunFeederCommand(FeederSubsystem feeder, FeederState feederState, double shooterFeederSpeed,
            double intakeFeederSpeed) {
        this.feeder = feeder;
        this.shooterFeederSpeed = shooterFeederSpeed;
        this.intakeFeederSpeed = intakeFeederSpeed;
        this.feederState = feederState;
        addRequirements(feeder);
    }

    @Override
    public void execute() {
        feeder.updateGates();
        if (feederState == FeederState.TEST) {
            feeder.diagnostics();
        } else if (feederState == FeederState.INTAKE) {
            feeder.setShooterFeeder(shooterFeederSpeed);
            feeder.setIntakeFeeder(intakeFeederSpeed);
        } else if (feederState == FeederState.OUTTAKE) {
            feeder.setShooterFeeder(-shooterFeederSpeed);
            feeder.setIntakeFeeder(-intakeFeederSpeed);
        } else {
            feeder.setShooterFeeder(0.0);
            feeder.setIntakeFeeder(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setIntakeFeeder(0.0);
        feeder.setShooterFeeder(0.0);
    }
}