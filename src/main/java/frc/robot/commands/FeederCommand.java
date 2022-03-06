package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;

public class FeederCommand extends CommandBase {
    private final FeederSubsystem feeder;
    private FeederState feederState;
    private double shooterFeederSpeed, intakeFeederSpeed;

    // public FeederCommand(FeederSubsystem feeder, FeederState feederState, double shooterFeederSpeed,
    //         double intakeFeederSpeed) {
    //     this.feeder = feeder;
    //     this.shooterFeederSpeed = shooterFeederSpeed;
    //     this.intakeFeederSpeed = intakeFeederSpeed;
    //     this.feederState = feederState;
    //     addRequirements(feeder);
    // }

    public FeederCommand(FeederSubsystem feeder) {
    this.feeder = feeder;
    addRequirements(feeder);
}

    @Override
    public void execute() {
        // feeder.setShooterFeeder(0.3);
        feeder.setFeederSpeed();
        // feeder.updateGates();
        // if (feederState == FeederState.TEST) {
        //     feeder.diagnostics();
        // } else if (feederState == FeederState.INTAKE) {
        //     feeder.setShooterFeeder(shooterFeederSpeed);
        //     feeder.setIntakeFeeder(intakeFeederSpeed);
        // } else if (feederState == FeederState.OUTTAKE) {
        //     feeder.setShooterFeeder(-shooterFeederSpeed);
        //     feeder.setIntakeFeeder(-intakeFeederSpeed);
        // } else {
        //     feeder.setShooterFeeder(0.0);
        //     feeder.setIntakeFeeder(0.0);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.setIntakeFeeder(0.0);
        feeder.setShooterFeeder(0.0);
    }
}