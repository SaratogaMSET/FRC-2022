package frc.robot.commands.IntakeFeeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;

public class RunFeederCommand extends CommandBase {
    private FeederSubsystem feeder;
    private FeederState direction;
    private double shooterFeederSpeed, intakeFeederSpeed;

    public RunFeederCommand(FeederSubsystem feeder, FeederState direction, 
            double shooterFeederSpeed, double intakeFeederSpeed) {
        this.feeder = feeder;
        this.shooterFeederSpeed = shooterFeederSpeed;
        this.intakeFeederSpeed = intakeFeederSpeed;
        this.direction = direction;
        addRequirements(feeder);
    }

    @Override
    public void execute() {
        if (direction == FeederState.IR_ASSISTED_INTAKE) {
            feeder.runIntakeIfPossible(shooterFeederSpeed, intakeFeederSpeed);
        } else if (direction == FeederState.OUTTAKE) {
            feeder.runOuttake(shooterFeederSpeed, intakeFeederSpeed);
        } else if (direction == FeederState.MANUAL_INTAKE){
            feeder.feedToShoot(shooterFeederSpeed, intakeFeederSpeed);
        } else if (direction == FeederState.IDLE) {
            feeder.stopFeeder();
        }

        SmartDashboard.putBoolean("Intake gate", feeder.intakeGate.get());
        SmartDashboard.putBoolean("Shooter gate", feeder.shooterGate.get());
        SmartDashboard.putNumber("Intake feeder speed", feeder.intakeFeederMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("Shooter feeder speed", feeder.shooterFeederMotor.getMotorOutputPercent());
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stopFeeder();
    }
}