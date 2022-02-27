package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder.FeederState;
import edu.wpi.first.wpilibj.DigitalInput;

public class FeederCommand extends CommandBase {
    private final Feeder m_feeder;
    private double topMotorVelocity, bottomMotorVelocity;

    public FeederCommand(Feeder feederSub, FeederState feederState, double topMotorVelocity, double bottomMotorVelocity) {
        m_feeder = feederSub;
        addRequirements(feederSub);
        this.topMotorVelocity = topMotorVelocity;
        this.bottomMotorVelocity = bottomMotorVelocity;
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.setTopMotor(0);
        m_feeder.setBottomMotor(0);
    }
}