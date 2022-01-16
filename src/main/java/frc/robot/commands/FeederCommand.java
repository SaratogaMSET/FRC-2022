package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.RobotState;

public class FeederCommand extends CommandBase {
    private final Feeder m_feeder;
    private double topMotorVelocity, bottomMotorVelocity;
    private RobotState robotState;

    public FeederCommand(Feeder feederSub, double topMotorVelocity, double bottomMotorVelocity, RobotState robotState) {
        m_feeder = feederSub;
        addRequirements(feederSub);
        this.robotState = robotState;
        this.topMotorVelocity = topMotorVelocity;
        this.bottomMotorVelocity = bottomMotorVelocity;
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }
}