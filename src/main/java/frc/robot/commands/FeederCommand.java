package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj.DigitalInput;

public class FeederCommand extends CommandBase {
    private final Feeder m_feeder;
    private double topMotorVelocity, bottomMotorVelocity;
    private RobotState robotState;
    private DigitalInput[] IR_GATES;

    public FeederCommand(Feeder feederSub, double topMotorVelocity, double bottomMotorVelocity, RobotState robotState) {
        m_feeder = feederSub;
        addRequirements(feederSub);
        this.robotState = robotState;
        this.topMotorVelocity = topMotorVelocity;
        this.bottomMotorVelocity = bottomMotorVelocity;
    }

    @Override
    public void execute() {
        if (m_feeder.getIRGate(Constants.FeederConstants.BOTTOM_GATE)) {
            m_feeder.setBottomMotorVelocity(bottomMotorVelocity);
        } else {
            m_feeder.setBottomMotorVelocity(0);
        }
        if (m_feeder.getIRGate(Constants.FeederConstants.TOP_GATE)) {
            m_feeder.setTopMotorVelocity(topMotorVelocity);
        } else {
            m_feeder.setTopMotorVelocity(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_feeder.setTopMotorVelocity(0);
        m_feeder.setBottomMotorVelocity(0);
    }
}