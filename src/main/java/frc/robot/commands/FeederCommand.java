package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Feeder.FeederState;


public class FeederCommand extends CommandBase {
    private final Feeder feeder;
    private Feeder.FeederState feederState;
    private Feeder.FeederState test;
    private double bottomVelocity;
    private double topVelocity;

    public FeederCommand(Feeder feeder, Feeder.FeederState test, double topVelocity, double bottomVelocity){
        this.feeder = feeder;
        this.test = test;
        this.bottomVelocity = bottomVelocity;
        this.topVelocity = topVelocity;
        feederState = FeederState.RUN;
        addRequirements(feeder);
    }

    @Override
    public void execute() {
       if(test == FeederState.TEST){
            feeder.diagnostics();
       }
       else {
           feeder.setBottomMotor(bottomVelocity);
           feeder.setTopMotor(topVelocity);
    }
}

    @Override
    public void end(boolean interrupted) {
        feeder.setBottomMotor(0.0);
        feeder.setTopMotor(0.0);
    }
}
