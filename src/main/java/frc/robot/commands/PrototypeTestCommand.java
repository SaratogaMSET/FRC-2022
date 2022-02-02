package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PrototypeTesting;

public class PrototypeTestCommand extends CommandBase {
    private final Joystick joystick34;
    private final Joystick joystick40;
    private final PrototypeTesting prototype;

    public PrototypeTestCommand(Joystick joystick34, Joystick joystick40) {
        this.joystick34 = joystick34;
        this.joystick40 = joystick40;
        prototype = new PrototypeTesting();
        // addRequirements();
    }

    @Override
    public void execute() {
        prototype.set34Limit(joystick34.getZ());
        prototype.set40Limit(joystick40.getZ());
        prototype.run34(joystick34.getY());
        prototype.run40(joystick40.getY());
    }

    @Override
    public void end(boolean interrupted) {
        prototype.run34(0.0);
        prototype.run34(0.0);
        prototype.reset();
    }
}
