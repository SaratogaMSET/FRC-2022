package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Shooter.ShootCommand;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;
public class LeftTrigger extends Trigger {
    private final XboxController m_driver;
    public LeftTrigger(XboxController m_driver){
        this.m_driver = m_driver;
    }
    public boolean get(){
        if(m_driver.getLeftTriggerAxis()>0.5){
            return true;
    }
    return false;
}
public Trigger whileActiveOnce(final Command command, boolean interruptible) {
    requireNonNullParam(command, "command", "whileActiveOnce");

    CommandScheduler.getInstance()
        .addButton(
            new Runnable() {
              private boolean m_pressedLast = get();

              @Override
              public void run() {
                boolean pressed = get();

                if (!m_pressedLast && pressed) {
                  command.schedule(interruptible);
                } else if (m_pressedLast && !pressed) {
                  command.cancel();
                }

                m_pressedLast = pressed;
              }
            });
    return this;
  }
  public Trigger whenPressed(final Runnable toRun) {
    whenActive(toRun);
    return this;
  }
  public Trigger whenReleased(final Command command) {
    whenInactive(command);
    return this;
  }
public void whileActiveOnce(double d, ParallelCommandGroup command, boolean b) {
  CommandScheduler.getInstance()
        .addButton(
            new Runnable() {
              private boolean m_pressedLast = get();

              @Override
              public void run() {
                boolean pressed = get();

                if (!m_pressedLast && pressed) {
                  command.schedule(b);
                } else if (m_pressedLast && !pressed) {
                  command.cancel();
                }

                m_pressedLast = pressed;
              }
            });
}
public Trigger whenReleased(InstantCommand instantCommand, double d) {
  whenInactive(instantCommand);
    return this;
}
}
