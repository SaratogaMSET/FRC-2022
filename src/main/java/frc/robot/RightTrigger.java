package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;
public class RightTrigger extends Trigger {
    private final XboxController m_driver;
    public RightTrigger(XboxController m_driver){
        this.m_driver = m_driver;
    }
    public boolean get(){
        if(m_driver.getRightTriggerAxis()>0.5){
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
}
