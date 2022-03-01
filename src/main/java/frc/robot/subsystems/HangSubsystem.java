package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class HangSubsystem extends SubsystemBase {
    private final TalonSRX hangMotor;

    public HangSubsystem(int motorType) {
        hangMotor = new TalonSRX(motorType);
    }

    public void setHangSpeed(double speed) {
        hangMotor.set(ControlMode.PercentOutput, speed);
    }
}