package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.drivers.LazyTalonFX;
public class ShooterSubsystem extends SubsystemBase {

        public LazyTalonFX shooterMotor;
        private LazyTalonFX shooterMotor2;

        public ShooterSubsystem() {
                shooterMotor = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR);
                shooterMotor2 = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR_2);
        }
        @Override
        public void periodic() {
                SmartDashboard.putNumber("Sensor Vel:", shooterMotor.getSelectedSensorVelocity());
        }
        public void run(double d) {
                shooterMotor.set(TalonFXControlMode.PercentOutput,d);
                shooterMotor2.set(TalonFXControlMode.PercentOutput,-d);
        }
}