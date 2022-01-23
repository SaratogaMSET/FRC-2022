package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.drivers.LazyTalonFX;
public class ShooterSubsystem extends SubsystemBase {

        public LazyTalonFX shooterMotor;
        private LazyTalonFX lsMotor;

        public ShooterSubsystem() {
                shooterMotor = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR);
                lsMotor = new LazyTalonFX(Constants.ShooterConstants.LS_MOTOR);
        }
        @Override
        public void periodic() {
                SmartDashboard.putNumber("Sensor Vel:", shooterMotor.getSelectedSensorVelocity());
        }
        public void run(double d) {
                shooterMotor.set(TalonFXControlMode.PercentOutput,0.85);
                lsMotor.set(TalonFXControlMode.PercentOutput,-0.85);
        }
}