package frc.robot.subsystems;
 
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.drivers.LazyTalonFX;

public class ShooterSubsystem extends SubsystemBase {
  public static final boolean SHOOTER_UP = true;
  public static final boolean SHOOTER_DOWN = false;

  public static enum ShooterZone {
    MOVING, ZONE_1, ZONE_2, ZONE_3, ZONE_4, ZONE_5, ZONE_6
  };

  public static enum ShooterAngle {
    TWOFIVE, FOURZERO
  };

  private LazyTalonFX shooterMotor1;
  private LazyTalonFX shooterMotor2;
  private Solenoid shooterSolenoid;

  private ShuffleboardTab tab = Shuffleboard.getTab("Teleop");
  private NetworkTableEntry shooterPercentRPMEntry = tab.add("Shooter Percent RPM", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
  private NetworkTableEntry shooterPercentOutputEntry = tab.add("Shooter Percent Output", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
  private NetworkTableEntry shooterHoodEntry = tab.add("Shooter Hood Position", 0).getEntry();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterMotor1 = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR1);
    shooterMotor2 = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR2);

    shooterSolenoid = new Solenoid(2, PneumaticsModuleType.REVPH, Constants.ShooterConstants.SHOOTER_SOLENOID);
  }

  public void setAngle(ShooterAngle desiredAngle) {
    if (desiredAngle == ShooterAngle.FOURZERO)
      shooterSolenoid.set(SHOOTER_UP);
    else
      shooterSolenoid.set(SHOOTER_DOWN);
  }

  public void setRPM(double rpm) {
    shooterMotor1.set(ControlMode.Velocity, rpm / Constants.ShooterConstants.kFalconSensorUnitsToRPM);
    shooterMotor2.set(ControlMode.Velocity, - rpm / Constants.ShooterConstants.kFalconSensorUnitsToRPM);
  }

  public ShooterZone getShooterZone(double distance) {
    if(distance == 0.0) {
      return ShooterZone.ZONE_2;
    }

    if (distance < Constants.Vision.Distance.ZONE_2) {
      return ShooterZone.ZONE_2;
    } 
    
    if (distance < Constants.Vision.Distance.ZONE_3) {
      return ShooterZone.ZONE_3;
    } 
    
    if (distance < Constants.Vision.Distance.ZONE_4) {
      return ShooterZone.ZONE_4;
    } 
    
    if (distance < Constants.Vision.Distance.ZONE_5) {
      return ShooterZone.ZONE_5;
    } 
    
    if (distance < Constants.Vision.Distance.ZONE_6) {
      return ShooterZone.ZONE_6;
    } 
    
    return ShooterZone.ZONE_6;
  }

  public double getDesiredShooterRPM(ShooterZone state) {
    switch(state) {
      case ZONE_2:
        return Constants.ShooterConstants.DistanceConstants.ZONE_2.getRPM();
      case ZONE_3:
        return Constants.ShooterConstants.DistanceConstants.ZONE_3.getRPM();
      case ZONE_4:
        return Constants.ShooterConstants.DistanceConstants.ZONE_4.getRPM();
      case ZONE_5:
        return Constants.ShooterConstants.DistanceConstants.ZONE_5.getRPM();
      case ZONE_6:
        return Constants.ShooterConstants.DistanceConstants.ZONE_6.getRPM();
      default:
        return 0;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Sensor Vel:", shooterMotor1.getSelectedSensorVelocity());
    shooterPercentRPMEntry.setDouble(shooterMotor1.getSelectedSensorVelocity() * Constants.ShooterConstants.kFalconSensorUnitsToRPM / Constants.ShooterConstants.kFalcon500FreeSpeed);
    shooterPercentOutputEntry.setDouble(shooterMotor1.getMotorOutputPercent());
    shooterHoodEntry.setDouble(shooterMotor2.getSelectedSensorPosition());
  }

  public ShooterAngle getDesiredShooterAngle(ShooterZone zone) {
    switch(zone) {
      case ZONE_2:
        return ShooterAngle.TWOFIVE;
      case ZONE_3:
        return ShooterAngle.TWOFIVE;
      case ZONE_4:
        return ShooterAngle.TWOFIVE;
      case ZONE_5:
        return ShooterAngle.TWOFIVE;
      case ZONE_6:
        return ShooterAngle.TWOFIVE;
      default:
        return ShooterAngle.TWOFIVE;
    }
  }
}


// /* Code for manually testing */
// package frc.robot.subsystems;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.util.drivers.LazyTalonFX;
// public class ShooterSubsystem extends SubsystemBase {
 
//         public WPI_TalonFX shooterMotor; //change to LazyTalonFX for safety
//         public WPI_TalonFX shooterMotor2; //change to LazyTalonFX for safety
 
//         public ShooterSubsystem() {
//                 shooterMotor = new WPI_TalonFX(Constants.ShooterConstants.SHOOTER_MOTOR); //change to LazyTalonFX for safety
//                 shooterMotor2 = new WPI_TalonFX(Constants.ShooterConstants.LEADSCREW); //change to LazyTalonFX for safety
//         }
//         @Override
//         public void periodic() {
//                 SmartDashboard.putNumber("Sensor Vel:", shooterMotor.getSelectedSensorVelocity());
//                 // Shuffleboard.
//         }
//         public void run(double d) {
//                 shooterMotor.set(TalonFXControlMode.PercentOutput,d);
//                 shooterMotor2.set(TalonFXControlMode.PercentOutput,-d);
//         }

// }
 
