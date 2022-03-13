package frc.robot.subsystems;
 
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  private static final boolean SHOOTER_UP = false;
  private static final boolean SHOOTER_DOWN = true;

  public static enum ShooterZone {
    MOVING, ZONE_1, ZONE_2, ZONE_3, ZONE_4, ZONE_5, ZONE_6, TEST
  };

  public static enum ShooterAngle {
    TWOFIVE, FOURZERO
  };

  public LazyTalonFX shooterMotor1;
  public LazyTalonFX shooterMotor2;
  public Solenoid shooterSolenoid;

  private double m_lastNonZeroRPM = 0.60;

  private ShuffleboardTab tab = Shuffleboard.getTab("Teleop");
  private NetworkTableEntry shooterPercentRPMEntry = tab.add("Shooter Percent RPM", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
  private NetworkTableEntry shooterPercentOutputEntry = tab.add("Shooter Percent Output", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
  private NetworkTableEntry shooterHoodEntry = tab.add("Shooter Hood Position", 0).getEntry();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterMotor1 = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR1);
    shooterMotor2 = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR2);

    shooterSolenoid = new Solenoid(2, PneumaticsModuleType.REVPH, Constants.ShooterConstants.SHOOTER_SOLENOID); //CHANGE VALUES
  
    resetSensors();
  }

  public void setRPM(double rpm) {

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.66189, 0.14002, 0.0092594);
    double actual_rpm = feedforward.calculate(rpm);

    shooterMotor1.set(ControlMode.PercentOutput, actual_rpm);
    shooterMotor2.set(ControlMode.PercentOutput, -actual_rpm);

    if (rpm > 0) {
      m_lastNonZeroRPM = rpm;
    }
  }

  public void resetSensors() {
    shooterMotor1.setSelectedSensorPosition(0);
    shooterMotor2.setSelectedSensorPosition(0);
  }

  private ShooterAngle getShooterAngle(){
      if(shooterSolenoid.get()) {
        return ShooterAngle.TWOFIVE;
      } else {
        return ShooterAngle.FOURZERO;
      }
  }

  public void toggleShooterAngle() {
    ShooterAngle angle = getShooterAngle();
    if (angle == ShooterAngle.FOURZERO) {
      setShooterAngle(ShooterAngle.TWOFIVE);
    } else {
      setShooterAngle(ShooterAngle.FOURZERO);
    }
  }

  public double getShooterRPM(ShooterZone state) {
    switch(state) {
      case ZONE_2:
        return Constants.ShooterConstants.DistanceConstants.ZONE_2.getPercentOutput();
      case ZONE_3:
        return Constants.ShooterConstants.DistanceConstants.ZONE_3.getPercentOutput();
      case ZONE_4:
        return Constants.ShooterConstants.DistanceConstants.ZONE_4.getPercentOutput();
      case ZONE_5:
        return Constants.ShooterConstants.DistanceConstants.ZONE_5.getPercentOutput();
      case ZONE_6:
        return Constants.ShooterConstants.DistanceConstants.ZONE_6.getPercentOutput();
      case TEST:
        return Constants.ShooterConstants.DistanceConstants.TEST.getPercentOutput();
      default:
        return 0;
    }
  }

  public void setShooterAngle(ShooterAngle desiredAngle) {
    if (desiredAngle == ShooterAngle.FOURZERO)
      shooterSolenoid.set(SHOOTER_UP);
    else
      shooterSolenoid.set(SHOOTER_DOWN);
  }

  public ShooterZone getShooterZone(double distance) {
    if(distance <= 0.0) {
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

  public ShooterAngle getShooterAngle(ShooterZone state) {
    switch(state) {
      case ZONE_2:
        return Constants.ShooterConstants.DistanceConstants.ZONE_2.getHoodAngle();
      case ZONE_3:
        return Constants.ShooterConstants.DistanceConstants.ZONE_3.getHoodAngle();
      case ZONE_4:
        return Constants.ShooterConstants.DistanceConstants.ZONE_4.getHoodAngle();
      case ZONE_5:
        return Constants.ShooterConstants.DistanceConstants.ZONE_5.getHoodAngle();
      case ZONE_6:
        return Constants.ShooterConstants.DistanceConstants.ZONE_6.getHoodAngle();
      case TEST:
        return Constants.ShooterConstants.DistanceConstants.TEST.getHoodAngle();
      default:
        return Constants.ShooterConstants.DistanceConstants.ZONE_2.getHoodAngle();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Sensor Curr Out:", shooterMotor1.getStatorCurrent());
    SmartDashboard.putNumber("Sensor Vel:", shooterMotor1.getSelectedSensorVelocity());
    shooterPercentRPMEntry.setDouble(shooterMotor1.getSelectedSensorVelocity() * Constants.ShooterConstants.kFalconSensorUnitsToRPM / Constants.ShooterConstants.kFalcon500FreeSpeed);
    shooterPercentOutputEntry.setDouble(shooterMotor1.getMotorOutputPercent());
    shooterHoodEntry.setDouble(shooterMotor2.getSelectedSensorPosition());
  }

  public double getLastNonZeroRPM() {
      return m_lastNonZeroRPM;
  }
}
