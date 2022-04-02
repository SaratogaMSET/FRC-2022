package frc.robot.subsystems;
 
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.drivers.LazyTalonFX;


public class ShooterSubsystem extends SubsystemBase {
  public static final boolean SHOOTER_UP = true;
  public static final boolean SHOOTER_DOWN = false;
  // public static final PIDController pid = new PIDController(0.2, 0.03, 0);

  public static enum ShooterZone {
    MOVING, ZONE_1, ZONE_2, ZONE_3, ZONE_4, ZONE_5, ZONE_6, ZONE_7, TEST, EMERGENCY, LIRP_1, LIRP_2, LIRP_3
  };

  public static enum ShooterAngle {
    TWOFIVE, FOURZERO
  };

  public LazyTalonFX shooterMotor1;
  public LazyTalonFX shooterMotor2;
  private Solenoid shooterSolenoid;

  // private ShuffleboardTab tab = Shuffleboard.getTab("Teleop");
  // private NetworkTableEntry shooterPercentRPMEntry = tab.add("Shooter Percent RPM", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
  // private NetworkTableEntry shooterPercentOutputEntry = tab.add("Shooter Percent Output", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
  // private NetworkTableEntry shooterHoodEntry = tab.add("Shooter Hood Position", 0).getEntry();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    shooterMotor1 = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR1);
    shooterMotor2 = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR2);

    shooterSolenoid = new Solenoid(2, PneumaticsModuleType.REVPH, Constants.ShooterConstants.SHOOTER_SOLENOID); //CHANGE VALUES
  
    resetSensors();
  }

  public void setRPM(double rpm) {

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(rpm, 0.14002, 0.0092594);
    double actual_rpm = feedforward.calculate(rpm);

    // SmartDashboard.putNumber("Velocity Setpoint", actual_rpm);
    
    // shooterMotor1.set(ControlMode.PercentOutput, rpm);
    // shooterMotor2.set(ControlMode.PercentOutput, -rpm);

    shooterMotor1.set(ControlMode.PercentOutput, actual_rpm);
    shooterMotor2.set(ControlMode.PercentOutput, -actual_rpm);
  }

  public void resetSensors() {
    shooterMotor1.setSelectedSensorPosition(0);
    shooterMotor2.setSelectedSensorPosition(0);
  }

  public double getShooterStateRPM(ShooterZone state, double distance) {
    double deltaRPM;
    double distanceFromSetpoint;
    double ratio;

    switch(state) {
      case ZONE_1:
        return Constants.ShooterConstants.DistanceConstants.ZONE_1.getPercentOutput();
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
      case ZONE_7:
        return Constants.ShooterConstants.DistanceConstants.ZONE_7.getPercentOutput();
      case EMERGENCY:
        return Constants.ShooterConstants.DistanceConstants.EMERGENCY.getPercentOutput();
      case TEST:
        return Constants.ShooterConstants.DistanceConstants.TEST.getPercentOutput();
      case LIRP_1:
        return Constants.ShooterConstants.DistanceConstants.LIRP_1.getPercentOutput();
      case LIRP_2:
        deltaRPM = Constants.ShooterConstants.DistanceConstants.LIRP_2.getPercentOutput() - Constants.ShooterConstants.DistanceConstants.LIRP_1.getPercentOutput();
        distanceFromSetpoint = distance - Constants.Vision.Distance.LIRP_1;
        ratio = distanceFromSetpoint/(Constants.Vision.Distance.LIRP_2-Constants.Vision.Distance.LIRP_1);
        return deltaRPM * ratio + Constants.ShooterConstants.DistanceConstants.LIRP_1.getPercentOutput();
      case LIRP_3:
        deltaRPM = Constants.ShooterConstants.DistanceConstants.LIRP_3.getPercentOutput() - Constants.ShooterConstants.DistanceConstants.LIRP_2.getPercentOutput();
        distanceFromSetpoint = distance - Constants.Vision.Distance.LIRP_2;
        ratio = distanceFromSetpoint/(Constants.Vision.Distance.LIRP_3-Constants.Vision.Distance.LIRP_2);
        return deltaRPM * ratio + Constants.ShooterConstants.DistanceConstants.LIRP_2.getPercentOutput();
      default:
        return Constants.ShooterConstants.DistanceConstants.EMERGENCY.getPercentOutput();
    }
  }

  public void setAngle(boolean desiredAngle) {
    shooterSolenoid.set(!desiredAngle);
  }

  public ShooterZone getShooterZone(double distance) {
    if(distance <= 0.0) {
      return ShooterZone.EMERGENCY;
    }
    if (distance < Constants.Vision.Distance.LIRP_1) {
      return ShooterZone.LIRP_1;
    } 
    if (distance < Constants.Vision.Distance.LIRP_2) {
      return ShooterZone.LIRP_2;
    } 
    if (distance < Constants.Vision.Distance.LIRP_3) {
      return ShooterZone.LIRP_3;
    } 
    // if (distance < Constants.Vision.Distance.ZONE_1) {
    //   return ShooterZone.ZONE_1;
    // } 
    // if (distance < Constants.Vision.Distance.ZONE_2) {
    //   return ShooterZone.ZONE_2;
    // } 
    // if (distance < Constants.Vision.Distance.ZONE_3) {
    //   return ShooterZone.ZONE_3;
    // } 
    // if (distance < Constants.Vision.Distance.ZONE_4) {
    //   return ShooterZone.ZONE_4;
    // }
    // if (distance < Constants.Vision.Distance.ZONE_5) {
    //   return ShooterZone.ZONE_5;
    // }
    // if (distance < Constants.Vision.Distance.ZONE_6) {
    //   return ShooterZone.ZONE_6;
    // }
    // if (distance < Constants.Vision.Distance.ZONE_7) {
    //   return ShooterZone.ZONE_7;
    // }
    return ShooterZone.EMERGENCY;
  }

  public boolean getShooterAngle(ShooterZone state) {
    switch(state) {
      case ZONE_1:
        return Constants.ShooterConstants.DistanceConstants.ZONE_1.getHoodAngle();
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
      case ZONE_7:
        return Constants.ShooterConstants.DistanceConstants.ZONE_7.getHoodAngle();
      case EMERGENCY:
        return Constants.ShooterConstants.DistanceConstants.EMERGENCY.getHoodAngle();

      case TEST:
        return Constants.ShooterConstants.DistanceConstants.TEST.getHoodAngle();
      default:
        return Constants.ShooterConstants.DistanceConstants.ZONE_1.getHoodAngle();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Sensor Curr Out:", shooterMotor1.getStatorCurrent());
    SmartDashboard.putNumber("Sensor Vel:", shooterMotor1.getSelectedSensorVelocity());
  //   shooterPercentRPMEntry.setDouble(shooterMotor1.getSelectedSensorVelocity() * Constants.ShooterConstants.kFalconSensorUnitsToRPM / Constants.ShooterConstants.kFalcon500FreeSpeed);
  //   shooterPercentOutputEntry.setDouble(shooterMotor1.getMotorOutputPercent());
  //   shooterHoodEntry.setDouble(shooterMotor2.getSelectedSensorPosition());
  }
}