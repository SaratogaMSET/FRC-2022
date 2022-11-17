package frc.robot.subsystems;
 
import java.sql.Time;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.drivers.LazyTalonFX;


public class ShooterSubsystem extends SubsystemBase {
  public static final boolean SHOOTER_UP = true;
  public static final boolean SHOOTER_DOWN = false;
  public BooleanSupplier shooterReady = ()->false;

  // public static final PIDController pid = new PIDController(0.2, 0.03, 0);

  public static enum ShooterZone {
    MOVING, ZONE_1, ZONE_2, ZONE_3, ZONE_4, ZONE_5, ZONE_6, ZONE_7, TEST, EMERGENCY, LIRP_1, LIRP_2, LIRP_3, QUADRATIC
  };

  public static enum ShooterAngle {
    TWOFIVE, FOURZERO
  };

  public WPI_TalonFX shooterMotor1;
  public WPI_TalonFX shooterMotor2;
  // public LazyTalonFX shooterMotor1;
  // public LazyTalonFX shooterMotor2;
  private Solenoid shooterSolenoid;

  // private ShuffleboardTab tab = Shuffleboard.getTab("Teleop");
  // private NetworkTableEntry shooterPercentRPMEntry = tab.add("Shooter Percent RPM", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
  // private NetworkTableEntry shooterPercentOutputEntry = tab.add("Shooter Percent Output", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
  // private NetworkTableEntry shooterHoodEntry = tab.add("Shooter Hood Position", 0).getEntry();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // shooterMotor1 = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR1);
    // shooterMotor2 = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR2);
    shooterMotor1 = new WPI_TalonFX(Constants.ShooterConstants.SHOOTER_MOTOR1);
    shooterMotor2 = new WPI_TalonFX(Constants.ShooterConstants.SHOOTER_MOTOR2);

    shooterSolenoid = new Solenoid(2, PneumaticsModuleType.REVPH, Constants.ShooterConstants.SHOOTER_SOLENOID); //CHANGE VALUES
    shooterSolenoid.set(false);

    resetSensors();
  }

  public void setRPM(double rpm) {
    SmartDashboard.putNumber("DesiredRPM:", rpm);
    SmartDashboard.putNumber("RPM Difference",  ((600*4/3*(shooterMotor1.getSelectedSensorVelocity() - shooterMotor2.getSelectedSensorVelocity())/2/2048)-rpm));
    rpm *= 2.85/5.0;
    double rps = rpm/60;
    // SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.637, 0.14245, 0.0093589);
    // double feedforwardVoltage = feedforward.calculate(rpm);

    // SmartDashboard.putNumber("Velocity Setpoint", feedforwardVoltage);
    
    // shooterMotor1.set(ControlMode.PercentOutput, rpm);
    // shooterMotor2.set(ControlMode.PercentOutput, -rpm);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.637, 0.14002, 0.0092594);
    double feedVoltage = feedforward.calculate(rps);
    double readVelocity = 600*4/3*(shooterMotor1.getSelectedSensorVelocity() - shooterMotor2.getSelectedSensorVelocity())/2/2048;
    double feedbackGain = ((rpm - readVelocity)/(6380*4.0/3.0)) * 5;
    //if(rpm != 0) feedVoltage += feedbackGain;
    SmartDashboard.putNumber("KP Shooter Voltage", feedbackGain);
    SmartDashboard.putNumber("Voltage Setpoint", feedVoltage);
    
    //SmartDashboard.putNumber("ActualRPS:", 4/3*(shooterMotor1.getSelectedSensorVelocity() - shooterMotor2.getSelectedSensorVelocity())/2/2048);

    shooterMotor1.setVoltage(feedVoltage);
    shooterMotor2.setVoltage(-feedVoltage);

    // shooterReady = () -> rpm <= shooterMotor1.getMotorOutputVoltage()
    // * Constants.ShooterConstants.REV_UP_BUFFER / 6380
    // && rpm >= -shooterMotor2.getMotorOutputVoltage() * Constants.ShooterConstants.REV_UP_BUFFER
    //         / 6380 ? true : false;

    //At rpm ready to shoot
    //Not at RPM ball is not at ir
    //Not at RPM ball is at ir
    // SmartDashboard.putNumber("Difference in RPM", Math.abs(rpm / 2.85 * 5.0 - readVelocity));
    if(Math.abs(rpm / 2.85 * 5.0 - readVelocity) < 50){ 
      shooterReady = ()-> true;
  
    }else{
     shooterReady = ()->false;
   }
  }
  
  public void resetSensors() {
    shooterMotor1.setSelectedSensorPosition(0);
    shooterMotor2.setSelectedSensorPosition(0);
  }

  public double getShooterStateRPM(ShooterZone state, double distance) {
    if(state == ShooterZone.QUADRATIC){
    double a = 0.177437;
    double b = -32.7534;
    double c = 1.02 * 5371.93 ;
    return a * distance * distance + b * distance + c;
    }
    return 2000;
  }

  public void setAngle(boolean desiredAngle) {
    shooterSolenoid.set(!desiredAngle);
  }

  public ShooterZone getShooterZone(double distance) {
    // if(distance <= 0.0) {
    //   return ShooterZone.ZONE_4;
    // }
    // if (distance < Constants.Vision.Distance.LIRP_1) {
    //   return ShooterZone.LIRP_1;
    // } 
    // if (distance < Constants.Vision.Distance.LIRP_2) {
    //   return ShooterZone.LIRP_2;
    // } 
    // if (distance < Constants.Vision.Distance.LIRP_3) {
    //   return ShooterZone.LIRP_3;
    // } 
    if(RobotContainer.m_visionSubsystem!=null|| distance >0.0){
      return ShooterZone.QUADRATIC;
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
    SmartDashboard.putNumber("ActualRPM:", 600*4/3*(shooterMotor1.getSelectedSensorVelocity() - shooterMotor2.getSelectedSensorVelocity())/2/2048);
    SmartDashboard.putNumber("Sensor Curr Out:", shooterMotor1.getStatorCurrent());
    SmartDashboard.putNumber("Sensor Vel:", shooterMotor1.getSelectedSensorVelocity());
    SmartDashboard.putBoolean("Shooter Ready", shooterReady.getAsBoolean());
    
  //   shooterPercentRPMEntry.setDouble(shooterMotor1.getSelectedSensorVelocity() * Constants.ShooterConstants.kFalconSensorUnitsToRPM / Constants.ShooterConstants.kFalcon500FreeSpeed);
  //   shooterPercentOutputEntry.setDouble(shooterMotor1.getMotorOutputPercent());
  //   shooterHoodEntry.setDouble(shooterMotor2.getSelectedSensorPosition());
  }
}