 package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
 
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.drivers.LazyTalonFX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
public class ShooterSubsystem extends SubsystemBase{

        public static final boolean SHOOTER_UP = true;
        public static final boolean SHOOTER_DOWN = false;

  

  public static enum ShooterState {
      MOVING, ZONE_1, ZONE_2, ZONE_3, ZONE_4, ZONE_5, ZONE_6
  }

  public static enum ShooterStateAngle {
        TWOFIVE, FOURZERO
    }


  private final LazyTalonFX shooterMotor;
  private final LazyTalonFX shooterMotor2;
  private Solenoid shooterValve;




private ShuffleboardTab tab = Shuffleboard.getTab("Teleop");
private NetworkTableEntry shooterPercentRPMEntry = tab.add("Shooter Percent RPM", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
private NetworkTableEntry shooterPercentOutputEntry = tab.add("Shooter Percent Output", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
private NetworkTableEntry shooterHoodEntry = tab.add("Shooter Hood Position", 0).getEntry();





/** Creates a new ShooterSubsystem. */
public ShooterSubsystem() {
  shooterMotor = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR);
  shooterMotor2 = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR2);

  shooterValve = new Solenoid(2, PneumaticsModuleType.REVPH, Constants.ShooterConstants.SHOOTER_PISTON); //CHANGE VALUES
 
  resetSensors();
}




public void set(ControlMode mode, double demand) {
  shooterMotor.set(mode, demand);
}

public void setRPM(double rpm) {
  shooterMotor.set(ControlMode.Velocity, rpm / Constants.ShooterConstants.kFalconSensorUnitsToRPM);
  shooterMotor2.set(ControlMode.Velocity, - rpm / Constants.ShooterConstants.kFalconSensorUnitsToRPM);
}
public void resetSensors() {
  shooterMotor.setSelectedSensorPosition(0);
}

public void deploy(boolean status) {
        if (status) { // moves the piston out if the status is true (shooter down)
                shooterValve.set(true);
        } else { // moves the piston in if the status is false (shooter up)
                shooterValve.set(false);
        }
}

public ShooterStateAngle updateShooterStateAngle(){

        if(shooterValve.get()){
                return ShooterStateAngle.TWOFIVE;
        } else {
                return ShooterStateAngle.FOURZERO;
        }
}


public double getShooterStateRPM(ShooterState state) {
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
  SmartDashboard.putNumber("Sensor Vel:", shooterMotor.getSelectedSensorVelocity());
  shooterPercentRPMEntry.setDouble(shooterMotor.getSelectedSensorVelocity() * Constants.ShooterConstants.kFalconSensorUnitsToRPM / Constants.ShooterConstants.kFalcon500FreeSpeed);
  shooterPercentOutputEntry.setDouble(shooterMotor.getMotorOutputPercent());
  shooterHoodEntry.setDouble(shooterMotor2.getSelectedSensorPosition());
}




public ShooterStateAngle getAngleState(ShooterState state) {
        switch(state) {
                case ZONE_2:
                        return ShooterStateAngle.TWOFIVE;
                case ZONE_3:
                        return ShooterStateAngle.TWOFIVE;
                case ZONE_4:
                        return ShooterStateAngle.TWOFIVE;
                case ZONE_5:
                        return ShooterStateAngle.TWOFIVE;
                case ZONE_6:
                        return ShooterStateAngle.TWOFIVE;
                default:
                        return ShooterStateAngle.TWOFIVE;
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
 
