package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.drivers.LazyTalonFX;
public class ShooterSubsystem extends SubsystemBase {
 
        public LazyTalonFX shooterMotor;
        private LazyTalonFX shooterMotor2;
 
        public ShooterSubsystem() {
                shooterMotor = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR);
                shooterMotor2 = new LazyTalonFX(Constants.ShooterConstants.LEADSCREW);
        }
        @Override
        public void periodic() {
                SmartDashboard.putNumber("Sensor Vel:", shooterMotor.getSelectedSensorVelocity());
                // Shuffleboard.
        }
        public void run(double d) {
                shooterMotor.set(TalonFXControlMode.PercentOutput,d);
                shooterMotor2.set(TalonFXControlMode.PercentOutput,-d);
        }
}
 
 
// package frc.robot.subsystems;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;
 
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.util.drivers.LazyTalonFX;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// public class ShooterSubsystem extends SubsystemBase{
  

//   public static enum ShooterState {
//       MOVING, ZONE_1, ZONE_2, ZONE_3, ZONE_4, ZONE_5, ZONE_6
//   }

//   public static enum ShooterStateAngle {
//         TWOFIVE, FOURZERO
//     }


//   private final LazyTalonFX shooterMotor;
//   private final LazyTalonFX lsMotor;
//   private Solenoid rightValve;
//   private Solenoid leftValve;




// private ShuffleboardTab tab = Shuffleboard.getTab("Teleop");
// private NetworkTableEntry shooterPercentRPMEntry = tab.add("Shooter Percent RPM", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
// private NetworkTableEntry shooterPercentOutputEntry = tab.add("Shooter Percent Output", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
// private NetworkTableEntry shooterHoodEntry = tab.add("Shooter Hood Position", 0).getEntry();
// private NetworkTableEntry leadscrewSetpointEntry = Shuffleboard.getTab("Teleop").add("Shooter Hood Setpoint", 0).withPosition(1, 2).getEntry();





// /** Creates a new ShooterSubsystem. */
// public ShooterSubsystem() {
//   shooterMotor = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR);
//   lsMotor = new LazyTalonFX(Constants.ShooterConstants.LS_MOTOR);
//   lsMotor.setInverted(true);
//   rightValve = new Solenoid(2, PneumaticsModuleType.REVPH, Constants.ShooterConstants.RIGHT_PISTON);
//   leftValve = new Solenoid(2, PneumaticsModuleType.REVPH, Constants.ShooterConstants.LEFT_PISTON);
 
//   resetSensors();
// }




// public void set(ControlMode mode, double demand) {
//   shooterMotor.set(mode, demand);
// }
// // public void set(double output) {
// //   set(ControlMode.PercentOutput, output);
// // }
// public void setRPM(double rpm) {
//   set(ControlMode.Velocity, rpm / Constants.ShooterConstants.kFalconSensorUnitsToRPM);
// }
// public void resetSensors() {
//   shooterMotor.setSelectedSensorPosition(0);
// }

// public void deploy(boolean status) {
//         if (status) { // moves the piston out if the status is true (intake down)
//             rightValve.set(true);
//             // leftValve.set(true);
//         } else { // moves the piston in if the status is false (intake up)
//             rightValve.set(false);
//             leftValve.set(false);
//         }
// }

// public ShooterStateAngle updateShooterStateAngle(){
//         if(rightValve.get()){
//                 return ShooterStateAngle.TWOFIVE;
//         } else {
//                 return ShooterStateAngle.FOURZERO;
//         }
// }


// public double getShooterStateRPM(ShooterState state) {
//   switch(state) {
//     case ZONE_2:
//       return Constants.ShooterConstants.DistanceConstants.ZONE_2.getRPM();
//     case ZONE_3:
//       return Constants.ShooterConstants.DistanceConstants.ZONE_3.getRPM();
//     case ZONE_4:
//       return Constants.ShooterConstants.DistanceConstants.ZONE_4.getRPM();
//     case ZONE_5:
//       return Constants.ShooterConstants.DistanceConstants.ZONE_5.getRPM();
//     case ZONE_6:
//       return Constants.ShooterConstants.DistanceConstants.ZONE_6.getRPM();
//     default:
//       return 0;
//   }
// }

// // public double getShooterStateAngle(ShooterStateAngle state) {
// //         switch(state) {
// //           case TWOFIVE:
// //             return Constants.ShooterConstants.AngleConstants.TWOFIVE.getAngle();
// //           case FOURZERO:
// //             return Constants.ShooterConstants.AngleConstants.TWOFIVE.getAngle();
// //           default:
// //             return 0;
// //         }
// // }




// // public void setLeadscrew(ControlMode mode, double demand) {
// //   lsMotor.set(mode, demand);
// // }
// // public void setLeadscrew(double encoderCount) {
// //   leadscrewSetpointEntry.setDouble(encoderCount);
// //   lsMotor.set(ControlMode.Position, encoderCount, DemandType.ArbitraryFeedForward, 0.0635);
// // }
// // public void resetLeadscrew() {
// //   lsMotor.setSelectedSensorPosition(0);
// // }
// // public double getAngle() {
// //   return lsMotor.getSelectedSensorPosition();
// // }
 



// // public boolean withinTolerance(ShooterState target) {
// //   return (Math.abs(getLeadscrewStatePosition(target) - lsMotor.getSelectedSensorPosition()) < Constants.ShooterConstants.LEADSCREW_TOLERANCE) ? true: false;
// // }


// // public boolean withinStateTolerance(ShooterState target) {
// //   return (Math.abs(getLeadscrewStatePosition(target) - lsMotor.getSelectedSensorPosition()) < Constants.ShooterConstants.LEADSCREW_STATE_TOLERANCE) ? true: false;
// // }




// public int getLeadscrewStatePosition(ShooterState state) {
//   switch(state) {
//     case ZONE_2:
//        return (int) Math.PI/360*55;
//       //return angleToEncoder(Constants.ShooterConstants.DistanceConstants.ZONE_2.getHoodAngle());
//     case ZONE_3:
//        return (int) Math.PI/360*55;
//       //return angleToEncoder(Constants.ShooterConstants.DistanceConstants.ZONE_3.getHoodAngle());
//     case ZONE_4:
//        return (int) Math.PI/360*55;
//       //return angleToEncoder(Constants.ShooterConstants.DistanceConstants.ZONE_4.getHoodAngle());
//     case ZONE_5:
//        return (int) Math.PI/360*55;
//       //return angleToEncoder(Constants.ShooterConstants.DistanceConstants.ZONE_5.getHoodAngle());
//     case ZONE_6:
//        return (int) Math.PI/360*55;
//       //return angleToEncoder(Constants.ShooterConstants.DistanceConstants.ZONE_6.getHoodAngle());
//     default:
//       return 0;
//   }
// }
 



// @Override
// public void periodic() {
//   // This method will be called once per scheduler run
//   // if both states are the same then cut off power
//   SmartDashboard.putNumber("Sensor Vel:", shooterMotor.getSelectedSensorVelocity());
//   shooterPercentRPMEntry.setDouble(shooterMotor.getSelectedSensorVelocity() * Constants.ShooterConstants.kFalconSensorUnitsToRPM / Constants.ShooterConstants.kFalcon500FreeSpeed);
//   shooterPercentOutputEntry.setDouble(shooterMotor.getMotorOutputPercent());
//   shooterHoodEntry.setDouble(lsMotor.getSelectedSensorPosition());
// }
// }