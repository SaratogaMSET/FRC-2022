// package frc.robot.subsystems;
// import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.util.drivers.LazyTalonFX;
// public class ShooterSubsystem extends SubsystemBase {

//         public LazyTalonFX shooterMotor;
//         private LazyTalonFX shooterMotor2;

//         public ShooterSubsystem() {
//                 shooterMotor = new LazyTalonFX(Constants.ShooterConstants.ShooterConstants_MOTOR);
//                 shooterMotor2 = new LazyTalonFX(Constants.ShooterConstants.ShooterConstants_MOTOR_2);
//         }
//         @Override
//         public void periodic() {
//                 SmartDashboard.putNumber("Sensor Vel:", shooterMotor.getSelectedSensorVelocity());
//         }
//         public void run(double d) {
//                 shooterMotor.set(TalonFXControlMode.PercentOutput,d);
//                 shooterMotor2.set(TalonFXControlMode.PercentOutput,-d);
//         }
// }


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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
 
 
public class ShooterSubsystem extends SubsystemBase{
   public static enum ShooterState {
       BOTTOM, TOP, IDLE, MOVING, ZONE_1, ZONE_2, ZONE_3, ZONE_4, ZONE_5, ZONE_6
   }
 
   private final LazyTalonFX shooterMotor;
   private final LazyTalonFX lsMotor;
 
 private ShuffleboardTab tab = Shuffleboard.getTab("Teleop");
 // private NetworkTableEntry shooterRPMEntry = tab.add("Shooter RPM", 0).getEntry();
 private NetworkTableEntry shooterPercentRPMEntry = tab.add("Shooter Percent RPM", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
 private NetworkTableEntry shooterPercentOutputEntry = tab.add("Shooter Percent Output", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
//  private NetworkTableEntry shooterStateEntry = tab.add("Shooter State", "").getEntry();
 private NetworkTableEntry shooterHoodEntry = tab.add("Shooter Hood Position", 0).getEntry();
 private NetworkTableEntry leadscrewSetpointEntry = Shuffleboard.getTab("Teleop").add("Shooter Hood Setpoint", 0).withPosition(1, 2).getEntry();
 
 // constants
//  private double D1 = 4.25, D2 = 6.25, C1 = .5, C2 = .75;
//  private double ANGLE_OFFSET = 6.32, INIT_LS_LEN = 2.8;
 
 /** Creates a new ShooterSubsystem. */
 public ShooterSubsystem() {
   shooterMotor = new LazyTalonFX(Constants.ShooterConstants.SHOOTER_MOTOR);
 
   lsMotor = new LazyTalonFX(Constants.ShooterConstants.LS_MOTOR);
   lsMotor.setInverted(true);

   resetSensors();
 }
 
 
 public void set(ControlMode mode, double demand) {
   shooterMotor.set(mode, demand);
 }
 
 public void set(double output) {
   set(ControlMode.PercentOutput, output);
 }
 
 public void setRPM(double rpm) {
   set(ControlMode.Velocity, rpm / Constants.ShooterConstants.kFalconSensorUnitsToRPM);
 }
 
 public void resetSensors() {
   shooterMotor.setSelectedSensorPosition(0);
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
 
 public void setLeadscrew(ControlMode mode, double demand) {
   lsMotor.set(mode, demand);
 }
 
 public void setLeadscrew(double encoderCount) {
   leadscrewSetpointEntry.setDouble(encoderCount);
   lsMotor.set(ControlMode.Position, encoderCount, DemandType.ArbitraryFeedForward, 0.0635);
 }
 
 public void resetLeadscrew() {
   lsMotor.setSelectedSensorPosition(0);
 }
 
 public double getLeadscrewPosition() {
   return lsMotor.getSelectedSensorPosition();
 }

 
//  public int angleToEncoder(double theta) {
//    // offset then convert to rad
//    theta += ANGLE_OFFSET;
//    theta = Math.toRadians(theta);
 
//    // leadscrew length
//    double lsLen = Math.sqrt((-2 * D1 * D2 * Math.cos(theta)) + Math.pow(D1, 2) + Math.pow(D2, 2) - Math.pow(C1, 2)
//        - Math.pow(C2, 2) - (2 * C1 * C2));
//    double encoderCount = (lsLen - INIT_LS_LEN) * 10 * 2048;
 
//    // then run leadscrew motor to distance
//    return (int) encoderCount;
//  }
 
 public boolean withinTolerance(ShooterState target) {
   return (Math.abs(getLeadscrewStatePosition(target) - lsMotor.getSelectedSensorPosition()) < Constants.ShooterConstants.LEADSCREW_TOLERANCE) ? true: false;
 }
 
 public boolean withinStateTolerance(ShooterState target) {
   return (Math.abs(getLeadscrewStatePosition(target) - lsMotor.getSelectedSensorPosition()) < Constants.ShooterConstants.LEADSCREW_STATE_TOLERANCE) ? true: false;
 }
 
 public int getLeadscrewStatePosition(ShooterState state) {
   switch(state) {
 
     case ZONE_2:
        return (int) Math.PI/360*55;
       //return angleToEncoder(Constants.ShooterConstants.DistanceConstants.ZONE_2.getHoodAngle());
     case ZONE_3:
        return (int) Math.PI/360*55;
       //return angleToEncoder(Constants.ShooterConstants.DistanceConstants.ZONE_3.getHoodAngle());
     case ZONE_4:
        return (int) Math.PI/360*55;
       //return angleToEncoder(Constants.ShooterConstants.DistanceConstants.ZONE_4.getHoodAngle());
     case ZONE_5:
        return (int) Math.PI/360*55;
       //return angleToEncoder(Constants.ShooterConstants.DistanceConstants.ZONE_5.getHoodAngle());
     case ZONE_6:
        return (int) Math.PI/360*55;
       //return angleToEncoder(Constants.ShooterConstants.DistanceConstants.ZONE_6.getHoodAngle());
     default:
       return 0;
   }
 }

 @Override
 public void periodic() {
   // This method will be called once per scheduler run
 
   // if both states are the same then cut off power
   SmartDashboard.putNumber("Sensor Vel:", shooterMotor.getSelectedSensorVelocity());
 
   shooterPercentRPMEntry.setDouble(shooterMotor.getSelectedSensorVelocity() * Constants.ShooterConstants.kFalconSensorUnitsToRPM / Constants.ShooterConstants.kFalcon500FreeSpeed);
   shooterPercentOutputEntry.setDouble(shooterMotor.getMotorOutputPercent());
   shooterHoodEntry.setDouble(lsMotor.getSelectedSensorPosition());
 }
 
 }
 
 
