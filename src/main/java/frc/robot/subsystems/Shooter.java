package frc.robot.subsystems;
 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
 
import org.opencv.calib3d.StereoSGBM;
 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.util.drivers.LazyTalonFX;
 
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.drivers.LazyTalonFX;
import frc.robot.util.drivers.TalonFXFactory;
 
 
public class Shooter extends SubsystemBase{
   public static enum ShooterState {
       BOTTOM, TOP, IDLE, MOVING, ZONE_1, ZONE_2, ZONE_3, ZONE_4, ZONE_5, ZONE_6
   }
 
   private final LazyTalonFX shooterMotor;
 
   private final LazyTalonFX lsMotor;
 
 private ShuffleboardTab tab = Shuffleboard.getTab("Teleop");
 // private NetworkTableEntry shooterRPMEntry = tab.add("Shooter RPM", 0).getEntry();
 private NetworkTableEntry shooterPercentRPMEntry = tab.add("Shooter Percent RPM", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
 private NetworkTableEntry shooterPercentOutputEntry = tab.add("Shooter Percent Output", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
 private NetworkTableEntry shooterStateEntry = tab.add("Shooter State", "").getEntry();
 private NetworkTableEntry shooterHoodEntry = tab.add("Shooter Hood Position", 0).getEntry();
 private NetworkTableEntry leadscrewSetpointEntry = Shuffleboard.getTab("Teleop").add("Shooter Hood Setpoint", 0).withPosition(1, 2).getEntry();
 
 // constants
 private double D1 = 4.25, D2 = 6.25, C1 = .5, C2 = .75;
 private double ANGLE_OFFSET = 6.32, INIT_LS_LEN = 2.8;
 
 /** Creates a new ShooterSubsystem. */
 public Shooter() {
   shooterMotor = new LazyTalonFX(Constants.Shooter.SHOOTER_MOTOR);
   // shooterMotor.setNeutralMode(NeutralMode.Coast);
   // // shooterMotor.setInverted(true);
   // shooterMotor.config_kF(0, 0.04882);
   // shooterMotor.config_kP(0, 0.08);
   // shooterMotor.config_kI(0, 0.0005);
   // shooterMotor.config_kD(0, 0.0);
   // shooterMotor.config_IntegralZone(0, 800);
 
 
 
   lsMotor = new LazyTalonFX(Constants.Shooter.LS_MOTOR);
   lsMotor.setInverted(true);
   // lsMotor.setNeutralMode(NeutralMode.Brake);
   // lsMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
   // lsMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
   // lsMotor.overrideLimitSwitchesEnable(true);
   // lsMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
   // lsMotor.configMotionCruiseVelocity(10000);
   // lsMotor.configMotionAcceleration(7500);
   // // lsMotor.config_kP(0, 0.00);
   // // lsMotor.config_kI(0, 0.00);
   // // lsMotor.config_kD(0, 0.00);
   // lsMotor.config_kP(0, 0.03);
   // lsMotor.config_kI(0, 0.0001);
   // lsMotor.config_kD(0, 0.003);
   // lsMotor.config_IntegralZone(0, 1024);
 
   // normClosed = new DigitalInput(Constants.Shooter.LIMIT_SWITCH[0]);
   // normOpen = new DigitalInput(Constants.Shooter.LIMIT_SWITCH[1]);
 
   resetSensors();
 }
 
 public ShooterState updateShooterState() {
   SmartDashboard.putNumber("Hood Angle", encoderToAngle(getLeadscrewPosition()));
   SmartDashboard.putNumber("Hood Angle", getLeadscrewPosition());
 
   double leadscrewPower = lsMotor.getMotorOutputPercent();
   if (lsMotor.isFwdLimitSwitchClosed() == 0) {
     shooterStateEntry.setString("Top");
     return ShooterState.TOP;
   } else if (withinStateTolerance(ShooterState.ZONE_2)) {
     shooterStateEntry.setString("Zone 2");
     return ShooterState.ZONE_2;
   } else if (withinStateTolerance(ShooterState.ZONE_3)) {
     shooterStateEntry.setString("Zone 3");
     return ShooterState.ZONE_3;
   }  else if (withinStateTolerance(ShooterState.ZONE_4)) {
     shooterStateEntry.setString("Zone 4");
     return ShooterState.ZONE_4;
   } else if (withinStateTolerance(ShooterState.ZONE_5)) {
     shooterStateEntry.setString("Zone 5");
     return ShooterState.ZONE_5;
   }else if (withinStateTolerance(ShooterState.ZONE_6)) {
     shooterStateEntry.setString("Zone 6");
     return ShooterState.ZONE_6;
   }else if (leadscrewPower == 0) {
     shooterStateEntry.setString("Idle");
     return ShooterState.IDLE;
   }
   shooterStateEntry.setString("Moving");
   return ShooterState.MOVING;
 }
 
 public void set(ControlMode mode, double demand) {
   shooterMotor.set(mode, demand);
 }
 
 public void set(double output) {
   set(ControlMode.PercentOutput, output);
 }
 
 public void setRPM(double rpm) {
   set(ControlMode.Velocity, rpm / Constants.Shooter.kFalconSensorUnitsToRPM);
 }
 
 public void resetSensors() {
   shooterMotor.setSelectedSensorPosition(0);
 }
 
 public double getShooterStateRPM(ShooterState state) {
   switch(state) {
     case ZONE_2:
       return Constants.Shooter.DistanceConstants.ZONE_2.getRPM();
     case ZONE_3:
       return Constants.Shooter.DistanceConstants.ZONE_3.getRPM();
     case ZONE_4:
       return Constants.Shooter.DistanceConstants.ZONE_4.getRPM();
     case ZONE_5:
       return Constants.Shooter.DistanceConstants.ZONE_5.getRPM();
     case ZONE_6:
       return Constants.Shooter.DistanceConstants.ZONE_6.getRPM();
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
   // lsMotor.set(ControlMode.MotionMagic, encoderCount, DemandType.ArbitraryFeedForward, 0.0635);
 }
 
 public void resetLeadscrew() {
   lsMotor.setSelectedSensorPosition(0);
 }
 
 public double getLeadscrewPosition() {
   return lsMotor.getSelectedSensorPosition();
 }
  public double encoderToAngle(double enc){
   double theta = Math.pow(((enc)/(2048 * 10) + INIT_LS_LEN), 2);
   theta += (2 * C1 * C2);
   theta += Math.pow(C1, 2) + Math.pow(C2, 2);
   theta += -Math.pow(D1, 2) - Math.pow(D2, 2);
   theta = theta/(-2 * D1 * D2);
   theta = Math.acos(theta);
   return Math.toDegrees(theta)-ANGLE_OFFSET;
 }
 
 public int angleToEncoder(double theta) {
   // offset then convert to rad
   theta += ANGLE_OFFSET;
   theta = Math.toRadians(theta);
 
   // leadscrew length
   double lsLen = Math.sqrt((-2 * D1 * D2 * Math.cos(theta)) + Math.pow(D1, 2) + Math.pow(D2, 2) - Math.pow(C1, 2)
       - Math.pow(C2, 2) - (2 * C1 * C2));
   double encoderCount = (lsLen - INIT_LS_LEN) * 10 * 2048;
 
   // then run leadscrew motor to distance
   return (int) encoderCount;
 }
 
 public boolean withinTolerance(ShooterState target) {
   return (Math.abs(getLeadscrewStatePosition(target) - lsMotor.getSelectedSensorPosition()) < Constants.Shooter.LEADSCREW_TOLERANCE) ? true: false;
 }
 
 public boolean withinStateTolerance(ShooterState target) {
   return (Math.abs(getLeadscrewStatePosition(target) - lsMotor.getSelectedSensorPosition()) < Constants.Shooter.LEADSCREW_STATE_TOLERANCE) ? true: false;
 }
 
 public int getLeadscrewStatePosition(ShooterState state) {
   switch(state) {
 
     case ZONE_2:
       return angleToEncoder(Constants.Shooter.DistanceConstants.ZONE_2.getHoodAngle());
     case ZONE_3:
       return angleToEncoder(Constants.Shooter.DistanceConstants.ZONE_3.getHoodAngle());
     case ZONE_4:
       return angleToEncoder(Constants.Shooter.DistanceConstants.ZONE_4.getHoodAngle());
     case ZONE_5:
       return angleToEncoder(Constants.Shooter.DistanceConstants.ZONE_5.getHoodAngle());
     case ZONE_6:
       return angleToEncoder(Constants.Shooter.DistanceConstants.ZONE_6.getHoodAngle());
     default:
       return 0;
   }
 }
 
 // public void increaseTESTAngle() {
 //   Constants.Shooter.DistanceConstants.TEST.setHoodAngle(
 //     Constants.Shooter.DistanceConstants.TEST.getHoodAngle() + 2);
 // }
 
 // public void decreaseTESTAngle() {
 //   Constants.Shooter.DistanceConstants.TEST.setHoodAngle(
 //     Constants.Shooter.DistanceConstants.TEST.getHoodAngle() - 2);
 // }
 
 @Override
 public void periodic() {
   // This method will be called once per scheduler run
 
   // if both states are the same then cut off power
   SmartDashboard.putNumber("Sensor Vel:", shooterMotor.getSelectedSensorVelocity());
 
   // shooterRPMEntry.setDouble(shooterMotor.getSelectedSensorVelocity() * Constants.kFalconSensorUnitsToRPM);
   shooterPercentRPMEntry.setDouble(shooterMotor.getSelectedSensorVelocity() * Constants.Shooter.kFalconSensorUnitsToRPM / Constants.Shooter.kFalcon500FreeSpeed);
   shooterPercentOutputEntry.setDouble(shooterMotor.getMotorOutputPercent());
   shooterHoodEntry.setDouble(lsMotor.getSelectedSensorPosition());
 }
 
 }
 
 

