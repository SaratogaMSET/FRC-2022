// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Drivetrain;
// import frc.robot.util.drivers.LazyTalonFX;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  
  public static final double MAX_VELOCITY_METERS_PER_SECOND = (6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI);
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0);


  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(m_kinematics,
          new Rotation2d(0));

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.

 public final static AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  public double offset = 0;

  // These are our modules. We initialize them in the constructor.
  public final SwerveModule m_frontLeftModule;
  public final SwerveModule m_frontRightModule;
  public final SwerveModule m_backLeftModule;
  public final SwerveModule m_backRightModule;
  
  private SwerveModuleState[] currentState = new SwerveModuleState[4];
  private SwerveModuleState[] previousState = new SwerveModuleState[4];

  private double trackingAngle = 0;
  private boolean trackingState = false;

  BooleanLogEntry myBooleanLog;
  DoubleLogEntry myDoubleLog;
  StringLogEntry myStringLog;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  // private LazyTalonFX m_frontLeftSteer;
  // private LazyTalonFX m_frontRightSteer;
  // private LazyTalonFX m_backLeftSteer;
  // private LazyTalonFX m_backRightSteer;

  // private LazyTalonFX m_frontLeftDrive;
  // private LazyTalonFX m_frontRightDrive;
  // private LazyTalonFX m_backLeftDrive;
  // private LazyTalonFX m_backRightDrive;

  public DrivetrainSubsystem() {
    // m_frontLeftSteer = new LazyTalonFX(Drivetrain.FRONT_LEFT_MODULE_STEER_MOTOR);
    // m_frontRightSteer = new LazyTalonFX(Drivetrain.FRONT_RIGHT_MODULE_STEER_MOTOR);
    // m_backLeftSteer = new LazyTalonFX(Drivetrain.BACK_LEFT_MODULE_STEER_MOTOR);
    // m_backRightSteer = new LazyTalonFX(Drivetrain.BACK_RIGHT_MODULE_STEER_MOTOR);

    // m_frontLeftDrive = new LazyTalonFX(Drivetrain.FRONT_LEFT_MODULE_DRIVE_MOTOR);
    // m_frontRightDrive = new LazyTalonFX(Drivetrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR);
    // m_backLeftDrive = new LazyTalonFX(Drivetrain.BACK_LEFT_MODULE_DRIVE_MOTOR);
    // m_backRightDrive = new LazyTalonFX(Drivetrain.BACK_RIGHT_MODULE_DRIVE_MOTOR);

    Mk4ModuleConfiguration swerveConfig = new Mk4ModuleConfiguration();
    swerveConfig.setDriveCurrentLimit(20);
    swerveConfig.setSteerCurrentLimit(20);
    // swerveConfig
    m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            swerveConfig,
            Mk4SwerveModuleHelper.GearRatio.L2,
            Drivetrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            Drivetrain.FRONT_LEFT_MODULE_STEER_MOTOR,
            Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER,
            Drivetrain.FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            swerveConfig,
            Mk4SwerveModuleHelper.GearRatio.L2,
            Drivetrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            Drivetrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
            Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
            Drivetrain.FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            swerveConfig,
            Mk4SwerveModuleHelper.GearRatio.L2,
            Drivetrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
            Drivetrain.BACK_LEFT_MODULE_STEER_MOTOR,
            Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER,
            Drivetrain.BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            swerveConfig,
            Mk4SwerveModuleHelper.GearRatio.L2,
            Drivetrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            Drivetrain.BACK_RIGHT_MODULE_STEER_MOTOR,
            Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER,
            Drivetrain.BACK_RIGHT_MODULE_STEER_OFFSET
    );

  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
        // offset = m_navx.getFusedHeading();
        offset = m_navx.getYaw() + 180;
        resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
        
  }
  public void zeroGyroscopeAutonLeft() {
    // offset = m_navx.getFusedHeading();
    offset = m_navx.getYaw() - 32.3;
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    
}
public void zeroGyroscopeAutonMiddle() {
  // offset = m_navx.getFusedHeading();
  offset = m_navx.getYaw() + 35.3 - Math.abs(RobotContainer.AutonMiddleOffsetAT- RobotContainer.AutonMiddleOffsetBT); //get new Theta(currently 35.3)
  resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  
}
public void zeroGyroscopeAutonRight() {
  // offset = m_navx.getFusedHeading();
  offset = m_navx.getYaw() + 80.3;
  resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  
}

  public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(Math.toDegrees(getNavHeading()));
  }

  public double getNavHeading(){
    // double angle = m_navx.getFusedHeading() - offset;
    double angle = m_navx.getYaw() + 180 - offset;
    angle = angle + 90 + 360;
    angle = angle % 360;
    // SmartDashboard.putNumber("navX Angle", angle);
    SmartDashboard.putNumber("navX Offset", offset);
    return Math.toRadians(angle);
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(pose, getRotation2d());
  }

  

public void drive(ChassisSpeeds chassisSpeeds) {
    //Check for angle tracking conditions
    if(Math.abs(chassisSpeeds.omegaRadiansPerSecond) < 0.01 && trackingState == false){
      trackingAngle = getNavHeading();
      trackingState = true;
    }else if(Math.abs(chassisSpeeds.omegaRadiansPerSecond) >= 0.01 || Math.abs(chassisSpeeds.vxMetersPerSecond) + Math.abs(chassisSpeeds.vyMetersPerSecond) <= 0.05 ){
      trackingState = false;
    }
    //If we have 0 input angular velocity, add a pid gain to get to the last angle detected
    m_chassisSpeeds = chassisSpeeds;
    double maxTrackingError = Math.toRadians(10); //If we get pushed too far off we don't want to keep engaging the turning pid
    double trackingError = trackingAngle - getNavHeading();
    
    if(trackingError > maxTrackingError){
      trackingState = false;
    }else if(trackingState){//Add Angular pid to correct for drive drift
      double kP = 0.025;
      m_chassisSpeeds.omegaRadiansPerSecond = kP * (trackingError);
    }
  }

  public void setModuleStates(SwerveModuleState[] setState){
    m_frontLeftModule.set(setState[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, setState[0].angle.getRadians());
    m_frontRightModule.set(setState[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, setState[1].angle.getRadians());
    m_backLeftModule.set(setState[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, setState[2].angle.getRadians());
    m_backRightModule.set(setState[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, setState[3].angle.getRadians());

    // DataLogManager.start();

    // // Set up custom log entries
    // DataLog log = DataLogManager.getLog();
    // myBooleanLog = new BooleanLogEntry(log, "/my/boolean");
    // myDoubleLog = new DoubleLogEntry(log, "/my/double");
    // myStringLog = new StringLogEntry(log, "/my/string");

    odometer.update(
      getRotation2d(),
      new SwerveModuleState(m_frontLeftModule.getDriveVelocity(), new Rotation2d(m_frontLeftModule.getSteerAngle())),
      new SwerveModuleState(m_frontRightModule.getDriveVelocity(), new Rotation2d(m_frontRightModule.getSteerAngle())),
      new SwerveModuleState(m_backLeftModule.getDriveVelocity(), new Rotation2d(m_backLeftModule.getSteerAngle())),
      new SwerveModuleState(m_backRightModule.getDriveVelocity(), new Rotation2d(m_backRightModule.getSteerAngle()))
    );
  }

  @Override
  public void periodic() {
    previousState = currentState;
    currentState = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(currentState, MAX_VELOCITY_METERS_PER_SECOND);
    
    if(previousState[0] == null || previousState[1] == null || previousState[2] == null || previousState[3] == null){
      previousState = currentState;
    }

    double joystickCenterState = 0;
    boolean joystickCentered = true;
    for(int i = 0; i < currentState.length && joystickCentered; i++){
      if(Math.abs(currentState[i].angle.getRadians() - joystickCenterState) > 0.001) joystickCentered = false;
    }
    
    if(joystickCentered){
      currentState[0].angle = previousState[0].angle;
      currentState[1].angle = previousState[1].angle;
      currentState[2].angle = previousState[2].angle;
      currentState[3].angle = previousState[3].angle;
    }

    setModuleStates(currentState);

    SmartDashboard.putNumber("Tracking Error", 180 /Math.PI * trackingAngle - getNavHeading());
    SmartDashboard.putNumber("Tracking Angle", 180 /Math.PI * trackingAngle);
    SmartDashboard.putNumber("Robot Angle", 180 /Math.PI * getNavHeading());
    SmartDashboard.putBoolean("Is Tracking", trackingState);
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    SmartDashboard.putString("Robot Rotation", getPose().getRotation().toString()); //271.33
    SmartDashboard.putNumber("Front Right Steer", m_frontRightModule.getSteerAngle()*180/Math.PI);
    SmartDashboard.putNumber("Front Left Steer", m_frontLeftModule.getSteerAngle()*180/Math.PI);
    // double csx = m_kinematics.toChassisSpeeds(currentState).vxMetersPerSecond;
    // SmartDashboard.putNumber("X Velocity", csx);
    // double csy = m_kinematics.toChassisSpeeds(currentState).vyMetersPerSecond;
    // SmartDashboard.putNumber("Y Velocity", csy);
    // double cst = m_kinematics.toChassisSpeeds(currentState).omegaRadiansPerSecond;
    // SmartDashboard.putNumber("Theta Velocity", cst);
    // SmartDashboard.putString("Wheel Velocity Real fL", ""+m_frontLeftSteer.getSelectedSensorVelocity());
  }
}
