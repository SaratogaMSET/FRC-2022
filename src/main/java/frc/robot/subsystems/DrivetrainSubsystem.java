// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.VisionSubsystem.VisionState;

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

    // TODO tune UKF
    private final SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(
        new Rotation2d(0),
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), 
        m_kinematics, 
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
        new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.02),
        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
    );

    private final VisionSubsystem m_vision;

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

    BooleanLogEntry myBooleanLog;
    DoubleLogEntry myDoubleLog;
    StringLogEntry myStringLog;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DrivetrainSubsystem() {

        m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                Drivetrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                Drivetrain.FRONT_LEFT_MODULE_STEER_MOTOR,
                Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER,
                Drivetrain.FRONT_LEFT_MODULE_STEER_OFFSET
        );

        m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                Drivetrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                Drivetrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
                Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
                Drivetrain.FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                Drivetrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Drivetrain.BACK_LEFT_MODULE_STEER_MOTOR,
                Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER,
                Drivetrain.BACK_LEFT_MODULE_STEER_OFFSET
        );

        m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                Drivetrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                Drivetrain.BACK_RIGHT_MODULE_STEER_MOTOR,
                Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER,
                Drivetrain.BACK_RIGHT_MODULE_STEER_OFFSET
        );

        m_vision = null;
    }

    public DrivetrainSubsystem(VisionSubsystem vision) {

        m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                Drivetrain.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                Drivetrain.FRONT_LEFT_MODULE_STEER_MOTOR,
                Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER,
                Drivetrain.FRONT_LEFT_MODULE_STEER_OFFSET
        );

        m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                Drivetrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                Drivetrain.FRONT_RIGHT_MODULE_STEER_MOTOR,
                Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER,
                Drivetrain.FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                Drivetrain.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Drivetrain.BACK_LEFT_MODULE_STEER_MOTOR,
                Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER,
                Drivetrain.BACK_LEFT_MODULE_STEER_OFFSET
        );

        m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                Drivetrain.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                Drivetrain.BACK_RIGHT_MODULE_STEER_MOTOR,
                Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER,
                Drivetrain.BACK_RIGHT_MODULE_STEER_OFFSET
        );

        m_vision = vision;
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        offset = m_navx.getYaw() + 180;
        resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    }

    public Rotation2d getRotation2d() {
            return Rotation2d.fromDegrees(Math.toDegrees(getNavHeading()));
    }

    public double getNavHeading(){
        double angle = m_navx.getYaw() + 180 - offset;
        angle = angle + 90 + 360;
        angle = angle % 360;
        SmartDashboard.putNumber("navX Offset", offset);
        return Math.toRadians(angle);
    }

    public Pose2d getPose() {
        return odometer.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        // FIXME "You NEED to reset your encoders (to zero) when calling this method" - resetPosition
        odometer.resetPosition(pose, getRotation2d());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    public void setModuleStates(SwerveModuleState[] setState){
        m_frontLeftModule.set(setState[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, setState[0].angle.getRadians());
        m_frontRightModule.set(setState[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, setState[1].angle.getRadians());
        m_backLeftModule.set(setState[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, setState[2].angle.getRadians());
        m_backRightModule.set(setState[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, setState[3].angle.getRadians());

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

        if (m_vision != null) {
            if (m_vision.updateVisionState() == VisionState.TARGET_VISIBLE) {
                odometer.addVisionMeasurement(
                    m_vision.getCamPose(), 
                    Timer.getFPGATimestamp() // FIXME we need to account for camera latency
                );
            }
        }

        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putString("Robot Rotation", getPose().getRotation().toString());
    }
}
