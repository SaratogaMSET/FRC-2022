package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.drivers.LazyTalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;


public class WestCoastDriveTest extends SubsystemBase{

    private WPI_TalonFX m_rightEncoder = new WPI_TalonFX(Constants.Drivetrain.BACK_RIGHT_MODULE_DRIVE_MOTOR);
    private WPI_TalonFX m_rightMotor = new WPI_TalonFX(Constants.Drivetrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR);
    private WPI_TalonFX m_leftEncoder = new WPI_TalonFX(Constants.Drivetrain.BACK_LEFT_MODULE_DRIVE_MOTOR);
    private WPI_TalonFX m_leftMotor = new WPI_TalonFX(Constants.Drivetrain.FRONT_LEFT_MODULE_DRIVE_MOTOR);

    private LazyTalonFX m_frontLeftSteer;
    private LazyTalonFX m_frontRightSteer;
    private LazyTalonFX m_backLeftSteer;
    private LazyTalonFX m_backRightSteer;

    private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_leftEncoder, m_leftMotor);

    private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_rightEncoder, m_rightMotor);
  

    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    // The gyro sensor
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    /** Creates a new DriveSubsystem. */
    public WestCoastDriveTest() {

        m_frontLeftSteer = new LazyTalonFX(Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_MOTOR);
        m_frontRightSteer = new LazyTalonFX(Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_MOTOR);
        m_backLeftSteer = new LazyTalonFX(Constants.Drivetrain.BACK_LEFT_MODULE_STEER_MOTOR);
        m_backRightSteer = new LazyTalonFX(Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_MOTOR);

        m_frontLeftSteer.setNeutralMode(NeutralMode.Brake);
        m_frontRightSteer.setNeutralMode(NeutralMode.Brake);
        m_backLeftSteer.setNeutralMode(NeutralMode.Brake);
        m_backRightSteer.setNeutralMode(NeutralMode.Brake);


        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        m_rightMotors.setInverted(true);

        // Sets the distance per pulse for the encoders
        // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    }

    @Override
    public void periodic() {
        m_frontLeftSteer.set(ControlMode.PercentOutput, 0);
        m_frontRightSteer.set(ControlMode.PercentOutput, 0);
        m_backLeftSteer.set(ControlMode.PercentOutput, 0);
        m_backRightSteer.set(ControlMode.PercentOutput, 0);
        // Update the odometry in the periodic block
        m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getSelectedSensorPosition()/(2048.00*15.50), m_rightEncoder.getSelectedSensorPosition()/(2048.00*15.50));
    }

    /**
    * Returns the currently-estimated pose of the robot.
    *
    * @return The pose.
    */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
    * Returns the current wheel speeds of the robot.
    *
    * @return The current wheel speeds.
    */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getSelectedSensorVelocity()/(2048.00*15.50),
        m_leftEncoder.getSelectedSensorVelocity()/(2048.00*15.50));
    }

    /**
    * Resets the odometry to the specified pose.
    *
    * @param pose The pose to which to set the odometry.
    */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    /**
    * Drives the robot using arcade controls.
    *
    * @param fwd the commanded forward movement
    * @param rot the commanded rotation
    */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    /**
    * Controls the left and right sides of the drive directly with voltages.
    *
    * @param leftVolts the commanded left output
    * @param rightVolts the commanded right output
    */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(rightVolts);
        m_drive.feed();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_leftEncoder.setSelectedSensorPosition(0);
        m_rightEncoder.setSelectedSensorPosition(0);
    }

    /**
    * Gets the average distance of the two encoders.
    *
    * @return the average of the two encoder readings
    */
    public double getAverageEncoderDistance() {
        return (m_leftEncoder.getSelectedSensorPosition()/(2048.00*15.50) +
            m_rightEncoder.getSelectedSensorPosition()/(2048.00*15.50)) / 2.0;
    }

    /**
    * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
    *
    * @param maxOutput the maximum output to which the drive will be constrained
    */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
    * Returns the heading of the robot.
    *
    * @return the robot's heading in degrees, from -180 to 180
    */
    public double getHeading() {
        return m_gyro.getFusedHeading()-180;
    }

    /**
    * Returns the turn rate of the robot.
    *
    * @return The turn rate of the robot, in degrees per second
    */
    public double getTurnRate() {
        return -m_gyro.getRate();
    }
}
