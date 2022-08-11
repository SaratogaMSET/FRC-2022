

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;


    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Drivetrain.angleEnableCurrentLimit, 
            Constants.Drivetrain.angleContinuousCurrentLimit, 
            Constants.Drivetrain.anglePeakCurrentLimit, 
            Constants.Drivetrain.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.Drivetrain.kPThetaController;
        swerveAngleFXConfig.slot0.kI = 0.0;
        swerveAngleFXConfig.slot0.kD = 0.0;
        swerveAngleFXConfig.slot0.kF = 0.0;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Drivetrain.driveEnableCurrentLimit, 
            Constants.Drivetrain.driveContinuousCurrentLimit, 
            Constants.Drivetrain.drivePeakCurrentLimit, 
            Constants.Drivetrain.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.Drivetrain.kPXController;
        swerveDriveFXConfig.slot0.kI = 0.0;
        swerveDriveFXConfig.slot0.kD = 0.0;
        swerveDriveFXConfig.slot0.kF = 0.0;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = Constants.Drivetrain.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.Drivetrain.closedLoopRamp;

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Drivetrain.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        

    }

}