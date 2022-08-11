package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.util.CTREModuleState;
import frc.robot.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;


public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Drivetrain.driveKS, Constants.Drivetrain.driveKV, Constants.Drivetrain.driveKA);

    public SwerveModule(int moduleNumber){
        this.moduleNumber = moduleNumber;
        if(this.moduleNumber == 0){
            angleOffset = Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_OFFSET;
        }
        else if(this.moduleNumber == 1){
            angleOffset = Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_OFFSET;
        }
        else if(this.moduleNumber == 2){
            angleOffset = Constants.Drivetrain.BACK_LEFT_MODULE_STEER_OFFSET;
        }
        else{
            angleOffset = Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_OFFSET;
        }
        
        /* Angle Encoder Config */
        if(this.moduleNumber == 0){
            angleEncoder = new CANCoder(Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_ENCODER);
            configAngleEncoder();
            mAngleMotor = new TalonFX(Constants.Drivetrain.FRONT_LEFT_MODULE_STEER_MOTOR);
            configAngleMotor();
            mDriveMotor = new TalonFX(Constants.Drivetrain.FRONT_LEFT_MODULE_DRIVE_MOTOR);
            configDriveMotor();
        }
        else if(this.moduleNumber == 1){
            angleEncoder = new CANCoder(Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_ENCODER);
            configAngleEncoder();
            mAngleMotor = new TalonFX(Constants.Drivetrain.FRONT_RIGHT_MODULE_STEER_MOTOR);
            configAngleMotor();
            mDriveMotor = new TalonFX(Constants.Drivetrain.FRONT_RIGHT_MODULE_DRIVE_MOTOR);
            configDriveMotor();
        }
        else if(this.moduleNumber == 2){
            angleEncoder = new CANCoder(Constants.Drivetrain.BACK_LEFT_MODULE_STEER_ENCODER);
            configAngleEncoder();
            mAngleMotor = new TalonFX(Constants.Drivetrain.BACK_LEFT_MODULE_STEER_MOTOR);
            configAngleMotor();
            mDriveMotor = new TalonFX(Constants.Drivetrain.BACK_LEFT_MODULE_DRIVE_MOTOR);
            configDriveMotor();
        }
        else{
            angleEncoder = new CANCoder(Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_ENCODER);
            configAngleEncoder();
            mAngleMotor = new TalonFX(Constants.Drivetrain.BACK_RIGHT_MODULE_STEER_MOTOR);
            configAngleMotor();
            mDriveMotor = new TalonFX(Constants.Drivetrain.BACK_RIGHT_MODULE_DRIVE_MOTOR);
            configDriveMotor();
        }
        

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Drivetrain.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI, Constants.Drivetrain.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Drivetrain.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.Drivetrain.angleGearRatio)); 
        lastAngle = angle;
    }

    private void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.Drivetrain.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig); //error reason, it was called from robot.java but it is unable to call it correctly
    }

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Drivetrain.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Drivetrain.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Drivetrain.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Drivetrain.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState(){
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), SdsModuleConfigurations.MK4_L2.getWheelDiameter()*Math.PI, Constants.Drivetrain.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Drivetrain.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }
    
}