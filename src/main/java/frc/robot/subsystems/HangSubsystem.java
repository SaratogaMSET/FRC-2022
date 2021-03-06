package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangSubsystem extends SubsystemBase {
    public TalonFX rightHangMotor;
    public TalonFX leftHangMotor;
    public WPI_TalonFX encoderRight;
    public WPI_TalonFX encoderLeft; 
    public DigitalInput hangRightLimitSwitch; 
    public DigitalInput hangLeftLimitSwitch;
    public Solenoid hangSolenoid;

    public boolean triggeredLeftSwitch = true;
    public boolean triggeredRightSwitch = true;
    public boolean triggeredLeftSoftStop = true;
    public boolean triggeredRightSoftStop = true;

    public boolean maxHeightLeft;
    public boolean maxHeightRight;
    public boolean halfHeightLeft;
    public boolean halfHeightRight;

    public boolean hangPosition = false;

    public static final boolean POSITION_FORWARD = false;
    public static final boolean POSITION_NORMAL = true;

    // private final PIDController pid;

    public HangSubsystem() {
        rightHangMotor = new TalonFX(Constants.HangConstants.HANG_RIGHT_MOTOR);
        leftHangMotor = new TalonFX(Constants.HangConstants.HANG_LEFT_MOTOR);
        rightHangMotor.setNeutralMode(NeutralMode.Brake);
        leftHangMotor.setNeutralMode(NeutralMode.Brake);

        encoderRight = new WPI_TalonFX(Constants.HangConstants.HANG_RIGHT_MOTOR);
        encoderLeft = new WPI_TalonFX(Constants.HangConstants.HANG_LEFT_MOTOR);

        hangRightLimitSwitch = new DigitalInput(Constants.HangConstants.RIGHT_HANG_LIMIT_SWITCH);
        hangLeftLimitSwitch = new DigitalInput(Constants.HangConstants.LEFT_HANG_LIMIT_SWITCH);
    
        hangSolenoid = new Solenoid(2, PneumaticsModuleType.REVPH, Constants.HangConstants.HANG_SOLENOID);

        rightResetEncoders();
        leftResetEncoders();
    }

    public void setHangLeftSpeed(double speed) {
        leftHangMotor.set(ControlMode.PercentOutput, speed);
    }
    public void setHangRightSpeed(double speed) {
        rightHangMotor.set(ControlMode.PercentOutput, -speed);
    }


    public void rightResetEncoders() {
        if(hangRightLimitSwitch.get())
            encoderRight.setSelectedSensorPosition(0);
    }
    public void leftResetEncoders() {
        if(hangLeftLimitSwitch.get())
            encoderLeft.setSelectedSensorPosition(0);
    }

    public double getRightEncoderValue() {
        return -encoderRight.getSelectedSensorPosition();
    }
    public double getLeftEncoderValue() {
        return encoderLeft.getSelectedSensorPosition();
    }

    public void deployHang() {
        hangPosition = !hangPosition;
        hangSolenoid.set(hangPosition);
    }

    public boolean isHangDeployed() {
        return hangSolenoid.get();
    }

    public double metersToNativeUnits(double positionMeters){
        double wheelRotations = positionMeters/(2 * Math.PI * 0.5);
        double motorRotations = wheelRotations * (54/8 * 1.5); 
        double sensorCounts = (double)(motorRotations * 2048);
        return sensorCounts;
    }

    public double nativeUnitsToMeters(double sensorCounts){
        double motorRotations = (double)sensorCounts / 2048;
        double wheelRotations = motorRotations / (54/8 * 1.5);
        double positionMeters = wheelRotations * (2 * Math.PI * 0.5);
        return positionMeters;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Hang Current", leftHangMotor.getStatorCurrent());
        SmartDashboard.putNumber("Right Hang Current", rightHangMotor.getStatorCurrent());
        // if(hangRightLimitSwitch.get()){
        //     rightResetEncoders();
        // }

        // if(hangLeftLimitSwitch.get()){
        //     leftResetEncoders();
        // }
    }
}