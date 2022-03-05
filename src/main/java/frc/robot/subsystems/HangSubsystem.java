package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HangSubsystem extends SubsystemBase {
    private final TalonFX rightHangMotor;
    private final TalonFX leftHangMotor;
    private final WPI_TalonFX encoderRight;
    private final WPI_TalonFX encoderLeft; 

    public boolean triggeredLeftSwitch;
    public boolean triggeredRightSwitch;

    public HangSubsystem() {
        rightHangMotor = new TalonFX(Constants.HangConstants.HANG_RIGHT_MOTOR);
        leftHangMotor = new TalonFX(Constants.HangConstants.HANG_LEFT_MOTOR);
        rightHangMotor.setNeutralMode(NeutralMode.Brake);
        leftHangMotor.setNeutralMode(NeutralMode.Brake);

        encoderRight = new WPI_TalonFX(Constants.HangConstants.HANG_RIGHT_MOTOR);
        encoderLeft = new WPI_TalonFX(Constants.HangConstants.HANG_LEFT_MOTOR);
    }

    public void hitLeft() {
        triggeredLeftSwitch = true;
    }

    public void hitRight() {
        triggeredRightSwitch = true;
    }

    public void setHangLeftSpeed(double speed) {
        leftHangMotor.set(ControlMode.PercentOutput, speed);
    }
    public void setHangRightSpeed(double speed) {
        rightHangMotor.set(ControlMode.PercentOutput, -speed);
    }

    public double getLeftEncoder() {
        SmartDashboard.putString("encoder left ", encoderLeft.getSelectedSensorPosition() + "");
        return encoderLeft.getSelectedSensorPosition();
    }

    public double getRightEncoder() {
        SmartDashboard.putString("encoder right ", encoderRight.getSelectedSensorPosition() + "");
        return encoderRight.getSelectedSensorPosition();
    }


    public void rightResetEncoders() {
        encoderRight.setSelectedSensorPosition(0);

        //SmartDashboard.putString("encoder right after reset ", encoderRight.getSelectedSensorPosition() + "");
    }
    public void leftResetEncoders() {
        encoderLeft.setSelectedSensorPosition(0);

        //SmartDashboard.putString("encoder left after reset ", encoderLeft.getSelectedSensorPosition() + "");
    }

    public double getRightEncoderValue() {
        return encoderRight.getSelectedSensorPosition();
    }
    public double getLeftEncoderValue() {
        return encoderLeft.getSelectedSensorPosition();
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
}