package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class Feeder extends SubsystemBase {
    public static enum FeederState {
        INTAKE,
        OUTTAKE,
        IDLE
    }

    private boolean manual;
    private final TalonFX bottomMotor, topMotor;
    private final DigitalInput shooterGate, intakeGate;
    private boolean inIntakeFeeder, inShooterFeeder, runIntakeFeeder, runShooterFeeder;
    private double intakeFeederVelocity, shooterFeederVelocity;
    
    
    public Feeder () {
        bottomMotor = new TalonFX(Constants.FeederConstants.BOTTOM_MOTOR);
        topMotor = new TalonFX(Constants.FeederConstants.TOP_MOTOR);
        shooterGate = new DigitalInput(0);
        intakeGate = new DigitalInput(1);
        inIntakeFeeder = false;
        inShooterFeeder = false;
        runIntakeFeeder = false; 
        runShooterFeeder = false;
    }

  
    public void updateGates() {
      inIntakeFeeder = !intakeGate.get();
      inShooterFeeder = !shooterGate.get();
      SmartDashboard.putBoolean("Ball in Intake", inIntakeFeeder);
      SmartDashboard.putBoolean("Ball in Shooter", inShooterFeeder);
      
      if (shooterFeederVelocity < 0.0 || intakeFeederVelocity < 0.0) {
        runIntakeFeeder = true;
        runShooterFeeder = true;
      }
      else if (inIntakeFeeder && inShooterFeeder) {
        runIntakeFeeder = false;
        runShooterFeeder = false;
      } else if (inIntakeFeeder) {
        runIntakeFeeder = true;
        runShooterFeeder = true;
      } else if (inShooterFeeder) {
        runIntakeFeeder = true;
        runShooterFeeder = false;
      } else {
        runIntakeFeeder = true;
        runShooterFeeder = true;
      }
    }

    public FeederState getState() {
      double measuredShooterFeeder = topMotor.getMotorOutputPercent();
      double measuredIntakeFeeder = bottomMotor.getMotorOutputPercent();
      if (measuredShooterFeeder > 0.0 || measuredIntakeFeeder > 0.0) return FeederState.INTAKE;
      else if (measuredShooterFeeder < 0.0 || measuredIntakeFeeder < 0.0) return FeederState.OUTTAKE;
      else return FeederState.IDLE;
    }
  
    public void setTopMotor(double velocity) {
      shooterFeederVelocity = velocity;
      updateGates();
      if (runShooterFeeder)
        topMotor.set(ControlMode.PercentOutput, velocity);
      else
        topMotor.set(ControlMode.PercentOutput, 0.0);
    }
  
    public void setBottomMotor(double velocity) {
      intakeFeederVelocity = velocity;
      updateGates();
      if (runIntakeFeeder)
        bottomMotor.set(ControlMode.PercentOutput, velocity);
      else
        bottomMotor.set(ControlMode.PercentOutput, 0.0);
    }


    public FeederState updateState() {
        double bottomVelocity = bottomMotor.getMotorOutputPercent();
        double topVelocity = topMotor.getMotorOutputPercent();
        
        if(bottomVelocity > 0 || topVelocity > 0) return FeederState.INTAKE;
        else if(bottomVelocity < 0 || topVelocity < 0) return FeederState.OUTTAKE;
        else return FeederState.IDLE;
    }

    
    public void diagnostics() {
      String topStatus = "Feeder Top Status";
      String bottomStatus = "Feeder Bottom Status";
  
      try {
        setTopMotor(-0.1);
        if (topMotor.getMotorOutputPercent() > -0.08 || topMotor.getMotorOutputPercent() < -0.12) {
          SmartDashboard.putString(topStatus, "Failed");
        } else
          SmartDashboard.putString(topStatus, "Success");
      } catch (Exception e) {
        SmartDashboard.putString(topStatus, "Failed");
      }
  
      try {
        setBottomMotor(0.1);
        if (bottomMotor.getMotorOutputPercent() < 0.08 || bottomMotor.getMotorOutputPercent() > 0.12) {
          SmartDashboard.putString(bottomStatus, "Failed");
        } else
          SmartDashboard.putString(bottomStatus, "Success");
      } catch (Exception e) {
        SmartDashboard.putString(bottomStatus, "Failed");
      }
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run

}
}
