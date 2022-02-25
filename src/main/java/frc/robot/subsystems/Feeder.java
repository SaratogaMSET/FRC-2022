package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class Feeder extends SubsystemBase {
    public static enum FeederState {
        FEED,
        REVERSE,
        IDLE,
        MANUAL, 
        AUTO
    }

    private boolean manual;
    private final TalonFX bottomMotor, topMotor;
    private final DigitalInput[] irGates;
    
    public Feeder () {
        bottomMotor = new TalonFX(Constants.FeederConstants.BOTTOM_MOTOR);
        topMotor = new TalonFX(Constants.FeederConstants.TOP_MOTOR);

        irGates = new DigitalInput[Constants.FeederConstants.IR_GATES.length];
        for(int i : Constants.FeederConstants.IR_GATES){
            irGates[i] = new DigitalInput(Constants.FeederConstants.IR_GATES[i]);
        }
    }

    public boolean getIRGate(int index){
        return irGates[index].get();
    }

    public void setManual(boolean manual){
        this.manual = manual;
    }

    public FeederState getManual(){
        if(manual) 
            return FeederState.MANUAL;
        else 
            return FeederState.AUTO;
    }

    public void setTopMotor(double velocity){
        topMotor.set(ControlMode.PercentOutput, velocity);
    }

    public void setBottomMotor(double velocity){
        bottomMotor.set(ControlMode.PercentOutput, velocity);
    }

    public FeederState updateState() {
        double bottomVelocity = bottomMotor.getMotorOutputPercent();
        double topVelocity = topMotor.getMotorOutputPercent();
        
        if(bottomVelocity > topVelocity)
            return FeederState.AUTO;
        else if(bottomVelocity > 0 && topVelocity > 0)
            return FeederState.FEED;
        else if(bottomVelocity < 0 && topVelocity < 0)
            return FeederState.REVERSE;
        else
            return FeederState.IDLE;
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
