package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

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

    public void setTopMotorVelocity(double velocity){
        topMotor.set(ControlMode.PercentOutput, velocity);
    }

    public void setBottomMotorVelocity(double velocity){
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

    @Override
    public void periodic() {
      // This method will be called once per scheduler run

}
}
