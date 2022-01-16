package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Feeder extends SubsystemBase {
    public static enum State {
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

    public void setManual(boolean manual){
        this.manual = manual;
    }

    public State getManual(){
        if(manual) 
            return State.MANUAL;
        else 
            return State.AUTO;
    }

    public void setTopMotorVelocity(double velocity){
        topMotor.set(ControlMode.PercentOutput, velocity);
    }

    public void setBottomMotorVelocity(double velocity){
        bottomMotor.set(ControlMode.PercentOutput, velocity);
    }

    public State updateState() {
        double bottomVelocity = bottomMotor.getMotorOutputPercent();
        double topVelocity = topMotor.getMotorOutputPercent();
        State state;

        if(bottomVelocity > 0 && topVelocity > 0)
            return State.FEED;
        else if(bottomVelocity < 0 && topVelocity < 0)
            return State.REVERSE;
        else
            return State.IDLE;
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run

}
}
