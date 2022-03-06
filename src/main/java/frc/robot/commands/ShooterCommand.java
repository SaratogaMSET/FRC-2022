// package frc.robot.commands;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.FeederSubsystem;
// // import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;


// public class ShooterCommand extends CommandBase {
//     private final ShooterSubsystem m_shooter;
//     private final FeederSubsystem m_feeder;


//     public ShooterCommand(FeederSubsystem feeder, ShooterSubsystem shooterSub) {
//         m_feeder = feeder;
//         m_shooter = shooterSub;
//         addRequirements(shooterSub);
//         addRequirements(m_feeder);
//     }
//     @Override
//     public void initialize(){
//         m_shooter.run(0.5);
//         m_feeder.setRawShooterFeeder(.3);
//     }
//     @Override
//     public void execute() {
        
//     }
//     @Override
//     public void end(boolean interrupted) {
//         m_shooter.run(0.0);
//     }

//     public void shoot(double runInitial, double timeDelay, double voltageIncrease, double duration){
//         double voltage1 = m_shooter.shooterMotor.getBusVoltage();
//         double voltage2 = m_shooter.shooterMotor2.getBusVoltage();
//         m_shooter.run(runInitial);
//         //TimeDelayInSeconds
//         double timeStart = Timer.getFPGATimestamp();
        
//         double timeStartIncrease = timeStart + timeDelay;
//         double timeEndIncrease = timeStartIncrease + duration;

//         if(Timer.getFPGATimestamp() >= timeStartIncrease && Timer.getFPGATimestamp() <= timeEndIncrease){
//             m_shooter.shooterMotor.setVoltage(voltage1 + voltageIncrease);
//             m_shooter.shooterMotor2.setVoltage(voltage2 + voltageIncrease);
//         }
// }
// }


package frc.robot.commands;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.ShooterSubsystem.ShooterStateAngle;
import frc.robot.subsystems.VisionSubsystem;

public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem m_shooter;
    // private final ShooterState current;
    private ShooterState target;
    private double rpm;
    // private int leadscrewPos;
    private boolean isFinished;
    private VisionSubsystem m_vision;
    private ShooterStateAngle angleState;



    /** Creates a new ShooterCommand. */
    public ShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision, ShooterState target) {
        this.m_shooter = shooter;
        // this.current = RobotState.shooterState;
        this.target = target;
        this.rpm = m_shooter.getShooterStateRPM(target);
        //   this.leadscrewPos = m_shooter.getLeadscrewStatePosition(target);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_shooter);
    }


    public ShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision, ShooterStateAngle angleState) {
        addRequirements(shooter);
        addRequirements(vision);
        this.m_shooter = shooter;
        this.m_vision = vision;
        this.angleState = angleState;
        
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isFinished = false;
        this.target = m_vision.getShooterStateFromDistance();
        this.rpm = m_shooter.getShooterStateRPM(target);
        this.angleState = m_shooter.getAngleState(target);
        m_shooter.setRPM(rpm);


        if (angleState == ShooterStateAngle.FOURZERO)
            m_shooter.deploy(true);
        else
            m_shooter.deploy(false);
    }

    //   this.leadscrewPos = (m_shooter.getLeadscrewStatePosition(target));
    

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            m_shooter.set(ControlMode.PercentOutput, 0);
        }
    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //   SmartDashboard.putBoolean("ShooterCommandState", m_shooter.withinTolerance(target));
        //   if (m_shooter.withinTolerance(target)) {
        //     return true;
        //   }
        return isFinished;
    }
}