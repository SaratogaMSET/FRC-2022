package frc.robot.commands;
 
import com.ctre.phoenix.motorcontrol.ControlMode;
 
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.Robot;
 
public class ShooterCommand extends CommandBase {
 private final ShooterSubsystem m_shooter;
 // private final ShooterState current;
 private ShooterState target;
 private double rpm;
 private int leadscrewPos;
 private boolean isFinished;
 private VisionSubsystem m_vision;
  /** Creates a new ShooterCommand. */
 public ShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision, ShooterState target) {
   this.m_shooter = shooter;
   // this.current = RobotState.shooterState;
   this.target = target;
   this.rpm = m_shooter.getShooterStateRPM(target);
   this.leadscrewPos = m_shooter.getLeadscrewStatePosition(target);
  
   // Use addRequirements() here to declare subsystem dependencies.
   addRequirements(m_shooter);
 }
 
 public ShooterCommand(ShooterSubsystem shooter, VisionSubsystem vision) {
   addRequirements(shooter);
   addRequirements(vision);
 
 
   this.m_shooter = shooter;
   this.m_vision = vision;
  

 }
 
 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
   isFinished = false;
 
   this.target = m_vision.getShooterStateFromDistance();
   this.rpm = m_shooter.getShooterStateRPM(target);
   this.leadscrewPos = (m_shooter.getLeadscrewStatePosition(target));

 }
 
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
   m_shooter.setLeadscrew(ControlMode.PercentOutput, 0);
 }
 
 // Returns true when the command should end.
 @Override
 public boolean isFinished() {
   SmartDashboard.putBoolean("ShooterCommandState", m_shooter.withinTolerance(target));
   if (m_shooter.withinTolerance(target)) {
     return true;
   }
   return isFinished;
 }
}


