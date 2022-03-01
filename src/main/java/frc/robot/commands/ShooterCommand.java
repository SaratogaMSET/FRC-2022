// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.commands;
 
import com.ctre.phoenix.motorcontrol.ControlMode;
 
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.Robot;
 
public class ShooterCommand extends CommandBase {
 private final Shooter m_shooter;
 // private final ShooterState current;
 private ShooterState target;
 private double rpm;
 private int leadscrewPos;
 private boolean isFinished;
 private VisionSubsystem m_vision;
  /** Creates a new ShooterCommand. */
 public ShooterCommand(Shooter shooter, VisionSubsystem vision, ShooterState target) {
   this.m_shooter = shooter;
   // this.current = RobotState.shooterState;
   this.target = target;
   this.rpm = m_shooter.getShooterStateRPM(target);
   this.leadscrewPos = m_shooter.getLeadscrewStatePosition(target);
  
   // Use addRequirements() here to declare subsystem dependencies.
   addRequirements(m_shooter);
 }
 
 public ShooterCommand(Shooter shooter, VisionSubsystem vision) {
   addRequirements(shooter);
   addRequirements(vision);
 
 
   this.m_shooter = shooter;
   this.m_vision = vision;
  
 
   // if(vision.getDistance() > 250){
   //   this.leadscrewPos = m_shooter.angleToEncoder(58);
   // } else {
    
   // }
  
  
   // Use addRequirements() here to declare subsystem dependencies.
  
 }
 
 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
   isFinished = false;
 
   this.target = m_vision.getShooterStateFromDistance();
   this.rpm = m_shooter.getShooterStateRPM(target);
   this.leadscrewPos = (m_shooter.getLeadscrewStatePosition(target));
 
   // SmartDashboard.putString("Checkpoint1", Boolean.toString(current == target) + " || " + Boolean.toString(target == ShooterState.IDLE));
   // if (current == target || target == ShooterState.IDLE) {
   //   isFinished = true;
   // } else
   // if (leadscrewPos < 0 || !Robot.canRunLeadscrew()) { // GET THIS CODE WORKING
   //   isFinished = true;
   // }
   // else {
   //   m_shooter.setLeadscrew(leadscrewPos);
   //   m_shooter.setRPM(rpm);
   // }
 
  
 }
 
 // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {
   // SmartDashboard.putNumber("AngleSetpoint", m_vision.returnAngle());
   // SmartDashboard.putString(Shoote, value)
   // if (!Robot.canRunLeadscrew()) { // GET THIS CODE WORKING
   //   // new RunLeadscrewAfterReset(m_shooter, target).schedule();
   //   isFinished = true;
   // }
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

