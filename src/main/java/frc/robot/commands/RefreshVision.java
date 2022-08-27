package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class RefreshVision extends CommandBase{
    private VisionSubsystem m_visionSubsystem;

    public RefreshVision(VisionSubsystem m_visionSubsystem){
        this.m_visionSubsystem = m_visionSubsystem;
        addRequirements(m_visionSubsystem);
    }
    @Override
    public void initialize() {
    
        m_visionSubsystem.getDistanceFromTarget();
    }
    @Override
    public boolean isFinished(){
        return true;
    }
}
