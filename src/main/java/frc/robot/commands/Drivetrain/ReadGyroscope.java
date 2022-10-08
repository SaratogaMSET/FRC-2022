// package frc.robot.commands.Drivetrain;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.DrivetrainSubsystem;

// public class ReadGyroscope extends CommandBase {
//     private DrivetrainSubsystem m_drivetrainSubsystem;
//     boolean b;
//     public ReadGyroscope(DrivetrainSubsystem dt, boolean b) {
//         this.m_drivetrainSubsystem = dt;
//         this.b = b;
//         addRequirements(m_drivetrainSubsystem);
//     }
//     @Override
//     public void execute() {
//         if(b){
//             RobotContainer.AutonMiddleOffsetBT = m_drivetrainSubsystem.m_navx.getYaw();
//         }
//         else{
//             RobotContainer.AutonMiddleOffsetAT = m_drivetrainSubsystem.m_navx.getYaw();
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//     }

//     @Override
//     public boolean isFinished(){
//         return true;
//     }
// }