// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Intake.IntakeState;
// import frc.robot.commands.FeederCommand;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import frc.robot.subsystems.Feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake m_intake = new Intake();
  private final Feeder m_feeder = new Feeder();
  private final RobotState m_robotState = new RobotState();

  private final Joystick driverVertical, driverHorizontal, gamepad;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driverHorizontal = new Joystick(Constants.OIConstants.JOYSTICK_DRIVE_HORIZONTAL);
    driverVertical = new Joystick(Constants.OIConstants.JOYSTICK_DRIVE_VERTICAL);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //******************** TELEOP ********************/

    new JoystickButton(driverVertical, 1).whileHeld(
        new RunIntake(m_intake, IntakeState.INTAKE, 0.1)  
    );

    new JoystickButton(driverHorizontal, 1).whileHeld(
        new RunIntake(m_intake, IntakeState.OUTTAKE, 0.1)
    );

    new JoystickButton(driverVertical, 1).or(new JoystickButton(driverHorizontal, 1)).whenInactive(
      new RunIntake(m_intake, IntakeState.FLIP_UP)
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new RunIntake(m_intake, IntakeState.INTAKE, 0.1);
    //return new FeederCommand(m_feeder, 0.1, 0.1, m_robotState);
  }

  public void updateRobotState() {
    m_robotState.intakeState = m_intake.updateState();
    // m_robotState.feederState = m_feeder.updateState();
    
    // SmartDashboard.putString("Feeder State", m_robotState.feederState.toString());
    SmartDashboard.putString("Feeder Can Run", Boolean.toString(m_robotState.canRunFeeder()));
    SmartDashboard.putString("Intake State", m_robotState.intakeState.toString());
    SmartDashboard.putString("Intake Can Run", Boolean.toString(m_robotState.canDeployIntake()));

  }

  // public Command getTestCommand() {
  //   //return new SequentialCommandGroup(new InstantCommand(() -> m_intake.deploy(true)) 
  //   //new ParallelCommandGroup(new RunIntake(m_intake).withTimeout(1), new FeederCommand(m_feeder, 0, 1, m_robotState)).withTimeout(2.0));
  // }
}
