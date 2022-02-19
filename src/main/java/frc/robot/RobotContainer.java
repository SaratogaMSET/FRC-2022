// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import frc.robot.commands.DefaultDriveCommand;
// import frc.robot.commands.ShooterCommand;
// import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.Feeder;
// import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;
// import frc.robot.subsystems.HangSubsystem;
// import frc.robot.commands.HangForwardCommand;
// import frc.robot.commands.HangReverseCommand;
// import frc.robot.commands.FeederCommand;
import frc.robot.commands.IntakeCommand;
// import frc.robot.commands.PrototypeTestCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  // private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final XboxController m_controller = new XboxController(0);
  // private final Feeder m_feeder;
  private final Intake m_intake;
  private final Compressor m_compressor;
  private final Joystick driverVertical, driverHorizontal;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    
    // m_shooterSubsystem.setDefaultCommand(new ShooterCommand(
    //         m_shooterSubsystem
    // ));

    // m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
    //         m_drivetrainSubsystem,
    //         () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
    //         () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    // ));

    driverVertical = new Joystick(Constants.OIConstants.JOYSTICK_DRIVE_VERTICAL); //send
    driverHorizontal = new Joystick(Constants.OIConstants.JOYSTICK_DRIVE_HORIZONTAL);
    // m_feeder = new Feeder();
    m_intake = new Intake();
    m_compressor = new Compressor(2, PneumaticsModuleType.REVPH);
    // //Configure the button bindings
    // configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    // new Button(m_controller::getBackButton)
    //         // No requirements because we don't need to interrupt anything
    //         .whenPressed(m_drivetrainSubsystem::zeroGyroscope);

    // new JoystickButton (driverVertical, 2)
    //   .whileHeld (new HangForwardCommand(Constants.Hang.HANG_RIGHT_MOTOR, 0.5))
    //   .whenReleased(new HangForwardCommand(Constants.Hang.HANG_RIGHT_MOTOR, 0));
    // new JoystickButton (driverHorizontal, 2)
    //   .whileHeld (new HangForwardCommand(Constants.Hang.HANG_LEFT_MOTOR, 0.5))
    //   .whenReleased(new HangForwardCommand(Constants.Hang.HANG_LEFT_MOTOR, 0));
    // new JoystickButton (driverVertical, 3)
    //   .whileHeld (new HangReverseCommand(Constants.Hang.HANG_RIGHT_MOTOR, 0.5))
    //   .whenReleased(new HangReverseCommand(Constants.Hang.HANG_RIGHT_MOTOR, 0));
    // new JoystickButton (driverHorizontal, 3)
    //   .whileHeld (new HangReverseCommand(Constants.Hang.HANG_LEFT_MOTOR, 0.5))
    //   .whenReleased(new HangReverseCommand(Constants.Hang.HANG_LEFT_MOTOR, 0));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new FeederCommand(m_feeder, FeederState.RUN, 0.0, 0.0);
    return new IntakeCommand(m_intake, IntakeState.RUN, 0.0);
  }

  // private static double deadband(double value, double deadband) {
  //   if (Math.abs(value) > deadband) {
  //     if (value > 0.0) {
  //       return (value - deadband) / (1.0 - deadband);
  //     } else {
  //       return (value + deadband) / (1.0 - deadband);
  //     }
  //   } else {
  //     return 0.0;
  //   }
  // }

  // private static double modifyAxis(double value) {
  //   // Deadband
  //   value = deadband(value, 0.05);

  //   // Square the axis
  //   value = Math.copySign(value * value, value);

  //   return value;
  // }

  public Command getTestCommand(){
    // return new PrototypeTestCommand(driverHorizontal, driverVertical);
    // return new SequentialCommandGroup(new FeederCommand(m_feeder, FeederState.TEST, 0.0, 0.0));
    return new SequentialCommandGroup(new IntakeCommand(m_intake, IntakeState.TEST, 0.0));
  }
}
