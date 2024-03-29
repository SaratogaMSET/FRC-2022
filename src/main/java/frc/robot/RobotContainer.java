// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.Vision;
import frc.robot.commands.Autos.AutoRunCommand;
import frc.robot.commands.Autos.DriveStraight;
import frc.robot.commands.Autos.TurnAngle;
import frc.robot.commands.Drivetrain.DefaultDriveCommand;
// import frc.robot.commands.Drivetrain.ReadGyroscope;
import frc.robot.commands.Drivetrain.SetXConfigCommand;
import frc.robot.commands.Drivetrain.ZeroGyroCommand;
import frc.robot.commands.Hang.DeployHangCommand;
import frc.robot.commands.Hang.HalfHangUpCommand;
import frc.robot.commands.Hang.HangDownCommand;
import frc.robot.commands.Hang.HangUpCommand;
import frc.robot.commands.IntakeFeeder.DeployIntakeCommand;
import frc.robot.commands.IntakeFeeder.RunFeederCommand;
import frc.robot.commands.Shooter.AimForShootCommand;
import frc.robot.commands.Shooter.ConstantAim;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.commands.Test.TestDrivetrainCommandGroup;
import frc.robot.commands.Test.TestFeederCommandGroup;
import frc.robot.commands.Test.TestHangCommandGroup;
import frc.robot.commands.Test.TestIntakeCommandGroup;
import frc.robot.commands.Test.TestShooterCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FeederSubsystem.FeederState;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterZone;
import frc.robot.subsystems.VisionSubsystem;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final SendableChooser<String> m_autoSwitcher = new SendableChooser<String>();
  // public final SendableChooser<String> trajectorySwitcher = new SendableChooser<String>();
  public static final String twoBall = "2 Ball";
  // public static final String twoBallLeft = "2 Ball Left";
  // public static final String twoBallMiddle = "2 Ball Middle";
  // public static final String twoBallRight = "2 Ball Right";
  public static final String fiveBall = "5 Ball";
  // public static double AutonMiddleOffsetBT = 0;
  // public static double AutonMiddleOffsetAT = 0;
  // public static final String threeBall = "3 Field";
  public static final String testBall = "Test Auto";
  // public static final String threeBallShort = "3 Wall";
  // public static final String pathTestBall = "Path Test Auto";
  // public static final String forward = "Forward";
  // public static final String strafe = "Strafe";
  // public static final String regularRotate = "Regular Rotate";
  // public static final String forwardRotate = "Forward Rotate";
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  public static VisionSubsystem m_visionSubsystem;
  private final LEDSubsystem m_LedSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final FeederSubsystem m_feeder;
  private final IntakeSubsystem m_intake;
  private final HangSubsystem m_hangSubsystem;
  private final RobotState m_robotState;
  public static final double pi = Math.PI;
  protected final XboxController m_driver = new XboxController(0);
  protected final Joystick m_gunner = new Joystick(1);
  private final Compressor m_compressor;
  private final LeftTrigger trigger = new LeftTrigger(m_driver);
  // private final Supplier<Integer> slowMode; // SLOWMODE
  // private final Supplier<Double> leftX, leftY, rightX;
  // private final Supplier<Double> driveXScale, driveYScale, turnXScale;
  // private final Joystick driverVertical, driverHorizontal;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = (6380.0 / 60.0 *
      SdsModuleConfigurations.MK4_L2.getDriveReduction() *
      SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI);

  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-Drivetrain.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Drivetrain.DRIVETRAIN_WHEELBASE_METERS / 2.0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // slowMode = () -> m_driver.getRightTriggerAxis() < 0.1 ? 1: 2; // SLOWMODE +
    // 10% deadzone (IN USE)
    // leftX = () -> Math.pow(m_driver.getLeftX(), 3); // Cubic incline
    // leftY = () -> Math.pow(m_driver.getLeftY(), 3); // Cubic incline
    // rightX = () -> Math.pow(m_driver.getRightX(), 3); // Cubic incline
    // driveXScale = () -> m_driver.getLeftX() < 0.4 ? 1.3 : 1;
    // driveYScale = () -> m_driver.getLeftY() < 0.4 ? 1.3 : 1;
    // turnXScale = () -> m_driver.getRightX() < 0.4 ? 1.2 : 1;
    m_drivetrainSubsystem = new DrivetrainSubsystem();
    m_visionSubsystem = new VisionSubsystem();
    m_LedSubsystem = new LEDSubsystem();
    m_shooterSubsystem = new ShooterSubsystem();
    m_feeder = new FeederSubsystem();
    m_intake = new IntakeSubsystem();
    m_hangSubsystem = new HangSubsystem();
    m_robotState = new RobotState();
    new Thread(() -> {
      try {
        Thread.sleep(500);
        m_drivetrainSubsystem.zeroGyroscope();
        // m_drivetrainSubsystem.resetOdometry(new Pose2d());
      } catch (Exception e) {
      }
    }).start();

    m_autoSwitcher.setDefaultOption(twoBall, twoBall);
    // m_autoSwitcher.addOption(twoBallLeft, twoBallLeft);
    // m_autoSwitcher.addOption(twoBallMiddle, twoBallMiddle);
    // m_autoSwitcher.addOption(twoBallRight, twoBallRight);
    // m_autoSwitcher.addOption(threeBall, threeBall);
    // m_autoSwitcher.addOption(threeBallShort, threeBallShort);
    m_autoSwitcher.addOption(fiveBall, fiveBall);
    m_autoSwitcher.addOption(testBall, testBall);

    // m_autoSwitcher.addOption(pathTestBall, pathTestBall);
    // trajectorySwitcher.setDefaultOption(forward, forward);
    // trajectorySwitcher.addOption(strafe, strafe);
    // trajectorySwitcher.addOption(regularRotate, regularRotate);
    // trajectorySwitcher.addOption(forwardRotate, forwardRotate);
    SmartDashboard.putData(m_autoSwitcher);
    // SmartDashboard.putData(trajectorySwitcher);

    // driverVertical = new Joystick(Constants.OIConstants.JOYSTICK_DRIVE_VERTICAL);
    // driverHorizontal = new
    // Joystick(Constants.OIConstants.JOYSTICK_DRIVE_HORIZONTAL);

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> modifyAxisTranslate(m_driver.getLeftX() / 1) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxisTranslate(m_driver.getLeftY() / 1) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> modifyAxis(m_driver.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    new SequentialCommandGroup(
        new WaitCommand(1),
        new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope()),
        new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
        new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(new Pose2d()))).schedule();

    m_compressor = new Compressor(2, PneumaticsModuleType.REVPH);
    // m_compressor.disable();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by ed` y
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Driver Controls
    new Button(m_driver::getRightBumper).whileActiveOnce(
        new ParallelCommandGroup(
            new DeployIntakeCommand(m_intake, IntakeState.DOWN),
            new RunFeederCommand(m_feeder, FeederState.IR_ASSISTED_INTAKE, 0.15, 0.8)));
    new Button(m_driver::getBButton).whileActiveOnce(
        new RunFeederCommand(m_feeder, FeederState.OUTTAKE, 0.2, 0.8));

    new Button(m_driver::getYButton).whileActiveOnce(
        new ParallelCommandGroup(
            // new SetXConfigCommand(m_drivetrainSubsystem),
            new ShootCommand(m_shooterSubsystem, m_visionSubsystem, m_compressor),
            new SequentialCommandGroup(
                new WaitCommand(0.7),
                new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.3, 0.5))));

    new Button(m_driver::getYButton).whenReleased(
        new InstantCommand(() -> m_shooterSubsystem.setRPM(0))
            
      );
    // Back button zeros the gyroscope
    new Button(m_driver::getAButton).whenPressed(
    new ZeroGyroCommand(m_drivetrainSubsystem)
    );

    new Button(m_driver::getLeftBumper).whileActiveOnce(
      new DefaultDriveCommand(
      m_drivetrainSubsystem,
      () -> modifyAxisTranslate(m_driver.getLeftX()*0.6) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, //+
      () -> -modifyAxisTranslate(m_driver.getLeftY()*0.6) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, //-
      () -> modifyAxis(m_driver.getRightX()*0.6) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
      ) //+
    );
    trigger.whileActiveOnce(
      new ParallelCommandGroup(
            // new SetXConfigCommand(m_drivetrainSubsystem),
            new ShootCommand(m_shooterSubsystem, ShooterZone.EMERGENCY, m_compressor),
            new SequentialCommandGroup(
                new WaitCommand(0.7),
                new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.3, 0.5))));
  
    trigger.whenReleased(
      new InstantCommand(() -> m_shooterSubsystem.setRPM(0)));
    // new Button(m_driver::getAButton).whileActiveOnce(
    //     new SequentialCommandGroup(
    //         new AimForShootCommand(m_drivetrainSubsystem, m_visionSubsystem),
    //         new ParallelCommandGroup(
    //             new ShootCommand(m_shooterSubsystem, m_visionSubsystem, m_compressor),
    //             new SequentialCommandGroup(
    //                 new WaitCommand(0.7),
    //                 new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.4, 0.7)))));

    // new Button(m_driver::getYButton).whenPressed(
    // new HangUpCommand(m_hangSubsystem, 0.5)
    // );

    // new Button(m_driver::getAButton).whenPressed(
    // new HangDownCommand(m_hangSubsystem, 0.5)
    // );

    // Gunner Controls
    new JoystickButton(m_gunner, 5).whenPressed(
        new HangUpCommand(m_hangSubsystem, 1));

    new JoystickButton(m_gunner, 6).whenPressed(
        new ParallelCommandGroup(
            new HalfHangUpCommand(m_hangSubsystem, 1),
            // new SequentialCommandGroup(
            // new WaitCommand(0.3),
            new InstantCommand(() -> m_hangSubsystem.checkDeployment())));

    new JoystickButton(m_gunner, 3).whenPressed(
      new SequentialCommandGroup(
        new InstantCommand(()->m_hangSubsystem.undeployHang()),
        new WaitCommand(1),
        new HangDownCommand(m_hangSubsystem, 0.85),
        new InstantCommand(() -> m_hangSubsystem.rightResetEncoders()),
        new InstantCommand(() -> m_hangSubsystem.leftResetEncoders())
        )
        
        );

    new JoystickButton(m_gunner, 12).whileActiveOnce(
        new HangDownCommand(m_hangSubsystem, 0.75, true));

    new JoystickButton(m_gunner, 7).whileActiveOnce(
      new HangUpCommand(m_hangSubsystem, 0.425)
    );
    new JoystickButton(m_gunner,9).whileActiveOnce(
      new HangDownCommand(m_hangSubsystem, 0.425)
    );
        new JoystickButton(m_gunner, 11).whenPressed(
        new SequentialCommandGroup(
          new HangDownCommand(m_hangSubsystem, 0.85),
          new InstantCommand(() -> m_hangSubsystem.rightResetEncoders()),
          new InstantCommand(() -> m_hangSubsystem.leftResetEncoders())));
        
    new JoystickButton(m_gunner, 10).whileActiveOnce(
        new HangUpCommand(m_hangSubsystem, .75));

    new JoystickButton(m_gunner, 4).whenPressed(
        new DeployHangCommand(m_hangSubsystem));

    // Arms up, engage pistons, pull the robot up
    // new JoystickButton(m_gunner, 9).whenPressed(
    // new SequentialCommandGroup(
    // new HangUpCommand(m_hangSubsystem, 1),
    // new WaitCommand(3),
    // new DeployHangCommand(m_hangSubsystem),
    // new WaitCommand(3),
    // new HangDownCommand(m_hangSubsystem, 0.85),
    // new WaitCommand(0.7),
    // new InstantCommand(() -> m_hangSubsystem.rightResetEncoders()),
    // new InstantCommand(() -> m_hangSubsystem.leftResetEncoders())
    // )
    // );

    new JoystickButton(m_gunner, 2).whileActiveOnce(
        new ParallelCommandGroup(
            new SetXConfigCommand(m_drivetrainSubsystem),
            new ShootCommand(m_shooterSubsystem, ShooterZone.EMERGENCY, m_compressor),
            new SequentialCommandGroup(
                new WaitCommand(0.7),
                new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.4, 0.7))));

                new JoystickButton(m_gunner, 1).whileActiveOnce(
                  // new ParallelCommandGroup(
                  // new InstantCommand(() -> m_shooterSubsystem.setRPM(
                  // m_shooterSubsystem.getShooterStateRPM(
                  // ShooterZone.QUADRATIC,
                  // m_visionSubsystem.getDistanceFromTarget()
                  // )
                  // ) ),
                  new ConstantAim(
                      () -> modifyAxisTranslate(m_driver.getLeftX() / 1) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                      () -> -modifyAxisTranslate(m_driver.getLeftY() / 1) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
                      () -> modifyAxis(m_driver.getRightX() / 2) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                      m_drivetrainSubsystem,
                      () -> m_visionSubsystem.getRawAngle())
              // )
              );
    
    new JoystickButton(m_gunner, 1).whenReleased(
        new InstantCommand(() -> m_shooterSubsystem.setRPM(0)));

    // new JoystickButton(m_gunner, 1).whileActiveOnce(
    // new ParallelCommandGroup(
    // new DeployIntakeCommand(m_intake, IntakeState.DOWN),
    // new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.15, 0.8)
    // )
    // );

    // new JoystickButton(m_gunner, 7).whileActiveOnce(
    // new HangUpCommand(m_hangSubsystem, 0.2)
    // );
  }

  public void updateRobotState() {
    RobotState.intakeState = m_intake.getIntakeState();
    RobotState.feederState = m_feeder.getFeederState();
    RobotState.visionState = m_visionSubsystem.updateVisionState();
    RobotState.shooterState = m_shooterSubsystem.getShooterZone(m_visionSubsystem.getDistanceFromTarget());

    m_LedSubsystem.setStatus(m_feeder.intakeGate.get(), m_feeder.shooterGate.get(), RobotState.visionState);

    SmartDashboard.putNumber("X JOYSTICK SPEED", m_driver.getLeftX());
    SmartDashboard.putNumber("Y JOYSTICK SPEED", m_driver.getLeftY());
    SmartDashboard.putNumber("navX Angle", m_drivetrainSubsystem.getNavHeading() * 180 / Math.PI);
    SmartDashboard.putNumber("VISION: Distance", m_visionSubsystem.getDistanceFromTarget());
    SmartDashboard.putNumber("VISION: Angle", m_visionSubsystem.getRawAngle());
    SmartDashboard.putString("SHOOTER: Zone", RobotState.shooterState.toString());
    SmartDashboard.putBoolean("FEEDER: Intake gate", m_feeder.intakeGate.get());
    SmartDashboard.putBoolean("FEEDER: Shooter gate", m_feeder.shooterGate.get());
    SmartDashboard.putString("HANG: limit switch right ", m_hangSubsystem.hangRightLimitSwitch.get() + "");
    SmartDashboard.putString("HANG: limit switch left", m_hangSubsystem.hangLeftLimitSwitch.get() + "");
    SmartDashboard.putString("HANG: encoder left ", m_hangSubsystem.getLeftEncoderValue() + "");
    SmartDashboard.putString("HANG: encoder right ", m_hangSubsystem.getRightEncoderValue() + "");
    SmartDashboard.putString("COMPRESSOR:", m_compressor.getPressure() + "");
    SmartDashboard.putNumber("CONTROLLER: Rotation", modifyAxis(m_driver.getRightX()));
    SmartDashboard.putNumber("CONTROLLER: Fwd Translate", modifyAxisTranslate(m_driver.getLeftY()));

  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else if (value > -deadband && value < 0) {
      return -deadband;
    } else if (value < deadband && value > 0) {
      return deadband;
    } else {
      return 0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // root the axis
    // value = Math.copySign(Math.sqrt(Math.abs(value)), value);

    return value;
  }   

  private static double modifyAxisTranslate(double value) {

    // root the axis
    value = Math.copySign(Math.pow(value, 2), value);

    // Deadband
    value = deadband(value, 0.03);

    return value;
  }

  public Command getTestCommand() {

    return new SequentialCommandGroup(
        // Will make the intake go up and down.
        // new TestLEDCommandGroup(m_led, LED_STATE.ENABLED)
        new TestIntakeCommandGroup(m_intake),

        // Will make the feeder intake, followed by outtake
        new TestFeederCommandGroup(m_feeder, 0.2, 0.2),

        new TestShooterCommandGroup(m_shooterSubsystem, ShooterZone.EMERGENCY, m_compressor),

        new TestHangCommandGroup(m_hangSubsystem, 0.4),
        // Will move all the drivetrain
        new TestDrivetrainCommandGroup(m_drivetrainSubsystem, 1, 1, 0.3));
  }

  public Command getFiveBallAuto() {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));

    // return new SequentialCommandGroup(
    // new WaitCommand(0.5),
    // new ZeroGyroCommand(m_drivetrainSubsystem),
    // new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0,
    // 0.0, 0.0))),
    // new WaitCommand(0.1),
    // new TurnAngle(m_drivetrainSubsystem, -50)
    // );

    // /*
    return new SequentialCommandGroup(
        new WaitCommand(0.2),
        new ZeroGyroCommand(m_drivetrainSubsystem),
        new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
        // new WaitCommand(0.5),
        // new AimForShootCommand(m_drivetrainSubsystem, m_visionSubsystem),
        new ParallelRaceGroup(
            new DeployIntakeCommand(m_intake, IntakeState.DOWN),
            new RunFeederCommand(m_feeder, FeederState.IR_ASSISTED_INTAKE, 0.2, 0.8),
            new SequentialCommandGroup(
                new WaitCommand(0.2),
                new ZeroGyroCommand(m_drivetrainSubsystem),
                new AutoRunCommand(m_drivetrainSubsystem, -1, 0, 0).withTimeout(1.0),
                new WaitCommand(0.5))),
        new SequentialCommandGroup(
            new AimForShootCommand(m_drivetrainSubsystem, m_visionSubsystem),
            new ParallelRaceGroup(
                new ShootCommand(m_shooterSubsystem, m_visionSubsystem, m_compressor),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.4, 0.5).withTimeout(1.0)))),

        new SequentialCommandGroup(
            new WaitCommand(0.2),
            new ZeroGyroCommand(m_drivetrainSubsystem),
            new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
            // new WaitCommand(0.2),
            new TurnAngle(m_drivetrainSubsystem, 90),
            // new AutoRunCommand(m_drivetrainSubsystem, -1, 0, 0).withTimeout(1.5),
            // new AutoRunCommand(m_drivetrainSubsystem, 0, 0, 2.2).withTimeout(1.2),

            new ZeroGyroCommand(m_drivetrainSubsystem),
            new ParallelRaceGroup(
                new DeployIntakeCommand(m_intake, IntakeState.DOWN),
                new RunFeederCommand(m_feeder, FeederState.IR_ASSISTED_INTAKE, 0.2, 0.8),
                new SequentialCommandGroup(
                    new AutoRunCommand(m_drivetrainSubsystem, -2, 0, 0).withTimeout(1.5),
                    new WaitCommand(0.5))

            ),
            // new WaitCommand(0.5),
            new ZeroGyroCommand(m_drivetrainSubsystem),
            new TurnAngle(m_drivetrainSubsystem, -50)
        // new ZeroGyroCommand(m_drivetrainSubsystem),
        // new TurnAngle(m_drivetrainSubsystem, 100),
        // new ZeroGyroCommand(m_drivetrainSubsystem),
        // new TurnAngle(m_drivetrainSubsystem, 100)
        ),

        new SequentialCommandGroup(
            new ParallelRaceGroup(
                new AimForShootCommand(m_drivetrainSubsystem, m_visionSubsystem),
                // new DeployIntakeCommand(m_intake, IntakeState.DOWN),
                new RunFeederCommand(m_feeder, FeederState.IR_ASSISTED_INTAKE, 0.2, 0.8)),
            new ParallelRaceGroup(
                new ShootCommand(m_shooterSubsystem, m_visionSubsystem, m_compressor),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.4, 0.5).withTimeout(.5))))
    // new ParallelRaceGroup(
    // new DeployIntakeCommand(m_intake, IntakeState.DOWN),
    // new RunFeederCommand(m_feeder, FeederState.IR_ASSISTED_INTAKE, 0.2, 0.8),
    // new SequentialCommandGroup(
    // new WaitCommand(0.2),
    // new ZeroGyroCommand(m_drivetrainSubsystem),
    // new AutoRunCommand(m_drivetrainSubsystem, -2.6, 1.23, 0).withTimeout(1.45),
    // new WaitCommand(1)
    // )
    // ),
    // new ZeroGyroCommand(m_drivetrainSubsystem),
    // new ParallelCommandGroup(
    // new ShootCommand(m_shooterSubsystem, ShooterZone.ZONE_4, m_compressor),
    // new SequentialCommandGroup(
    // new AutoRunCommand(m_drivetrainSubsystem, 3, 0, 0).withTimeout(1.3),
    // new ZeroGyroCommand(m_drivetrainSubsystem),
    // new TurnAngle(m_drivetrainSubsystem, 27),
    // // new WaitCommand(0.2),
    // new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.4,
    // 0.5).withTimeout(1.0)
    // )
    // )
    );

    // */
  }

  public Command getThreeBallAuto() {
    return new SequentialCommandGroup(
        new ZeroGyroCommand(m_drivetrainSubsystem),
        new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
        new WaitCommand(0.5),
        new AutoRunCommand(m_drivetrainSubsystem, -1, 0, 0).withTimeout(0.75),
        new SequentialCommandGroup(
            new ParallelRaceGroup(
                new ShootCommand(m_shooterSubsystem, m_visionSubsystem, m_compressor),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.4, 0.5).withTimeout(1.0)))),
        new WaitCommand(0.2),

        new ParallelRaceGroup(
            new DeployIntakeCommand(m_intake, IntakeState.DOWN),
            new RunFeederCommand(m_feeder, FeederState.IR_ASSISTED_INTAKE, 0.2, 0.8),
            new SequentialCommandGroup(
                new WaitCommand(0.2),
                new ZeroGyroCommand(m_drivetrainSubsystem),
                new AutoRunCommand(m_drivetrainSubsystem, -1, 0, 0).withTimeout(0.75),
                new WaitCommand(0.5))),
        new SequentialCommandGroup(
            new AimForShootCommand(m_drivetrainSubsystem, m_visionSubsystem),
            new ParallelRaceGroup(
                new ShootCommand(m_shooterSubsystem, m_visionSubsystem, m_compressor),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.4, 0.5).withTimeout(1.0),
                    new ZeroGyroCommand(m_drivetrainSubsystem),
                    new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0)))))
                    ),
          new InstantCommand(()->m_hangSubsystem.deployHang()),
          new InstantCommand(()->m_hangSubsystem.undeployHang()) );
  }

  public Command getTestAuto() {
    return new SequentialCommandGroup(
        // new SequentialCommandGroup(
        // new ParallelRaceGroup(
        // new ShootCommand(m_shooterSubsystem, m_visionSubsystem, m_compressor),
        // new SequentialCommandGroup(
        // new WaitCommand(0.5),
        // new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.4,
        // 0.5).withTimeout(1.0)
        // )
        // ),
        new WaitCommand(0.2), // 0.2
        new ZeroGyroCommand(m_drivetrainSubsystem),
        new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0)), m_drivetrainSubsystem), // new
                                                                                                                        // WaitCommand(0.5),
        // new AimForShootCommand(m_drivetrainSubsystem, m_visionSubsystem),
        new SequentialCommandGroup(
            new ZeroGyroCommand(m_drivetrainSubsystem),
            new AutoRunCommand(m_drivetrainSubsystem, -1, 0, 0).withTimeout(1.7)
            // new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscopeTest(), m_drivetrainSubsystem)
            )
            );
    // );
  }

  // public Command getTwoBallAutoMiddle() {
  //   // /*
  //   return new SequentialCommandGroup(
  //       new WaitCommand(0.2), // 0.2
  //       new ZeroGyroCommand(m_drivetrainSubsystem),
  //       new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
  //       // new WaitCommand(0.5),
  //       // new AimForShootCommand(m_drivetrainSubsystem, m_visionSubsystem),
  //       new ParallelRaceGroup(
  //           new DeployIntakeCommand(m_intake, IntakeState.DOWN),
  //           new RunFeederCommand(m_feeder, FeederState.IR_ASSISTED_INTAKE, 0.2, 0.8),
  //           new SequentialCommandGroup(
  //               new WaitCommand(0.2),
  //               new ZeroGyroCommand(m_drivetrainSubsystem),
  //               new AutoRunCommand(m_drivetrainSubsystem, -1, 0, 0).withTimeout(1.5),
  //               new WaitCommand(0.5))),
  //       new SequentialCommandGroup(
  //         new ReadGyroscope(m_drivetrainSubsystem, true),
  //           new AimForShootCommand(m_drivetrainSubsystem, m_visionSubsystem),
  //           new ReadGyroscope(m_drivetrainSubsystem, false),
  //           new ParallelRaceGroup(
  //               new ShootCommand(m_shooterSubsystem, m_visionSubsystem, m_compressor),
  //               new SequentialCommandGroup(
  //                   new WaitCommand(0.5),
  //                   new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.4, 0.5).withTimeout(1.0)))),
  //         new InstantCommand(()->m_hangSubsystem.deployHang()),
  //         new InstantCommand(()->m_hangSubsystem.undeployHang()),
  //         new ZeroGyroCommand(m_drivetrainSubsystem, "Middle")
  //         );
  // }
  //   public Command getTwoBallAuto(String path) {
  //           // /*
  //     return new SequentialCommandGroup(
  //         new WaitCommand(0.2), // 0.2
  //         new ZeroGyroCommand(m_drivetrainSubsystem),
  //         new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
  //               // new WaitCommand(0.5),
  //               // new AimForShootCommand(m_drivetrainSubsystem, m_visionSubsystem),
  //         new ParallelRaceGroup(
  //         new DeployIntakeCommand(m_intake, IntakeState.DOWN),
  //         new RunFeederCommand(m_feeder, FeederState.IR_ASSISTED_INTAKE, 0.2, 0.8),
  //         new SequentialCommandGroup(
  //           new WaitCommand(0.2),
  //           new ZeroGyroCommand(m_drivetrainSubsystem),
  //           new AutoRunCommand(m_drivetrainSubsystem, -1, 0, 0).withTimeout(1.5),
  //           new WaitCommand(0.5))),
  //         new SequentialCommandGroup(
  //           new ParallelRaceGroup(
  //             new ShootCommand(m_shooterSubsystem, m_visionSubsystem, m_compressor),
  //             new SequentialCommandGroup(
  //               new WaitCommand(0.7),
  //               new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.4, 0.5).withTimeout(1.0)))),
  //               new InstantCommand(()->m_hangSubsystem.deployHang()),
  //               new InstantCommand(()->m_hangSubsystem.undeployHang()),
  //               new ZeroGyroCommand(m_drivetrainSubsystem, path)
  //           );
          
          
  // }
  public Command getTwoBallAuto() {
    // /*
return new SequentialCommandGroup(
  new WaitCommand(0.2), // 0.2
  new ZeroGyroCommand(m_drivetrainSubsystem),
  new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
        // new WaitCommand(0.5),
        // new AimForShootCommand(m_drivetrainSubsystem, m_visionSubsystem),
  new ParallelRaceGroup(
  new DeployIntakeCommand(m_intake, IntakeState.DOWN),
  new RunFeederCommand(m_feeder, FeederState.IR_ASSISTED_INTAKE, 0.2, 0.8),
  new SequentialCommandGroup(
    new WaitCommand(0.2),
    new AutoRunCommand(m_drivetrainSubsystem, -1, 0, 0).withTimeout(1.5),
    new WaitCommand(0.5))),
  new SequentialCommandGroup(
    new ParallelRaceGroup(
      new ShootCommand(m_shooterSubsystem, m_visionSubsystem, m_compressor),
      new SequentialCommandGroup(
        new WaitCommand(0.7),
        new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.3, 0.5).withTimeout(1.0)))),
        new InstantCommand(()->m_hangSubsystem.deployHang()),
        new InstantCommand(()->m_hangSubsystem.undeployHang())
    );
  }

  public Command getThreeClosedAuto() {
    return new SequentialCommandGroup(
        new ZeroGyroCommand(m_drivetrainSubsystem),
        new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
        new WaitCommand(0.5),
        new AutoRunCommand(m_drivetrainSubsystem, -1, 0, 0).withTimeout(0.65),
        new SequentialCommandGroup(
            new ParallelRaceGroup(
                new ShootCommand(m_shooterSubsystem, m_visionSubsystem, m_compressor),
                new SequentialCommandGroup(
                    new WaitCommand(0.5),
                    new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.4, 0.5).withTimeout(1.0)))),
        new WaitCommand(0.2),

        new ParallelCommandGroup(
            new DeployIntakeCommand(m_intake, IntakeState.DOWN),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new RunFeederCommand(m_feeder, FeederState.IR_ASSISTED_INTAKE, 0.2, 0.8),
                    new SequentialCommandGroup(
                        new WaitCommand(0.2),
                        new ZeroGyroCommand(m_drivetrainSubsystem),
                        new AutoRunCommand(m_drivetrainSubsystem, -1, 0, 0).withTimeout(0.65),
                        new WaitCommand(0.5))),
                new SequentialCommandGroup(
                    new AimForShootCommand(m_drivetrainSubsystem, m_visionSubsystem),
                    new ParallelRaceGroup(
                        new ShootCommand(m_shooterSubsystem, m_visionSubsystem, m_compressor),
                        new SequentialCommandGroup(
                            new WaitCommand(0.5),
                            new RunFeederCommand(m_feeder, FeederState.MANUAL_INTAKE, 0.4, 0.5).withTimeout(1.0),
                            new ZeroGyroCommand(m_drivetrainSubsystem),
                            new InstantCommand(
                                () -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0)))))))));
  }
  // public Command getPathTestAuto(String trajectory, double MaxVelocityConstraint, double maxAccelerationConstraint){

  //   PathPlannerTrajectory path = PathPlanner.loadPath(trajectory, MaxVelocityConstraint, maxAccelerationConstraint);
  //   PIDController xController = new PIDController(Constants.Drivetrain.kPXController, Constants.Drivetrain.kIXController, 0); //FIXME currently 0.0,0.0,0.0
  //   PIDController yController = new PIDController(Constants.Drivetrain.kPYController, Constants.Drivetrain.kIYController, 0);//FIXME currently 0,0,0
  //   ProfiledPIDController thetaController = new ProfiledPIDController(
  //         Constants.Drivetrain.kPThetaControllerTrajectory, 0, 0, new TrapezoidProfile.Constraints(  //FIXME currently 0.06,0,0
  //             MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
  //             MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND/3));
    
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);
  //   PPSwerveControllerCommand pathFollower = new PPSwerveControllerCommand(
  //     path, 
  //     m_drivetrainSubsystem::getPose, 
  //     m_kinematics, 
  //     xController, 
  //     yController, 
  //     thetaController, 
  //     m_drivetrainSubsystem::setModuleStates, 
  //     m_drivetrainSubsystem);

  //   return new SequentialCommandGroup(
  //     new ZeroGyroCommand(m_drivetrainSubsystem),
  //     new InstantCommand(() -> m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0))),
  //     new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(path.getInitialPose())),
  //     new WaitCommand(0.5),
  //     pathFollower
  //   );
  // }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // double velocity = 0;
    // double acceleration;
    String auto = m_autoSwitcher.getSelected();
    // String trajectory = trajectorySwitcher.getSelected();
    // if(trajectory.equals("Forward")) {velocity = 5; acceleration = 0.7;}
    // else if(trajectory.equals("Strafe")) {velocity = 2; acceleration = 0.7;}
    // else if(trajectory.equals("Regular Rotate")) {velocity = 0.5; acceleration = 0.2;}
    // else if(trajectory.equals("Forward Rotate")) {velocity = 2; acceleration = 0.7;}
    // else {velocity = 2; acceleration = 0.7;}
    switch (auto) {
      case twoBall:
        return getTwoBallAuto();
      case fiveBall:
        return getFiveBallAuto();
      // case threeBall:
      //   return getThreeBallAuto();
      // case threeBallShort:
      //   return getThreeClosedAuto();
      case testBall:
        return getTestAuto();
      // case pathTestBall:
        // return getPathTestAuto(trajectory, velocity, acceleration);
      default:
        return getTwoBallAuto();
    }

  }
}
