package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterZone;

// Lazy on-the-move shooting solution
public class MoveShoot extends CommandBase {
    private final ShooterSubsystem m_shooter;
    private Compressor m_compressor;
    private VisionSubsystem m_vision;
    private DrivetrainSubsystem m_dt;

    private ShooterZone m_zone = ShooterZone.ZONE_1;

    private double m_rpm;

    private final SimpleMotorFeedforward m_shooterFf;
    
    // FIXME maybe different feedforwards for different shooter zones?
    private final double m_kS = 0;
    private final double m_kV = 0;
    private final double m_kA = 0;

    private DoubleSupplier m_translationXSupplier;
    private DoubleSupplier m_translationYSupplier;

    private double m_translationXTrapezoidal = 0;
    private double m_translationYTrapezoidal = 0;

    // private double resultX = 0;
    private double resultY = 0;

    public MoveShoot(
        ShooterSubsystem shooter, 
        VisionSubsystem vision, 
        Compressor compressor,
        DrivetrainSubsystem dt,
        DoubleSupplier xT,
        DoubleSupplier yT
    ) {
        m_shooter = shooter;
        m_vision = vision;
        m_compressor = compressor;
        m_dt = dt;

        m_translationXSupplier = xT;
        m_translationYSupplier = yT;

        m_shooterFf = new SimpleMotorFeedforward(m_kS, m_kV, m_kA);

        addRequirements(shooter);
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        m_translationXTrapezoidal = (m_translationXSupplier.getAsDouble()-m_translationXTrapezoidal)/1 + m_translationXTrapezoidal;
        m_translationYTrapezoidal = (m_translationYSupplier.getAsDouble()-m_translationYTrapezoidal)/1 + m_translationYTrapezoidal;

        double magnitude = Math.hypot(m_translationXTrapezoidal, m_translationYTrapezoidal);

        double joyAngle = Math.atan2(m_translationYTrapezoidal, m_translationXTrapezoidal);
        double roboAngle = (m_dt.getNavHeading() + joyAngle);

        // resultX = Math.cos(roboAngle) * magnitude;
        resultY = Math.sin(roboAngle) * magnitude;

        double distance = m_vision.getDistanceFromTarget();
        m_zone = m_shooter.getShooterZone(distance);
        // FIXME resultX or resultY for front/back movement? 
        // Should we pass in resultX, resultY, or magnitude?
        m_rpm = m_shooter.getShooterStateRPM(m_zone, distance) + m_shooterFf.calculate(resultY);
        m_compressor.disable();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("RPM Setpoint: ", m_rpm);

        m_shooter.setRPM(m_rpm);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setRPM(0);
        m_shooter.setAngle(true);
        m_compressor.enableDigital();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
