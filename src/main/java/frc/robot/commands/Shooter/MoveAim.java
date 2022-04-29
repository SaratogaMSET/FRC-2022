package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// Lazy on-the-move shooting solution
public class MoveAim extends CommandBase {
    private final DrivetrainSubsystem m_dt;
    private final VisionSubsystem m_vision;
    private final PIDController m_pid;
    private final SimpleMotorFeedforward m_dtFf;

    // TODO tune
    private final double m_kP = Constants.Drivetrain.kPThetaAimLock;
    private final double m_kI = Constants.Drivetrain.kIThetaAimLock;
    private final double m_kD = 0;

    private final double m_dtKS = 0;
    private final double m_dtKV = 0;
    private final double m_dtKA = 0;

    private double m_pidValue = 0;
    private double m_rotation = 0;

    private DoubleSupplier m_translationXSupplier;
    private DoubleSupplier m_translationYSupplier;

    private double m_translationXTrapezoidal = 0;
    private double m_translationYTrapezoidal = 0;

    private double resultX = 0;
    private double resultY = 0;

    public MoveAim(
        DrivetrainSubsystem drivetrain,
        VisionSubsystem vision,
        DoubleSupplier xTrans,
        DoubleSupplier yTrans
    ) {
        m_dt = drivetrain;
        m_vision = vision;
        m_translationXSupplier = xTrans;
        m_translationYSupplier = yTrans;

        m_pid = new PIDController(m_kP, m_kI, m_kD);
        m_dtFf = new SimpleMotorFeedforward(m_dtKS, m_dtKV, m_dtKA);

        addRequirements(m_dt);
        addRequirements(m_vision);
    }

    @Override
    public void execute() {
        m_translationXTrapezoidal = (m_translationXSupplier.getAsDouble()-m_translationXTrapezoidal)/1 + m_translationXTrapezoidal;
        m_translationYTrapezoidal = (m_translationYSupplier.getAsDouble()-m_translationYTrapezoidal)/1 + m_translationYTrapezoidal;

        double magnitude = Math.hypot(m_translationXTrapezoidal, m_translationYTrapezoidal);

        double joyAngle = Math.atan2(m_translationYTrapezoidal, m_translationXTrapezoidal);
        double roboAngle = (m_dt.getNavHeading() + joyAngle);

        resultX = Math.cos(roboAngle) * magnitude;
        resultY = Math.sin(roboAngle) * magnitude;

        m_pidValue = m_pid.calculate(m_vision.getRawAngle(), 0);
        // FIXME resultX or resultY for horizontal movement? 
        // Should we pass in resultX, resultY, or magnitude?
        m_rotation = m_pidValue + m_dtFf.calculate(resultX);

        m_dt.drive(
            new ChassisSpeeds(
                resultX,
                resultY,
                m_rotation
            )
        );
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_dt.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
