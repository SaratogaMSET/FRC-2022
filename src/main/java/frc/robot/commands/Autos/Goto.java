package frc.robot.commands.Autos;

import org.opencv.core.Point;
import java.util.ArrayList;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

// TODO cleaup
// FIXME everything.
public class Goto extends CommandBase {
    private final DrivetrainSubsystem m_drivetrain = DrivetrainSubsystem.getInstance();

    private double m_tX, m_tY, m_tTheta, m_velocity, m_turnVelocity;
    private double distanceToTarget;
    private double absoluteAngleToTarget;
    private double relativeAngleToPoint;
    private double relativeXToPoint;
    private double relativeYToPoint;
    private double movementXPower;
    private double movementYPower;
    private double relativeTurnAngle;

    public Goto(double x, double y, double theta, double velocity, double turnVelocity) {
        m_tX = x;
        m_tY = y;
        m_tTheta = theta;
        m_velocity = velocity;
        m_turnVelocity = turnVelocity;
    }

    public static double angleWrap(double angle) {
        /* while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        return angle; */

        angle = angle % (2 * Math.PI);
        angle = (angle + (2 * Math.PI)) % (2 * Math.PI);
        if (angle >= Math.PI) {
            angle -= (2 * Math.PI);
        }

        return angle;
    }

    public static double clip(double n, double min, double max) {
        return Math.max(min, Math.min(max, n));
    }

    public static ArrayList<Point> lineCircleIntersection(
        Point circleCenter, 
        double radius,
        Point linePoint1,
        Point linePoint2
    ) {
        if (Math.abs(linePoint1.y - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if (Math.abs(linePoint1.x - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }

        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);
        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticA = 1.0 + Math.pow(m1, 2);
        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2) * x1);
        double quadraticC = ((Math.pow(m1, 2) * Math.pow(x1, 2))) - (2.0 * y1 * m1 * x1) + Math.pow(y1, 2) - Math.pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try {
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minx = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxx = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;
            if (xRoot1 > minx && xRoot1 < maxx) {
                allPoints.add(new Point(xRoot1, yRoot1));
            }
            
            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;
            if (xRoot2 > minx && xRoot2 < maxx) {
                allPoints.add(new Point(xRoot2, yRoot2));
            }
        } catch(Exception e) {
            // We do some weird recursion if our circle doesn't intersect with the line
            allPoints = lineCircleIntersection(circleCenter, radius + 0.1, linePoint1, linePoint2);
        }

        return allPoints;
    }

    public CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size() - 1; ++i) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 1000000;
            for (Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - robotLocation.y, thisIntersection.x - robotLocation.x);
                double deltaAngle = Math.abs(angleWrap(angle - m_drivetrain.getRadians()));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }

        return followMe;
    }

    public void followCurve(ArrayList<CurvePoint> allPoints, double followAngle) {
        CurvePoint followMe = getFollowPointPath(
            allPoints, 
            new Point(
                m_drivetrain.getX(),
                m_drivetrain.getY()
            ), 
            allPoints.get(0).m_followDistance
            // TODO change followDistance based on the next point's followDistance, rather than the first point's
        );

        new Goto(followMe.m_x, followMe.m_y, followAngle, followMe.m_velocity, followMe.m_turnVelocity);
    }

    @Override
    public void initialize() {
        distanceToTarget = Math.hypot(m_tX - m_drivetrain.getX(), m_tY - m_drivetrain.getY());

        absoluteAngleToTarget = Math.atan2(m_tY - m_drivetrain.getY(), m_tX - m_drivetrain.getX());
        relativeAngleToPoint = angleWrap(absoluteAngleToTarget - (m_drivetrain.getRadians() - (Math.PI / 2)));

        relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        relativeTurnAngle = relativeAngleToPoint - Math.PI + m_tTheta;
        m_turnVelocity = clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * m_turnVelocity;
    }

    @Override
    public void execute() {
        distanceToTarget = Math.hypot(m_tX - m_drivetrain.getX(), m_tY - m_drivetrain.getY());

        absoluteAngleToTarget = Math.atan2(m_tY - m_drivetrain.getY(), m_tX - m_drivetrain.getX());
        relativeAngleToPoint = angleWrap(absoluteAngleToTarget - (m_drivetrain.getRadians() - (Math.PI / 2)));

        relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        relativeTurnAngle = relativeAngleToPoint - Math.PI + m_tTheta;
        m_turnVelocity = clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * m_turnVelocity;

        if (distanceToTarget < 10) {
            m_turnVelocity = 0;
        }

        m_drivetrain.drive(
            new ChassisSpeeds(
                movementXPower * m_velocity,
                movementYPower * m_velocity,
                m_turnVelocity
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(new ChassisSpeeds());
    }
}
