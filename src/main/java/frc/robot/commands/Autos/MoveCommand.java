package frc.robot.commands.Autos;

import static frc.robot.util.math.PurePursuitMath.*;

import java.util.ArrayList;

import org.opencv.core.Point;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class MoveCommand extends CommandBase {
    private final DrivetrainSubsystem m_dt = DrivetrainSubsystem.getInstance();

    private ArrayList<CurvePoint> m_points = null;
    private CurvePoint m_setpoint = null; // MSET point
    private ChassisSpeeds m_speeds = null;

    private double m_tolerance = 0.2;
    private double m_angleTolerance = Math.toRadians(7.5);

    // TODO implement these
    private boolean withinMoveTolerance = false;
    private boolean withinTurnTolerance = false;

    public MoveCommand(ArrayList<CurvePoint> allPoints, double tolerance, double angleTolerance) {
        m_points = allPoints;
        m_tolerance = tolerance;
        m_angleTolerance = angleTolerance;
    }

    private ChassisSpeeds goToPoint(double x, double y, double theta, double velocity, double turnVelocity) {
        double distanceToTarget = Math.hypot(x - m_dt.getX(), y - m_dt.getY());

        double absoluteAngleToTarget = Math.atan2(y - m_dt.getY(), x - m_dt.getX());
        double relativeAngleToPoint = angleWrap(absoluteAngleToTarget - (m_dt.getRadians() - (Math.PI / 2)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double relativeTurnAngle = relativeAngleToPoint - Math.PI + theta;
        turnVelocity = clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnVelocity;

        if (distanceToTarget < 10) {
            turnVelocity = 0;
        }

        return new ChassisSpeeds(movementXPower * velocity, movementYPower * velocity, turnVelocity);
    }

    private ArrayList<Point> lineCircleIntersection(
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

    private CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius) {
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for (int i = 0; i < pathPoints.size() - 1; ++i) {
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i + 1);

            // ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());
            // FIXME potentially?
            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, endLine.m_followDistance, startLine.toPoint(), endLine.toPoint());

            double closestAngle = 1000000;
            for (Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - robotLocation.y, thisIntersection.x - robotLocation.x);
                double deltaAngle = Math.abs(angleWrap(angle - m_dt.getRadians()));

                if (deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }
        }

        return followMe;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (
            distance2D(new Point(m_dt.getX(), m_dt.getY()), m_points.get(m_points.size() - 1).toPoint()) <= m_tolerance &&
            m_points.get(m_points.size() - 1).m_theta - m_dt.getRadians() <= m_angleTolerance
        ) {
            this.end(true);
        }

        m_setpoint = getFollowPointPath(
            m_points, 
            new Point(
                m_dt.getX(),
                m_dt.getY()
            ), 
            m_points.get(0).m_followDistance
            // TODO change followDistance based on the next point's followDistance rather than just first point's
        );

        m_speeds = goToPoint(m_setpoint.m_x, m_setpoint.m_y, m_setpoint.m_theta, m_setpoint.m_velocity, m_setpoint.m_turnVelocity);

        // TODO optimize/clean up logic

        if (distance2D(new Point(m_dt.getX(), m_dt.getY()), m_points.get(m_points.size() - 1).toPoint()) <= m_tolerance) {
            withinMoveTolerance = true;
            m_dt.drive(new ChassisSpeeds(0, 0, m_speeds.omegaRadiansPerSecond));
        }

        if (m_points.get(m_points.size() - 1).m_theta - m_dt.getRadians() <= m_angleTolerance) {
            withinTurnTolerance = true;
            m_dt.drive(new ChassisSpeeds(m_speeds.vxMetersPerSecond, m_speeds.vyMetersPerSecond, 0));
        }

        if (!withinMoveTolerance && !withinTurnTolerance) {
            m_dt.drive(m_speeds);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_dt.drive(new ChassisSpeeds());
    }
}
