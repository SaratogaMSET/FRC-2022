package frc.robot.commands.Autos;

import org.opencv.core.Point;

public class CurvePoint {
    public double m_x;
    public double m_y;
    public double m_theta;
    public double m_velocity;
    public double m_turnVelocity;
    public double m_followDistance;
    public double m_pointLength;
    public double m_slowDownTurnRadians;
    public double m_slowDownTurnAmount;

    public CurvePoint(
        double x, 
        double y, 
        double theta,
        double velocity, 
        double turnVelocity,
        double followDistance,
        double pointLength,
        double slowDownTurnRadians,
        double slowDownTurnAmount
    ) {
        m_x = x;
        m_y = y;
        m_theta = theta;
        m_velocity = velocity;
        m_turnVelocity = turnVelocity;
        m_followDistance = followDistance;
        m_pointLength = pointLength;
        m_slowDownTurnRadians = slowDownTurnRadians;
        m_slowDownTurnAmount = slowDownTurnAmount;
    }

    public CurvePoint(CurvePoint p) {
        m_x = p.m_x;
        m_y = p.m_y;
        m_theta = p.m_theta;
        m_velocity = p.m_velocity;
        m_turnVelocity = p.m_turnVelocity;
        m_followDistance = p.m_followDistance;
        m_pointLength = p.m_pointLength;
        m_slowDownTurnRadians = p.m_slowDownTurnRadians;
        m_slowDownTurnAmount = p.m_slowDownTurnAmount;
    }

    public Point toPoint() {
        return new Point(m_x, m_y);
    }

    public void setPoint(Point p) {
        m_x = p.x;
        m_y = p.y;
    }
}
