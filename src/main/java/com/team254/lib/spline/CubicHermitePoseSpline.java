package com.team254.lib.spline;

import java.util.Optional;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;

/**
 * Temporary spline for testing
 */
public class CubicHermitePoseSpline extends PoseSpline {
    final CubicHermiteSpline1d x, y;

    public CubicHermitePoseSpline(Pose2d p0, Pose2d p1) {
        double x0, x1, dx0, dx1, y0, y1, dy0, dy1;
        double scale = 2 * p0.getTranslation().distance(p1.getTranslation());
        x0 = p0.getTranslation().x();
        x1 = p1.getTranslation().x();
        dx0 = p0.getRotation().cos() * scale;
        dx1 = p1.getRotation().cos() * scale;
        y0 = p0.getTranslation().y();
        y1 = p1.getTranslation().y();
        dy0 = p0.getRotation().sin() * scale;
        dy1 = p1.getRotation().sin() * scale;

        x = new CubicHermiteSpline1d(x0, x1, dx0, dx1);
        y = new CubicHermiteSpline1d(y0, y1, dy0, dy1);
    }

    @Override
    public Translation2d getPoint(double t) {
        return new Translation2d(x.getPosition(t), y.getPosition(t));
    }

    @Override
    public Rotation2d getHeading(double t) {
        return new Rotation2d(x.getVelocity(t), y.getVelocity(t), true);
    }

    @Override
    public Optional<Rotation2d> getCourse(double t) {
        if (Util.epsilonEquals(x.getVelocity(t), 0.0) && Util.epsilonEquals(y.getVelocity(t), 0.0)) {
            return Optional.empty();
        }
        return Optional.of(getHeading(t));
    }

    @Override
    public double getVelocity(double t) {
        return Math.hypot(x.getVelocity(t), y.getVelocity(t));
    }

    @Override
    public double getDHeading(double t) {
        return getCurvature(t);
    }

    @Override
    public double getCurvature(double t) {
        final double dx = x.getVelocity(t);
        final double dy = y.getVelocity(t);
        final double ddx = x.getAcceleration(t);
        final double ddy = y.getAcceleration(t);

        return (dx * ddy - dy * ddx) / ((dx * dx + dy * dy) * Math.sqrt(dx * dx + dy * dy));
    }

    @Override
    public double getDCurvature(double t) {
        final double dx = x.getVelocity(t);
        final double dy = y.getVelocity(t);
        final double ddx = x.getAcceleration(t);
        final double ddy = y.getAcceleration(t);
        final double dddx = x.getJerk(t);
        final double dddy = y.getJerk(t);
        double dx2dy2 = (dx * dx + dy * dy);
        double num = (dx * dddy - dddx * dy) * dx2dy2 - 3 * (dx * ddy - ddx * dy) * (dx * ddx + dy * ddy);
        return num / (dx2dy2 * dx2dy2 * Math.sqrt(dx2dy2));
    }
}
