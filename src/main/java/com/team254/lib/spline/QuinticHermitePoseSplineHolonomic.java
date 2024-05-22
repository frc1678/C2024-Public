package com.team254.lib.spline;

import java.util.Optional;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.Util;


public class QuinticHermitePoseSplineHolonomic extends QuinticHermitePoseSplineNonholonomic {
    final QuinticHermiteSpline1d theta;
    final Rotation2d r0;

    /**
     * @param p0 The starting pose of the spline
     * @param p1 The ending pose of the spline
     */
    public QuinticHermitePoseSplineHolonomic(Pose2d p0, Pose2d p1, Rotation2d r0, Rotation2d r1) {
        super(p0, p1);
        this.r0 = r0;

        double delta = r0.inverse().rotateBy(r1).getRadians();

        theta = new QuinticHermiteSpline1d(0.0, delta, 0, 0, 0, 0);
    }

    private QuinticHermitePoseSplineHolonomic(QuinticHermiteSpline1d x, QuinticHermiteSpline1d y, QuinticHermiteSpline1d theta, Rotation2d r0) {
        super(x, y);
        this.theta = theta;
        this.r0 = r0;
    }

    // Return a new spline that is a copy of this one, but with adjustments to second derivatives.
    @Override
    protected QuinticHermitePoseSplineHolonomic adjustSecondDerivatives(double ddx0_adjustment, double ddx1_adjustment, double ddy0_adjustment, double ddy1_adjustment) {
        return new QuinticHermitePoseSplineHolonomic(
            x.addCoefs(new QuinticHermiteSpline1d(0, 0, 0, 0, ddx0_adjustment, ddx1_adjustment)),
            y.addCoefs(new QuinticHermiteSpline1d(0, 0, 0, 0, ddy0_adjustment, ddy1_adjustment)),
            theta,
            r0);
    }

    @Override
    public double getDHeading(double t) {
        return theta.getVelocity(t);
    }

    @Override
    public Rotation2d getHeading(double t) {
        return r0.rotateBy(Rotation2d.fromRadians(theta.getPosition(t)));
    }

    @Override
    public Optional<Rotation2d> getCourse(double t) {
        if (Util.epsilonEquals(x.getVelocity(t), 0.0) && Util.epsilonEquals(y.getVelocity(t), 0.0)) {
            return Optional.empty();
        }
        return Optional.of(new Rotation2d(x.getVelocity(t), y.getVelocity(t), true));
    }
}
