package com.team254.lib.spline;

public class CubicHermiteSpline1d extends Spline1d {
    final double a, b, c, d;

    public CubicHermiteSpline1d(double p0, double p1, double v0, double v1) {
        a = v0 + v1 + 2 * p0 - 2 * p1;
        b = -2 * v0 - v1 - 3 * p0 + 3 * p1;
        c = v0;
        d = p0;
    }

    @Override
    public double getPosition(double t) {
        return t * t * t * a + t * t * b + t * c + d;
    }

    @Override
    public double getVelocity(double t) {
        return 3 * t * t * a + 2 * t * b + c;
    }

    @Override
    public double getAcceleration(double t) {
        return 6 * t * a + 2 * b;
    }

    @Override
    public double getJerk(double t) {
        return 6 * a;
    }
}
