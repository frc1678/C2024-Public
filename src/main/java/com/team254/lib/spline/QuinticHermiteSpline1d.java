package com.team254.lib.spline;

public class QuinticHermiteSpline1d extends Spline1d {
    double a, b, c, d, e, f;

    public QuinticHermiteSpline1d(double x0, double x1, double dx0, double dx1, double ddx0, double ddx1) {
        a = -6 * x0 - 3 * dx0 - 0.5 * ddx0 + 0.5 * ddx1 - 3 * dx1 + 6 * x1;
        b = 15 * x0 + 8 * dx0 + 1.5 * ddx0 - ddx1 + 7 * dx1 - 15 * x1;
        c = -10 * x0 - 6 * dx0 - 1.5 * ddx0 + 0.5 * ddx1 - 4 * dx1 + 10 * x1;
        d = 0.5 * ddx0;
        e = dx0;
        f = x0;
    }

    public QuinticHermiteSpline1d(QuinticHermiteSpline1d other) {
        a = other.a;
        b = other.b;
        c = other.c;
        d = other.d;
        e = other.e;
        f = other.f;
    }

    QuinticHermiteSpline1d addCoefs(QuinticHermiteSpline1d other) {
        QuinticHermiteSpline1d ret = new QuinticHermiteSpline1d(this);
        ret.a += other.a;
        ret.b += other.b;
        ret.c += other.c;
        ret.d += other.d;
        ret.e += other.e;
        ret.f += other.f;
        return ret;
    }

    /**
     * @param t ranges from 0 to 1
     * @return the point on the spline for that t value
     */
    @Override
    public double getPosition(double t) {
        return a * t * t * t * t * t + b * t * t * t * t + c * t * t * t + d * t * t + e * t + f;
    }

    @Override
    public double getVelocity(double t) {
        return 5 * a * t * t * t * t + 4 * b * t * t * t + 3 * c * t * t + 2 * d * t + e;
    }

    @Override
    public double getAcceleration(double t) {
        return 20 * a * t * t * t + 12 * b * t * t + 6 * c * t + 2 * d;
    }

    @Override
    public double getJerk(double t) {
        return 60 * a * t * t + 24 * b * t + 6 * c;
    }
}
