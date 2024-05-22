package com.team254.lib.spline;

public abstract class Spline1d {
    public abstract double getPosition(double t);

    // ds/dt
    public abstract double getVelocity(double t);

    // ds^2/dt
    public abstract double getAcceleration(double t);

    // ds^3/dt
    public abstract double getJerk(double t);
}
