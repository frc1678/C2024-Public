package com.team254.lib.geometry;

import com.team254.lib.util.Interpolable;
import com.team254.lib.util.Util;

import java.text.DecimalFormat;
import java.util.Optional;

/**
 * A movement along an arc at constant curvature and velocity. We can use ideas from "differential calculus" to create
 * new RigidTransform2d's from a Twist2d and visa versa.
 * <p>
 * A Twist can be used to represent a difference between two poses, a velocity, an acceleration, etc.
 */
public class Twist2d implements Interpolable<Twist2d>, ICourse2d<Twist2d> {
    protected static final Twist2d kIdentity = new Twist2d(0.0, 0.0, 0.0);

    public static Twist2d identity() {
        return kIdentity;
    }

    public final double dx;
    public final double dy;
    public final double dtheta; // Radians!

    public Twist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    public Twist2d scaled(double scale) {
        return new Twist2d(dx * scale, dy * scale, dtheta * scale);
    }

    public Twist2d mirror() {
        return new Twist2d(dx, -dy, -dtheta);
    }

    public Twist2d mirrorAboutX() {
        return new Twist2d(-dx, dy, -dtheta);
    }

    public Twist2d mirrorAboutY() {
        return new Twist2d(dx, -dy, -dtheta);
    }

    public double norm() {
        // Common case of dy == 0
        if (dy == 0.0)
            return Math.abs(dx);
        return Math.hypot(dx, dy);
    }

    public double norm2() {
        return dx * dx + dy * dy;
    }

    public boolean hasTranslation() {
        return norm2() > Util.kEpsilon;
    }

    // Return a vector in the local direction of motion
    @Override
    public Optional<Rotation2d> getCourse() {
        if (hasTranslation()) {
            return Optional.of(new Rotation2d(dx, dy, true));
        } else {
            return Optional.empty();
        }
    }

    // Commented out to avoid confusion since we use Twists to refer to various things, some of which have meaningful curvatures and others don't.
    /*public double curvature() {
        if (Math.abs(dtheta) < Util.kEpsilon && norm() < Util.kEpsilon)
            return 0.0;
        return dtheta / norm();
    }*/

    public boolean epsilonEquals(final Twist2d other, double epsilon) {
        return Util.epsilonEquals(dx, other.dx, epsilon) &&
               Util.epsilonEquals(dy, other.dy, epsilon) &&
               Util.epsilonEquals(dtheta, other.dtheta, epsilon);
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(dx) + "," + fmt.format(dy) + "," + fmt.format(Math.toDegrees(dtheta)) + " deg)";
    }

    @Override
    public Twist2d interpolate(Twist2d other, double x) {
        return new Twist2d(Util.interpolate(dx, other.dx, x),
                           Util.interpolate(dy, other.dy, x),
                           Util.interpolate(dtheta, other.dtheta, x));
    }
}