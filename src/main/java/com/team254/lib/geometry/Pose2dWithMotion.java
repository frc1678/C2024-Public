package com.team254.lib.geometry;

import com.team254.lib.util.Util;

import java.text.DecimalFormat;
import java.util.Optional;

public class Pose2dWithMotion implements IPose2d<Pose2dWithMotion>, ICourse2d<Pose2dWithMotion>, IHeadingRate<Pose2dWithMotion>, ICurvature<Pose2dWithMotion> {
    protected static final Pose2dWithMotion kIdentity = new Pose2dWithMotion();

    public static Pose2dWithMotion identity() {
        return kIdentity;
    }

    protected final Pose2d pose_;

    // Even though this Twist provides scalar values for dx, dy, and dtheta, Pose2dWithMotion is purely a spatial construct -
    // it has no sense of time. So rather than being in units of distance-per-time (or radians-per-time), the denominator here is
    // actually distance as well. Thus, it isn't really meaningful to look at dx or dy directly - what is meaningful is:
    //  a) whether or not they are both zero (in which case this is a stationary or turn-in-place motion)
    //  b) the angle formed by them, which is the direction of translation (in the same coordinate frame as pose_).
    // Additionally, this means dtheta is in radians-per-distance if there is translation, or radians-per-radian otherwise.
    protected final Twist2d motion_direction_;

    protected final double curvature_;
    protected final double dcurvature_ds_;

    public Pose2dWithMotion() {
        pose_ = new Pose2d();
        motion_direction_ = new Twist2d(0.0, 0.0, 0.0);
        curvature_ = 0.0;
        dcurvature_ds_ = 0.0;
    }

    public Pose2dWithMotion(final Pose2d pose, double curvature) {
        pose_ = pose;
        motion_direction_ = new Twist2d(0.0, 0.0, 0.0);
        curvature_  = curvature;
        dcurvature_ds_ = 0.0;
    }

    public Pose2dWithMotion(final Pose2d pose, double curvature, double dcurvature_ds) {
        pose_ = pose;
        motion_direction_ = new Twist2d(0.0, 0.0, 0.0);
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }


    public Pose2dWithMotion(final Pose2d pose, final Twist2d motion_direction, double curvature, double dcurvature_ds) {
        pose_ = pose;
        motion_direction_ = motion_direction;
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }

    public Pose2dWithMotion(final Translation2d translation, final Rotation2d rotation, double curvature) {
        pose_ = new Pose2d(translation, rotation);
        motion_direction_ = new Twist2d(0.0, 0.0, 0.0);
        curvature_  = curvature;
        dcurvature_ds_ = 0.0;
    }

    public Pose2dWithMotion(final Translation2d translation, final Rotation2d rotation, double curvature, double dcurvature_ds) {
        pose_ = new Pose2d(translation, rotation);
        motion_direction_ = new Twist2d(0.0, 0.0, 0.0);
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }

    public Pose2dWithMotion(final Translation2d translation, final Rotation2d rotation, final Twist2d motion_direction, double curvature, double dcurvature_ds) {
        pose_ = new Pose2d(translation, rotation);
        motion_direction_ = motion_direction;
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }

    @Override
    public final Pose2d getPose() {
        return pose_;
    }

    @Override
    public Pose2dWithMotion transformBy(Pose2d transform) {
        return new Pose2dWithMotion(getPose().transformBy(transform), motion_direction_, getCurvature(), getDCurvatureDs());
    }

    @Override
    public Pose2dWithMotion mirror() {
        return new Pose2dWithMotion(getPose().mirror().getPose(), motion_direction_.mirror(), -getCurvature(), -getDCurvatureDs());
    }

    @Override
    public Pose2dWithMotion mirrorAboutX(double xValue) {
        return new Pose2dWithMotion(getPose().mirrorAboutX(xValue), motion_direction_.mirrorAboutX(), -getCurvature(), -getDCurvatureDs());
    }

    @Override
    public Pose2dWithMotion mirrorAboutY(double yValue) {
        return new Pose2dWithMotion(getPose().mirrorAboutY(yValue), motion_direction_.mirrorAboutY(), -getCurvature(), -getDCurvatureDs());
    }

    @Override
    public double getCurvature() {
        return curvature_;
    }

    @Override
    public double getDCurvatureDs() {
        return dcurvature_ds_;
    }

    @Override
    public final Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    @Override
    public final Rotation2d getRotation() {
        return getPose().getRotation();
    }

    @Override
    public Pose2dWithMotion interpolate(final Pose2dWithMotion other, double x) {
        return new Pose2dWithMotion(getPose().interpolate(other.getPose(), x),
                motion_direction_.interpolate(other.motion_direction_, x),
                Util.interpolate(getCurvature(), other.getCurvature(), x),
                Util.interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x));
    }

    @Override
    public double distance(final Pose2dWithMotion other) {
        return getPose().distance(other.getPose());
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof Pose2dWithMotion)) {
            return false;
        }

        Pose2dWithMotion p2dwc = (Pose2dWithMotion) other;
        return getPose().equals(p2dwc.getPose()) &&
            motion_direction_.equals(p2dwc.motion_direction_) &&
            Util.epsilonEquals(getCurvature(), p2dwc.getCurvature()) &&
            Util.epsilonEquals(getDCurvatureDs(), p2dwc.getDCurvatureDs());
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return getPose().toString() + ", twist: " + motion_direction_ + ", curvature: " + fmt.format(getCurvature()) + ", dcurvature_ds: " + fmt.format(getDCurvatureDs());
    }

    @Override
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return getPose().toCSV() + "," +
            fmt.format(motion_direction_.dx) + "," +
            fmt.format(motion_direction_.dy) + "," +
            fmt.format(motion_direction_.dtheta) + "," +
            fmt.format(getCurvature()) + "," +
            fmt.format(getDCurvatureDs());
    }

    @Override
    public Pose2dWithMotion rotateBy(Rotation2d other) {
        // motion direction is always relative to pose, so it gets rotated "for free".
        return new Pose2dWithMotion(getPose().rotateBy(other), getCurvature(), getDCurvatureDs());
    }


    @Override
    public Pose2dWithMotion add(Pose2dWithMotion other) {
        return this.transformBy(other.getPose());   // todo make work
    }

    public Optional<Rotation2d> getCourseLocalFrame() {
        var course = getCourse();
        if (course.isEmpty()) { return course; }
        return Optional.of(getRotation().inverse().rotateBy(course.get()));
    }

    @Override
    public Optional<Rotation2d> getCourse() {
        return motion_direction_.getCourse();
    }

    @Override
    public double getHeadingRate() {
        return motion_direction_.dtheta;
    }
}
