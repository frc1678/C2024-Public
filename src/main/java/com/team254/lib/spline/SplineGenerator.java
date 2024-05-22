package com.team254.lib.spline;

import com.team254.lib.geometry.*;

import java.util.ArrayList;
import java.util.List;

public class SplineGenerator {
    private static final double kMaxDX = 0.0508; //m
    private static final double kMaxDY = 0.00127; //m
    private static final double kMaxDTheta = 0.1; //radians!
    private static final int kMinSampleSize = 1;

    /**
     * Converts a spline into a list of Twist2d's.
     *
     * @param s  the spline to parametrize
     * @param t0 starting percentage of spline to parametrize
     * @param t1 ending percentage of spline to parametrize
     * @return list of Pose2dWithCurvature that approximates the original spline
     */
    public static List<Pose2dWithMotion> parameterizeSpline(PoseSpline s, double maxDx, double maxDy, double
            maxDTheta, double t0, double t1) {
        List<Pose2dWithMotion> rv = new ArrayList<>();
        rv.add(s.getPose2dWithMotion(0.0));
        double dt = (t1 - t0);
        for (double t = 0; t < t1; t += dt / kMinSampleSize) {
            getSegmentArc(s, rv, t, t + dt / kMinSampleSize, maxDx, maxDy, maxDTheta);
        }
        return rv;
    }

    /**
     * Convenience function to parametrize a spline from t 0 to 1
     */
    public static List<Pose2dWithMotion> parameterizeSpline(PoseSpline s) {
        return parameterizeSpline(s, kMaxDX, kMaxDY, kMaxDTheta, 0.0, 1.0);
    }

    public static List<Pose2dWithMotion> parameterizeSpline(PoseSpline s, double maxDx, double maxDy, double maxDTheta) {
        return parameterizeSpline(s, maxDx, maxDy, maxDTheta, 0.0, 1.0);
    }

    public static List<Pose2dWithMotion> parameterizeSplines(List<PoseSpline> splines) {
        return parameterizeSplines(splines, kMaxDX, kMaxDY, kMaxDTheta);
    }

    public static List<Pose2dWithMotion> parameterizeSplines(List<? extends PoseSpline> splines, double maxDx, double maxDy,
                                                                double maxDTheta) {
        List<Pose2dWithMotion> rv = new ArrayList<>();
        if (splines.isEmpty()) return rv;
        rv.add(splines.get(0).getPose2dWithMotion(0.0));
        for (int i = 0; i < splines.size(); i++) {
            PoseSpline s = splines.get(i);
            List<Pose2dWithMotion> samples = parameterizeSpline(s, maxDx, maxDy, maxDTheta);
            samples.remove(0);
            rv.addAll(samples);
        }
        return rv;
    }

    private static void getSegmentArc(PoseSpline s, List<Pose2dWithMotion> rv, double t0, double t1, double maxDx,
                                      double maxDy,
                                      double maxDTheta) {
        Pose2d p0 = s.getPose2d(t0);
        Pose2d phalf = s.getPose2d(t0 + (t1 - t0) * .5);
        Pose2d p1 = s.getPose2d(t1);
        Twist2d twist_full = Pose2d.log(p0.inverse().transformBy(p1));
        Pose2d phalf_predicted = p0.transformBy(Pose2d.exp(twist_full.scaled(0.5)));
        Pose2d error = phalf.inverse().transformBy(phalf_predicted);
        Rotation2d course_predicted = (new Rotation2d(twist_full.dx, twist_full.dy, true)).rotateBy(phalf_predicted.getRotation());
        Rotation2d course_half = s.getCourse(t0 + (t1 - t0) * .5).orElse(course_predicted);
        double course_error = course_predicted.inverse().rotateBy(course_half).getRadians();
        if (Math.abs(error.getTranslation().y()) > maxDy ||
            Math.abs(error.getTranslation().x()) > maxDx ||
            Math.abs(error.getRotation().getRadians()) > maxDTheta ||
            Math.abs(course_error) > maxDTheta) {
            getSegmentArc(s, rv, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta);
            getSegmentArc(s, rv, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta);
        } else {
            rv.add(s.getPose2dWithMotion(t1));
        }
    }

}
