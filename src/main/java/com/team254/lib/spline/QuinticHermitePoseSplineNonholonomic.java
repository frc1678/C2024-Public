package com.team254.lib.spline;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;

import java.util.List;
import java.util.Optional;

public class QuinticHermitePoseSplineNonholonomic extends PoseSpline {
    private static final double kEpsilon = 1e-5;
    private static final double kStepSize = 1.0;
    private static final double kMinDelta = 0.001;
    private static final int kSamples = 100;
    private static final int kMaxIterations = 100;

    final QuinticHermiteSpline1d x, y;

    /**
     * @param p0 The starting pose of the spline
     * @param p1 The ending pose of the spline
     */
    public QuinticHermitePoseSplineNonholonomic(Pose2d p0, Pose2d p1) {
        double scale = 1.2 * p0.getTranslation().distance(p1.getTranslation());
        double x0 = p0.getTranslation().x();
        double x1 = p1.getTranslation().x();
        double dx0 = p0.getRotation().cos() * scale;
        double dx1 = p1.getRotation().cos() * scale;
        double ddx0 = 0;
        double ddx1 = 0;
        double y0 = p0.getTranslation().y();
        double y1 = p1.getTranslation().y();
        double dy0 = p0.getRotation().sin() * scale;
        double dy1 = p1.getRotation().sin() * scale;
        double ddy0 = 0;
        double ddy1 = 0;

        x = new QuinticHermiteSpline1d(x0, x1, dx0, dx1, ddx0, ddx1);
        y = new QuinticHermiteSpline1d(y0, y1, dy0, dy1, ddy0, ddy1);
    }

    protected QuinticHermitePoseSplineNonholonomic(QuinticHermiteSpline1d x, QuinticHermiteSpline1d y) {
        this.x = x;
        this.y = y;
    }

    // Return a new spline that is a copy of this one, but with adjustments to second derivatives.
    protected QuinticHermitePoseSplineNonholonomic adjustSecondDerivatives(double ddx0_adjustment, double ddx1_adjustment, double ddy0_adjustment, double ddy1_adjustment) {
        return new QuinticHermitePoseSplineNonholonomic(
            x.addCoefs(new QuinticHermiteSpline1d(0, 0, 0, 0, ddx0_adjustment, ddx1_adjustment)),
            y.addCoefs(new QuinticHermiteSpline1d(0, 0, 0, 0, ddy0_adjustment, ddy1_adjustment)));
    }

    public Pose2d getStartPose() {
        return new Pose2d(
            getPoint(0),
            getHeading(0)
        );
    }

    public Pose2d getEndPose() {
        return new Pose2d(
            getPoint(1),
            getHeading(1)
        );
    }

    /**
     * @param t ranges from 0 to 1
     * @return the point on the spline for that t value
     */
    @Override
    public Translation2d getPoint(double t) {
        return new Translation2d(x.getPosition(t), y.getPosition(t));
    }

    private double dx(double t) {
        return x.getVelocity(t);
    }

    private double dy(double t) {
        return y.getVelocity(t);
    }

    private double ddx(double t) {
        return x.getAcceleration(t);
    }

    private double ddy(double t) {
        return y.getAcceleration(t);
    }

    private double dddx(double t) {
        return x.getJerk(t);
    }

    private double dddy(double t) {
        return y.getJerk(t);
    }

    @Override
    public double getVelocity(double t) {
        return Math.hypot(dx(t), dy(t));
    }

    @Override
    public double getDHeading(double t) {
        return getCurvature(t);
    }

    @Override
    public double getCurvature(double t) {
        return (dx(t) * ddy(t) - ddx(t) * dy(t)) / ((dx(t) * dx(t) + dy(t) * dy(t)) * Math.sqrt((dx(t) * dx(t) + dy
                (t) * dy(t))));
    }

    @Override
    public double getDCurvature(double t) {
        double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
        double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2 - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
        return num / (dx2dy2 * dx2dy2 * Math.sqrt(dx2dy2));
    }

    private double dCurvature2(double t) {
        double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
        double num = (dx(t) * dddy(t) - dddx(t) * dy(t)) * dx2dy2 - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
        return num * num / (dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2);
    }

    @Override
    public Rotation2d getHeading(double t) {
        return new Rotation2d(dx(t), dy(t), true);
    }

    @Override
    public Optional<Rotation2d> getCourse(double t) {
        if (Util.epsilonEquals(x.getVelocity(t), 0.0) && Util.epsilonEquals(y.getVelocity(t), 0.0)) {
            return Optional.empty();
        }
        return Optional.of(getHeading(t));
    }

    /**
     * @return integral of dCurvature^2 over the length of the spline
     */
    private double sumDCurvature2() {
        double dt = 1.0 / kSamples;
        double sum = 0;
        for (double t = 0; t < 1.0; t += dt) {
            sum += (dt * dCurvature2(t));
        }
        return sum;
    }

    /**
     * @return integral of dCurvature^2 over the length of multiple splines
     */
    public static double sumDCurvature2(List<QuinticHermitePoseSplineNonholonomic> splines) {
        double sum = 0;
        for (QuinticHermitePoseSplineNonholonomic s : splines) {
            sum += s.sumDCurvature2();
        }
        return sum;
    }

    /**
     * Makes optimization code a little more readable
     */
    private static class ControlPoint {
        private double ddx, ddy;
    }

    /**
     * Finds the optimal second derivative values for a set of splines to reduce the sum of the change in curvature
     * squared over the path
     *
     * @param splines the list of splines to optimize
     * @return the final sumDCurvature2
     */
    public static double optimizeSpline(List<QuinticHermitePoseSplineNonholonomic> splines) {
        int count = 0;
        double prev = sumDCurvature2(splines);
        while (count < kMaxIterations) {
            runOptimizationIteration(splines);
            double current = sumDCurvature2(splines);
            if (prev - current < kMinDelta)
                return current;
            prev = current;
            count++;
        }
        return prev;
    }


    /**
     * Runs a single optimization iteration
     */
    private static void runOptimizationIteration(List<QuinticHermitePoseSplineNonholonomic> splines) {
        //can't optimize anything with less than 2 splines
        if (splines.size() <= 1) {
            return;
        }

        ControlPoint[] controlPoints = new ControlPoint[splines.size() - 1];
        double magnitude = 0;

        for (int i = 0; i < splines.size() - 1; ++i) {
            //don't try to optimize colinear points
            if (splines.get(i).getStartPose().isColinear(splines.get(i + 1).getStartPose()) || splines.get(i).getEndPose().isColinear(splines.get(i + 1).getEndPose())) {
                continue;
            }
            double original = sumDCurvature2(splines);
            QuinticHermitePoseSplineNonholonomic temp, temp1;

            temp = splines.get(i);
            temp1 = splines.get(i + 1);
            controlPoints[i] = new ControlPoint(); //holds the gradient at a control point

            //calculate partial derivatives of sumDCurvature2
            splines.set(i, temp.adjustSecondDerivatives(0, kEpsilon, 0, 0));
            splines.set(i + 1, temp1.adjustSecondDerivatives(kEpsilon, 0, 0, 0));
            controlPoints[i].ddx = (sumDCurvature2(splines) - original) / kEpsilon;
            splines.set(i, temp.adjustSecondDerivatives(0, 0, 0, kEpsilon));
            splines.set(i + 1, temp1.adjustSecondDerivatives(0, 0, kEpsilon, 0));
            controlPoints[i].ddy = (sumDCurvature2(splines) - original) / kEpsilon;

            splines.set(i, temp);
            splines.set(i + 1, temp1);
            magnitude += controlPoints[i].ddx * controlPoints[i].ddx + controlPoints[i].ddy * controlPoints[i].ddy;
        }

        magnitude = Math.sqrt(magnitude);

        //minimize along the direction of the gradient
        //first calculate 3 points along the direction of the gradient
        Translation2d p1, p2, p3;
        p2 = new Translation2d(0, sumDCurvature2(splines)); //middle point is at the current location

        for (int i = 0; i < splines.size() - 1; ++i) { //first point is offset from the middle location by -stepSize
            if (splines.get(i).getStartPose().isColinear(splines.get(i + 1).getStartPose()) || splines.get(i).getEndPose().isColinear(splines.get(i + 1).getEndPose())) {
                continue;
            }
            //normalize to step size
            controlPoints[i].ddx *= kStepSize / magnitude;
            controlPoints[i].ddy *= kStepSize / magnitude;

            //move opposite the gradient by step size amount
            splines.set(i, splines.get(i).adjustSecondDerivatives(0, -controlPoints[i].ddx, 0, -controlPoints[i].ddy));
            splines.set(i + 1, splines.get(i + 1).adjustSecondDerivatives(-controlPoints[i].ddx, 0, -controlPoints[i].ddy, 0));
        }
        p1 = new Translation2d(-kStepSize, sumDCurvature2(splines));

        for (int i = 0; i < splines.size() - 1; ++i) { //last point is offset from the middle location by +stepSize
            if (splines.get(i).getStartPose().isColinear(splines.get(i + 1).getStartPose()) || splines.get(i).getEndPose().isColinear(splines.get(i + 1).getEndPose())) {
                continue;
            }
            //move along the gradient by 2 times the step size amount (to return to original location and move by 1
            // step)
            splines.set(i, splines.get(i).adjustSecondDerivatives(0, 2 * controlPoints[i].ddx, 0, 2 * controlPoints[i].ddy));
            splines.set(i + 1, splines.get(i + 1).adjustSecondDerivatives(2 * controlPoints[i].ddx, 0, 2 * controlPoints[i].ddy, 0));
        }

        p3 = new Translation2d(kStepSize, sumDCurvature2(splines));

        double stepSize = fitParabola(p1, p2, p3); //approximate step size to minimize sumDCurvature2 along the gradient

        for (int i = 0; i < splines.size() - 1; ++i) {
            if (splines.get(i).getStartPose().isColinear(splines.get(i + 1).getStartPose()) || splines.get(i).getEndPose().isColinear(splines.get(i + 1).getEndPose())) {
                continue;
            }
            //move by the step size calculated by the parabola fit (+1 to offset for the final transformation to find
            // p3)
            controlPoints[i].ddx *= 1 + stepSize / kStepSize;
            controlPoints[i].ddy *= 1 + stepSize / kStepSize;

            splines.set(i, splines.get(i).adjustSecondDerivatives(0, controlPoints[i].ddx, 0, controlPoints[i].ddy));
            splines.set(i + 1, splines.get(i + 1).adjustSecondDerivatives(controlPoints[i].ddx, 0, controlPoints[i].ddy, 0));
        }
    }

    /**
     * fits a parabola to 3 points
     *
     * @return the x coordinate of the vertex of the parabola
     */
    private static double fitParabola(Translation2d p1, Translation2d p2, Translation2d p3) {
        double A = (p3.x() * (p2.y() - p1.y()) + p2.x() * (p1.y() - p3.y()) + p1.x() * (p3.y() - p2.y()));
        double B = (p3.x() * p3.x() * (p1.y() - p2.y()) + p2.x() * p2.x() * (p3.y() - p1.y()) + p1.x() * p1.x() *
                (p2.y() - p3.y()));
        return -B / (2 * A);
    }
}
