package com.team254.lib.trajectory.timing;

import com.team254.lib.geometry.Pose2dWithMotion;

public class YawRateConstraint implements TimingConstraint<Pose2dWithMotion> {
    final double mMaxYawRate;

    public YawRateConstraint(final double max_yaw_rate) {
        // rad/s
        mMaxYawRate = max_yaw_rate;
    }

    @Override
    public double getMaxVelocity(final Pose2dWithMotion state) {
        var course = state.getCourse();
        if (course.isEmpty()) {
            // This is turn in place.
            return Double.MAX_VALUE;
        } else {
            // Heading rate in rad/m
            final double heading_rate = state.getHeadingRate();
            // Return max velocity in m/s.
            return mMaxYawRate / Math.abs(heading_rate);
        }
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(final Pose2dWithMotion state, final double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }
}
