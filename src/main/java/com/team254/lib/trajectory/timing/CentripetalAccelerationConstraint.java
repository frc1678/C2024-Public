package com.team254.lib.trajectory.timing;

import com.team254.lib.geometry.Pose2dWithMotion;

public class CentripetalAccelerationConstraint implements TimingConstraint<Pose2dWithMotion> {
    final double mMaxCentripetalAccel;

    public CentripetalAccelerationConstraint(final double max_centripetal_accel) {
        mMaxCentripetalAccel = max_centripetal_accel;
    }

    @Override
    public double getMaxVelocity(final Pose2dWithMotion state) {
        return Math.sqrt(Math.abs(mMaxCentripetalAccel / state.getCurvature()));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(final Pose2dWithMotion state, final double velocity) {
        return MinMaxAcceleration.kNoLimits;
    }
}
