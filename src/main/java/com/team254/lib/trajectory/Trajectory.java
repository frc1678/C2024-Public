package com.team254.lib.trajectory;

import com.team254.lib.geometry.State;
import com.team254.lib.util.CSVWritable;

import java.util.ArrayList;
import java.util.List;

public class Trajectory<S extends State<S>> implements CSVWritable {
    protected final List<TrajectoryPoint<S>> points_;
    protected final IndexView index_view_ = new IndexView();
    protected double default_velocity_;

    /**
     * Create an empty trajectory.
     */
    public Trajectory() {
        points_ = new ArrayList<>();
    }

    /**
     * Create a trajectory from the given states.
     *
     * @param states The states of the trajectory.
     */
    public Trajectory(final List<S> states) {
        points_ = new ArrayList<>(states.size());
        for (int i = 0; i < states.size(); ++i) {
            points_.add(new TrajectoryPoint<>(states.get(i), i));
        }
    }

    public void setDefaultVelocity(double velocity) {
        default_velocity_ = velocity;
    }

    public double getDefaultVelocity() {
        return default_velocity_;
    }

    public boolean isEmpty() {
        return points_.isEmpty();
    }

    public int length() {
        return points_.size();
    }

    public TrajectoryPoint<S> getLastPoint() {
        return points_.get(length() - 1);
    }

    public TrajectoryPoint<S> getPoint(final int index) {
        return points_.get(index);
    }

    public TrajectorySamplePoint<S> getInterpolated(final double index) {
        if (isEmpty()) {
            return null;
        } else if (index <= 0.0) {
            return new TrajectorySamplePoint<>(getPoint(0));
        } else if (index >= length() - 1) {
            return new TrajectorySamplePoint<>(getPoint(length() - 1));
        }
        final int i = (int) Math.floor(index);
        final double frac = index - i;
        if (frac <= Double.MIN_VALUE) {
            return new TrajectorySamplePoint<>(getPoint(i));
        } else if (frac >= 1.0 - Double.MIN_VALUE) {
            return new TrajectorySamplePoint<>(getPoint(i + 1));
        } else {
            return new TrajectorySamplePoint<>(getPoint(i).state().interpolate(getPoint(i + 1).state(), frac), i, i + 1);
        }
    }

    public IndexView getIndexView() {
        return index_view_;
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < length(); ++i) {
            builder.append(i);
            builder.append(": ");
            builder.append(getPoint(i).state());
            builder.append(System.lineSeparator());
        }
        return builder.toString();
    }

    @Override
    public String toCSV() {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < length(); ++i) {
            builder.append(i);
            builder.append(",");
            builder.append(getPoint(i).state().toCSV());
            builder.append(System.lineSeparator());
        }
        return builder.toString();
    }

    public class IndexView implements TrajectoryView<S> {
        @Override
        public TrajectorySamplePoint<S> sample(double index) {
            return Trajectory.this.getInterpolated(index);
        }

        @Override
        public double last_interpolant() {
            return Math.max(0.0, Trajectory.this.length() - 1);
        }

        @Override
        public double first_interpolant() {
            return 0.0;
        }

        @Override
        public Trajectory<S> trajectory() {
            return Trajectory.this;
        }
    }
}
