package org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.WheelPositions;

public class OdometerWheelsPositions implements WheelPositions<OdometerWheelsPositions> {
    public final double leftWheelMeters;
    public final double rightWheelMeters;
    public final double centerWheelMeters;

    public OdometerWheelsPositions(double leftWheelMeters, double rightWheelMeters, double centerWheelMeters) {
        this.leftWheelMeters = leftWheelMeters;
        this.rightWheelMeters = rightWheelMeters;
        this.centerWheelMeters = centerWheelMeters;
    }

    @Override
    public boolean equals(Object obj) {
        if (! (obj instanceof OdometerWheelsPositions))
            return false;
        final OdometerWheelsPositions other = (OdometerWheelsPositions) obj;
        return Math.abs(other.leftWheelMeters - this.leftWheelMeters) < 1e-9
                && Math.abs(other.rightWheelMeters - this.rightWheelMeters) < 1e-9
                && Math.abs(other.centerWheelMeters - this.centerWheelMeters) < 1e-9;
    }
    @NonNull
    @SuppressLint("DefaultLocale")
    @Override
    public String toString() {
        return String.format(
                "Odometry Pods Position (Left: %.2f m, Right: %.2f m, Center: %.2f m",
                leftWheelMeters, rightWheelMeters, centerWheelMeters
        );
    }

    @Override
    public OdometerWheelsPositions interpolate(OdometerWheelsPositions endValue, double t) {
        return new OdometerWheelsPositions(
                MathUtil.interpolate(this.leftWheelMeters, endValue.leftWheelMeters, t),
                MathUtil.interpolate(this.rightWheelMeters, endValue.rightWheelMeters, t),
                MathUtil.interpolate(this.centerWheelMeters, endValue.centerWheelMeters, t)
        );
    }

    @Override
    public OdometerWheelsPositions copy() {
        return new OdometerWheelsPositions(leftWheelMeters, rightWheelMeters, centerWheelMeters);
    }

    public OdometerWheelsPositions minus(OdometerWheelsPositions previousValue) {
        return new OdometerWheelsPositions(
                this.leftWheelMeters - previousValue.leftWheelMeters,
                this.rightWheelMeters - previousValue.rightWheelMeters,
                this.centerWheelMeters - previousValue.centerWheelMeters
        );
    }
}
