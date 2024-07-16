package org.firstinspires.ftc.teamcode.Utils.MapleOdometerWheels;

public class OdometerWheelsSpeeds {
    public final double leftWheelMetersPerSecond;
    public final double rightWheelMetersPerSecond;
    public final double centerWheelMetersPerSecond;
    public OdometerWheelsSpeeds() {
        this(0, 0, 0);
    }
    public OdometerWheelsSpeeds(double leftWheelMetersPerSecond, double rightWheelMetersPerSecond, double centerWheelMetersPerSecond) {
        this.leftWheelMetersPerSecond = leftWheelMetersPerSecond;
        this.rightWheelMetersPerSecond = rightWheelMetersPerSecond;
        this.centerWheelMetersPerSecond = centerWheelMetersPerSecond;
    }
}
