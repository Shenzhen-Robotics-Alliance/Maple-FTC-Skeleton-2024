package org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;

public class OdometerWheelsKinematics implements Kinematics<OdometerWheelsSpeeds, OdometerWheelsPositions> {
    public final double trackWidth, centerWheelOffSetFromCenter;

    public OdometerWheelsKinematics(double trackWidth, double centerWheelOffSetFromCenter) {
        this.trackWidth = trackWidth;
        this.centerWheelOffSetFromCenter = centerWheelOffSetFromCenter;
    }

    @Override
    public ChassisSpeeds toChassisSpeeds(OdometerWheelsSpeeds wheelSpeeds) {
        final double omegaRadPerSec = (wheelSpeeds.leftWheelMetersPerSecond - wheelSpeeds.rightWheelMetersPerSecond) / trackWidth,
                xMPS = (wheelSpeeds.leftWheelMetersPerSecond + wheelSpeeds.rightWheelMetersPerSecond) / 2,
                yMPS = (wheelSpeeds.centerWheelMetersPerSecond - (centerWheelOffSetFromCenter * omegaRadPerSec));
        return new ChassisSpeeds(xMPS, yMPS, omegaRadPerSec);
    }

    @Override
    public OdometerWheelsSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
        throw new UnsupportedOperationException("odometry pods are not used to drive!");
    }

    @Override
    public Twist2d toTwist2d(OdometerWheelsPositions start, OdometerWheelsPositions end) {
        double deltaLeftEncoder = end.leftWheelMeters - start.leftWheelMeters;
        double deltaRightEncoder = end.rightWheelMeters - start.rightWheelMeters;
        double deltaHorizontalEncoder = end.centerWheelMeters - start.centerWheelMeters;

        double dTheta = (deltaRightEncoder - deltaLeftEncoder) / trackWidth;


        double dx = (deltaLeftEncoder + deltaRightEncoder) / 2;
        double dy = deltaHorizontalEncoder - (centerWheelOffSetFromCenter * dTheta);

        return new Twist2d(dx, dy, dTheta);
    }

    @Override
    public OdometerWheelsPositions interpolate(OdometerWheelsPositions startValue, OdometerWheelsPositions endValue, double t) {
        return startValue.interpolate(endValue, t);
    }

    @Override
    public OdometerWheelsPositions copy(OdometerWheelsPositions positions) {
        return positions.copy();
    }
}
