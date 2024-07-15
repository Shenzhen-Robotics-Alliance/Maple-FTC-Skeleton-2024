package org.firstinspires.ftc.teamcode.Utils;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class InterpolatedMotorFeedForward {
    private final InterpolatingDoubleTreeMap speedToMotorPowerMap;

    public InterpolatedMotorFeedForward(double[] motorPowers, double[] measuredSpeeds) {
        this.speedToMotorPowerMap = new InterpolatingDoubleTreeMap();
        if (motorPowers.length != measuredSpeeds.length)
            throw new IllegalArgumentException("motor powers length does not match measured speeds length");

        speedToMotorPowerMap.put(0.0, 0.0);
        for (int i = 0; i < motorPowers.length; i++)
            speedToMotorPowerMap.put(measuredSpeeds[i], motorPowers[i]);
    }

    public double getMotorPower(double desiredVelocity) {
        final double desiredSpeed = Math.abs(desiredVelocity),
                correspondingPower = speedToMotorPowerMap.get(desiredSpeed);
        return Math.copySign(correspondingPower, desiredVelocity);
    }
}
