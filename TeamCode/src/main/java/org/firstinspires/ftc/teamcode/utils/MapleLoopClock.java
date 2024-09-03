package org.firstinspires.ftc.teamcode.utils;

/**
 * helper class to constrain the frequency of a loop
 * */
public class MapleLoopClock {
    private final long periodNanos;
    public MapleLoopClock(double frequencyHZ) {
        final double periodSeconds = 1.0/frequencyHZ;
        this.periodNanos = (long) (periodSeconds * 1e9);
    }

    /**
     * constrains the frequency of the loop time
     * note that it does not just sleep to the period
     * */

    private long previousTimeNanos = System.nanoTime();
    public void tick() {
        final long timeElapsed = System.nanoTime() - previousTimeNanos;
        final int sleepNanos = (int) Math.max(0, periodNanos - timeElapsed);

        final long NANOS_IN_1_MILLIS = 1000000;
        try {
            Thread.sleep(sleepNanos / NANOS_IN_1_MILLIS, (int) (sleepNanos % NANOS_IN_1_MILLIS));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        previousTimeNanos = System.nanoTime();
    }
}
