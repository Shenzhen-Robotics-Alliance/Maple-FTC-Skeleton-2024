package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Constants;

public class MapleTime {
    public static void wait(double seconds) {
        try {
            Thread.sleep((long)(seconds * 1000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public static double getMatchTimeSeconds() {
        final long timeMillis = System.currentTimeMillis() - Constants.matchStartTimeMillis;
        return timeMillis / 1000.0;
    }
}
