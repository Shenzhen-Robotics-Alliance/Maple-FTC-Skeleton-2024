package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.function.BooleanSupplier;

public class OpModeUtils {
    public static void runOpMode(Robot robot, Runnable waitForStart, BooleanSupplier opModeIsActivated, BooleanSupplier isStopRequested, Telemetry driveStationTelemetry) {
        final MapleLoopClock loopClock = new MapleLoopClock(Constants.SystemConfigs.ROBOT_UPDATE_RATE_HZ);
        Constants.telemetry = new MultipleTelemetry(driveStationTelemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart.run();

        Constants.matchStartTimeMillis = System.currentTimeMillis();

        while (opModeIsActivated.getAsBoolean() && !isStopRequested.getAsBoolean()) {
            robot.run();
            driveStationTelemetry.update();
            loopClock.tick();
        }
        robot.reset(); // cancel the commands
        Constants.telemetry = new EmptyTelemetry(); // destroy the telemetry instance
    }
}
