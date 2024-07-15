package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.function.BooleanSupplier;

public class OpModeUtils {
    public static void runOpMode(Robot robot, Runnable waitForStart, BooleanSupplier opModeIsActivated, BooleanSupplier isStopRequested, Telemetry telemetry) {
        final MapleLoopClock loopClock = new MapleLoopClock(Constants.SystemConfigs.robotUpdateRateHZ);
        Constants.telemetry = telemetry;

        waitForStart.run();

        Constants.matchStartTimeMillis = System.currentTimeMillis();

        while (opModeIsActivated.getAsBoolean() && !isStopRequested.getAsBoolean()) {
            robot.run();
            telemetry.update();
            loopClock.tick();
        }
        robot.reset(); // cancel the commands
        Constants.telemetry = new EmptyTelemetry(); // destroy the telemetry instance
    }
}
