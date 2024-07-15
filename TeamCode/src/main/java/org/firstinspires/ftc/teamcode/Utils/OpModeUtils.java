package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.function.BooleanSupplier;

public class OpModeUtils {
    public static void runOpMode(Robot robot, Runnable waitForStart, BooleanSupplier opModeIsActivated, BooleanSupplier isStopRequested) {
        waitForStart.run();
        Constants.matchStartTimeMillis = System.currentTimeMillis();
        while (opModeIsActivated.getAsBoolean() && !isStopRequested.getAsBoolean())
            robot.run();
    }
}
