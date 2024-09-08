package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutonomousRobot;
import org.firstinspires.ftc.teamcode.constants.SystemConstants;
import org.firstinspires.ftc.teamcode.TeleOpRobot;

import java.util.function.BooleanSupplier;

public class OpModeUtils {
    public static void runAutoMode(AutonomousRobot robot, LinearOpMode opModeEntrance) {
        final Runnable waitForStart = () -> {
            while (!opModeEntrance.isStarted() && !opModeEntrance.isStopRequested())
                robot.beforeStartPeriodic();
            robot.startAuto();
        };
        runOpMode(robot, waitForStart, opModeEntrance::opModeIsActive, opModeEntrance::isStopRequested, opModeEntrance.telemetry);
    }
    public static void runTeleOp(TeleOpRobot robot, LinearOpMode opModeEntrance) {
        runOpMode(robot, opModeEntrance::waitForStart, opModeEntrance::opModeIsActive, opModeEntrance::isStopRequested, opModeEntrance.telemetry);
    }
    private static void runOpMode(Robot robot, Runnable waitForStart, BooleanSupplier opModeIsActivated, BooleanSupplier isStopRequested, Telemetry driveStationTelemetry) {
        final MapleLoopClock loopClock = new MapleLoopClock(SystemConstants.ROBOT_UPDATE_RATE_HZ);
        SystemConstants.telemetry = new MultipleTelemetry(driveStationTelemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart.run();

        SystemConstants.matchStartTimeMillis = System.currentTimeMillis();

        while (opModeIsActivated.getAsBoolean() && !isStopRequested.getAsBoolean()) {
            robot.run();
            SystemConstants.telemetry.update();
            loopClock.tick();
        }
        robot.reset(); // cancel the commands
    }
}
