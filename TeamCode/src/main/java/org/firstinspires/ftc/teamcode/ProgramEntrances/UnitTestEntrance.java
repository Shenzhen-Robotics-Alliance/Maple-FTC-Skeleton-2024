package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Tests.MotorsMatch;
import org.firstinspires.ftc.teamcode.Tests.OdometerTest;
import org.firstinspires.ftc.teamcode.Tests.OdometryCalibration;
import org.firstinspires.ftc.teamcode.Tests.SimpleUnitTest;

@TeleOp(name = "UnitTest")
public class UnitTestEntrance extends OpMode {
    private SimpleUnitTest test;
    @Override
    public void init() {
        final Telemetry multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        test = new OdometryCalibration(hardwareMap, multipleTelemetry, gamepad1);
    }

    @Override
    public void loop() {
        test.testPeriodic();
    }

    @Override
    public void start() {
        test.testStart();
    }

    @Override
    public void stop() {
        test.testEnd();
    }
}

