package org.firstinspires.ftc.teamcode.codeEntrances;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tests.MotorsMatch;
import org.firstinspires.ftc.teamcode.tests.SimpleUnitTest;

@TeleOp(name = "UnitTest")
public class UnitTestEntrance extends OpMode {
    private SimpleUnitTest test;
    @Override
    public void init() {
        // test = new OdometryCalibration(hardwareMap, SystemConstants.telemetry, gamepad1);
        // test = new ServoTest(hardwareMap, gamepad1);
        test = new MotorsMatch(hardwareMap, gamepad1);
    }

    @Override
    public void init_loop() {
        test.beforeStartPeriodic();
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

