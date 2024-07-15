package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Tests.MotorsMatch;
import org.firstinspires.ftc.teamcode.Tests.SimpleUnitTest;

@TeleOp(name = "UnitTest")
public class UnitTestEntrance extends OpMode {
    private SimpleUnitTest test;
    @Override
    public void init() {
        test = new MotorsMatch(hardwareMap ,gamepad1);
        test.testStart();
    }

    @Override
    public void loop() {
        test.testPeriodic();
    }

    @Override
    public void start() {
        test.testEnd();
    }
}

