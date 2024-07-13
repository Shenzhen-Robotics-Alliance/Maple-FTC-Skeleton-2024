package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Tests.MotorsMatch;
import org.firstinspires.ftc.teamcode.Tests.SimpleUnitTest;

@TeleOp(name = "UnitTest")
public class UnitTestEntrance extends OpMode {
    private final SimpleUnitTest test = new MotorsMatch(hardwareMap);
    @Override
    public void init() {
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

