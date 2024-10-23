package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest implements SimpleUnitTest {
    final Servo servo;
    final Gamepad gamepad;

    public ServoTest(HardwareMap hardwareMap, Gamepad gamepad) {
        this.servo = hardwareMap.get(Servo.class, "TestServo");
        this.gamepad = gamepad;
    }

    @Override public void testStart() {}

    @Override
    public void testPeriodic() {
        servo.setPosition(0.5 - 0.5 * gamepad.left_stick_y);
    }
}
