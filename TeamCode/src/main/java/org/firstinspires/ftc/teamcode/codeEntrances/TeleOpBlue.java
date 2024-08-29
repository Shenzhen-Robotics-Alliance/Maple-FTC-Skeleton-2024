package org.firstinspires.ftc.teamcode.codeEntrances;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.TeleOpRobot;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.OpModeUtils;

@TeleOp(name="<Tele-Op>[Blue]")
public class TeleOpBlue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final TeleOpRobot robot = new TeleOpRobot(
                new RobotContainer(hardwareMap, AllianceSide.BLUE),
                gamepad1,gamepad2
        );

        OpModeUtils.runTeleOp(robot, this);
    }
}