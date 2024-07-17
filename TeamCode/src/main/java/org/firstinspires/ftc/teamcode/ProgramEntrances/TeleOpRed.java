package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.TeleOpRobot;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.OpModeUtils;

@TeleOp(name = "<Tele-Op>[Red]")
public class TeleOpRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final TeleOpRobot robot = new TeleOpRobot(
                new RobotCore(hardwareMap, AllianceSide.RED),
                gamepad1, gamepad2
        );

        OpModeUtils.runTeleOp(robot, this);
    }
}
