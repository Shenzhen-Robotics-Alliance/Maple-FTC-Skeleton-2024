package org.firstinspires.ftc.teamcode.codeEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousRobot;
import org.firstinspires.ftc.teamcode.autos.ExampleAuto;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;
import org.firstinspires.ftc.teamcode.Utils.OpModeUtils;

@Autonomous(name="<Auto>[Red] Example")
public class ExampleAutoRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final AutonomousRobot robot = new AutonomousRobot(
                new RobotContainer(hardwareMap, AllianceSide.RED),
                new ExampleAuto()
        );
        OpModeUtils.runAutoMode(robot, this);
    }
}
