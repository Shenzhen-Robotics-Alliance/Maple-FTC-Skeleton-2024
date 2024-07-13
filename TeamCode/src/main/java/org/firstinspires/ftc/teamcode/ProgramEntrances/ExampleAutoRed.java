package org.firstinspires.ftc.teamcode.ProgramEntrances;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Autos.ExampleAuto;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.Utils.AllianceSide;

@Autonomous(name="<Auto>[Red] Example")
public class ExampleAutoRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final AutonomousRobot robot = new AutonomousRobot(
                new RobotCore(hardwareMap, AllianceSide.RED),
                new ExampleAuto()
        );

        waitForStart();

        while (opModeIsActive() && !isStopRequested())
            robot.run();
    }
}
