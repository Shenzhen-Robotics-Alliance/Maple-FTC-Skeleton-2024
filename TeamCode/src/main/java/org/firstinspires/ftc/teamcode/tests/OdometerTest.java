package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.MapleLoopClock;
import org.firstinspires.ftc.teamcode.utils.MapleOdometerWheels.MapleEncoder;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class OdometerTest implements SimpleUnitTest {
    private final MapleEncoder encoder1, encoder2, encoder3;
    private final IMU imu;
    private final Telemetry telemetry;
    private int encoderReadTimeMillis;
    private final Queue<Double> reading1, reading2, reading3;
    private static final MapleLoopClock mainThreadClock = new MapleLoopClock(50), odometerClock = new MapleLoopClock(200);
    private static final long imuUpdateMarginMillis = 200;
    private final Lock odometryLock = new ReentrantLock();
    private boolean running;
    public OdometerTest(HardwareMap hardwareMap, Telemetry telemetry) {
        encoder1 = new MapleEncoder(hardwareMap.get(DcMotor.class, "backRight"), false, 2048, 1, 1, 200);
        encoder2 = new MapleEncoder(hardwareMap.get(DcMotor.class, "frontRight"), false, 2048, 1, 1, 200);
        encoder3 = new MapleEncoder(hardwareMap.get(DcMotor.class, "backLeft"), false, 2048, 1, 1, 200);
        this.imu = hardwareMap.get(IMU.class, "imu");
        this.telemetry = telemetry;

        encoderReadTimeMillis = 0;
        reading1 = new ArrayBlockingQueue<>(10);
        reading2 = new ArrayBlockingQueue<>(10);
        reading3 = new ArrayBlockingQueue<>(10);
    }

    private double vel1 = 0, vel2 = 0, vel3 = 0;
    private void updateEncoders() {
        final long millis0 = System.currentTimeMillis();
        encoder1.poll();
        encoder2.poll();
        encoder3.poll();
        final int elapsedMillis = (int) (System.currentTimeMillis() - millis0);

        odometryLock.lock();
        reading1.add(encoder1.getRevolutionsSinceCalibration());
        reading2.add(encoder2.getRevolutionsSinceCalibration());
        reading3.add(encoder3.getRevolutionsSinceCalibration());
        vel1 = encoder1.getVelocityRevolutionsPerSecond();
        vel2 = encoder2.getVelocityRevolutionsPerSecond();
        vel3 = encoder3.getVelocityRevolutionsPerSecond();
        odometryLock.unlock();
        this.encoderReadTimeMillis = elapsedMillis;
        odometerClock.tick();
    }

    @Override
    public void testStart() {
        running = true;
        new Thread(() -> {
            while(running) updateEncoders();
        }).start();
    }

    private double yawDegrees = 0;
    private long yawMillis = 0;
    private void updateIMU() {
        final long millis0 = System.currentTimeMillis();
        yawDegrees = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        yawMillis = System.currentTimeMillis() - millis0;
    }
    private long previousYawUpdateMillis = System.currentTimeMillis();
    @Override
    public void testPeriodic() {
        if (System.currentTimeMillis() - previousYawUpdateMillis > imuUpdateMarginMillis - (20/2)) {
            updateIMU();
            previousYawUpdateMillis = System.currentTimeMillis();
        }

        odometryLock.lock();
        telemetry.addData("yaw", yawDegrees);
        telemetry.addData("yaw read time millis", yawMillis);
        telemetry.addData("encoder read time millis", encoderReadTimeMillis);
        telemetry.addData("readings", reading1.toString() + "/" + reading2.toString() + "/" + reading3.toString());

        telemetry.addData("vel1", vel1);
        telemetry.addData("vel1 unfiltered", encoder1.getUnfilteredVelocityRevolutionsPerSecond());

        telemetry.update();
        reading1.clear();
        reading2.clear();
        reading3.clear();
        odometryLock.unlock();

        mainThreadClock.tick();
    }

    @Override
    public void testEnd() {
        running = false;
    }
}
