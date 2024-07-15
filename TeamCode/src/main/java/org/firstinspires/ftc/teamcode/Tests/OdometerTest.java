package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.MapleLoopClock;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class OdometerTest implements SimpleUnitTest {
    private final DcMotor encoder1, encoder2, encoder3;
    private final IMU imu;
    private final Telemetry telemetry;
    private int encoderMillis;
    private final Queue<Integer> reading1;
    private Queue<Integer> reading2;
    private Queue<Integer> reading3;
    private static final MapleLoopClock mainThreadClock = new MapleLoopClock(50), odometerClock = new MapleLoopClock(200);
    private static final long imuUpdateMarginMillis = 200;
    private final Lock odometryLock = new ReentrantLock();
    private boolean running;
    public OdometerTest(HardwareMap hardwareMap, Telemetry telemetry) {
        encoder1 = hardwareMap.get(DcMotor.class, "backRight");
        encoder2 = hardwareMap.get(DcMotor.class, "frontRight");
        encoder3 = hardwareMap.get(DcMotor.class, "backLeft");
        this.imu = hardwareMap.get(IMU.class, "imu");
        this.telemetry = telemetry;

        encoderMillis = 0;
        reading1 = new ArrayBlockingQueue<>(10);
        reading2 = new ArrayBlockingQueue<>(10);
        reading3 = new ArrayBlockingQueue<>(10);
    }

    private void updateEncoders() {
        final long millis0 = System.currentTimeMillis();
        final int
                reading1Cached = encoder1.getCurrentPosition(),
                reading2Cached = encoder2.getCurrentPosition(),
                reading3Cached = encoder3.getCurrentPosition();
        final int elapsedMillis = (int) (System.currentTimeMillis() - millis0);

        odometryLock.lock();
        reading1.add(reading1Cached);
        reading2.add(reading2Cached);
        reading3.add(reading3Cached);
        odometryLock.unlock();
        this.encoderMillis = elapsedMillis;
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
        telemetry.addData("read time millis", encoderMillis);
        telemetry.addData("readings", reading1.toString() + "/" + reading2.toString() + "/" + reading3.toString());
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
