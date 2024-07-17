package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utils.MapleLoopClock;
import org.firstinspires.ftc.teamcode.Utils.RectangularRegionColorComparisonPipeLine;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import edu.wpi.first.math.MathUtil;

public class ColorDetectionTest implements SimpleUnitTest {
    private static final int width = 320, height = 240;
    private final OpenCvCamera webcam;
    private final Gamepad gamepad;
    private static final MapleLoopClock clock = new MapleLoopClock(24);
    private static final RectangularRegionColorComparisonPipeLine.RegionOfInterest[] ROIs = new RectangularRegionColorComparisonPipeLine.RegionOfInterest[] {
            new RectangularRegionColorComparisonPipeLine.RegionOfInterest(50, 50, 160, 120),
            new RectangularRegionColorComparisonPipeLine.RegionOfInterest(50, 50, 80, 120),
            new RectangularRegionColorComparisonPipeLine.RegionOfInterest(50, 50, 160+80, 120)
    };

    public ColorDetectionTest(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        final int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        this.gamepad = gamepad;
        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId
        );

        this.webcam.setPipeline(new RectangularRegionColorComparisonPipeLine(
                RectangularRegionColorComparisonPipeLine.ColorChannel.BLUE,
                telemetry,
                ROIs
        ));
        this.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                throw new RuntimeException("Error while opening camera, code: " + errorCode);
            }
        });
    }

    private int currentIndex = 0;
    private boolean prevA = false;
    @Override
    public void beforeStartPeriodic() {
        if (gamepad.a && (!prevA)) {
            currentIndex++;
            if (currentIndex >= ROIs.length)
                currentIndex = 0;
        }
        prevA = gamepad.a;
        clock.tick();

        final RectangularRegionColorComparisonPipeLine.RegionOfInterest ROI = ROIs[currentIndex];
        ROI.centerX += gamepad.left_stick_x;
        ROI.centerX = MathUtil.clamp(ROI.centerX, 0, width);
        ROI.centerY += gamepad.left_stick_y;
        ROI.centerY = MathUtil.clamp(ROI.centerY, 0, height);

        System.out.println("gamepad, LX" + gamepad.left_stick_x);
    }

    @Override
    public void testStart() {
    }

    @Override
    public void testPeriodic() {
        clock.tick();
    }
}
