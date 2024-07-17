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

public class ColorDetectionTest implements SimpleUnitTest {
    private final OpenCvCamera webcam;
    private final Gamepad gamepad;
    private static final MapleLoopClock clock = new MapleLoopClock(20);

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
                new RectangularRegionColorComparisonPipeLine.RegionOfInterest(50, 50, 160, 120),
                new RectangularRegionColorComparisonPipeLine.RegionOfInterest(50, 50, 80, 120),
                new RectangularRegionColorComparisonPipeLine.RegionOfInterest(50, 50, 160+80, 120)
        ));
        this.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                throw new RuntimeException("Error while opening camera, code: " + errorCode);
            }
        });
    }

    @Override
    public void beforeStartPeriodic() {
        // TODO: use gamepad to control the position of the ROIs
    }

    @Override
    public void testStart() {
    }

    @Override
    public void testPeriodic() {
        clock.tick();
    }
}
