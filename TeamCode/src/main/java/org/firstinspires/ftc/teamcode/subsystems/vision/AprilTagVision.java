package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.Closeable;
import java.io.IOException;
import java.util.List;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class AprilTagVision implements Subsystem, Closeable {
    public static final long AVERAGE_LATENCY_MILLIS = 50;
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;

    public AprilTagVision(WebcamName webcamName) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setNumThreads(4)
                .setLensIntrinsics(514.826, 514.826, 311.864, 176.19)
                .setDrawCubeProjection(true)
                .build();

        aprilTagProcessor.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .setCameraResolution(new Size(640, 400))
                .addProcessor(aprilTagProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 5);
    }

    private boolean manualExposureConfigured = false;
    @Override
    public void periodic() {
        manualExposureConfigured = manualExposureConfigured
                | setManualExposure(3, 250);

        final List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
    }

    /**
     *
     * */
    private boolean setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return false;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
            return false;

        // Set camera controls unless we are stopping.
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual)
            exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);

        return true;
    }

    @Override
    public void close() throws IOException {
        visionPortal.close();
    }

    private static Transform3d getCameraToTarget(AprilTagPoseFtc aprilTagPoseFtc) {
        return new Transform3d(
                aprilTagPoseFtc.x,
                aprilTagPoseFtc.y,
                aprilTagPoseFtc.z,
                new Rotation3d(aprilTagPoseFtc.roll, aprilTagPoseFtc.pitch, aprilTagPoseFtc.yaw)
        );
    }
}
