package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.vision.TeamShippingElementPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Autonomous
public class VisionTestAuto extends OpModeTemplate {

    private TeamShippingElementPipeline.Randomization randomization;
    public static double r1x = 0;
    public static double r1y = 0;
    public static double r2x = 50;
    public static double r2y = 0;
    public static double r3x = 100;
    public static double r3y = 0;

    @Override
    public void initialize() {
        initHardware(true);

        OpenCvWebcam webcam;
        TeamShippingElementPipeline pipeline = new TeamShippingElementPipeline(
                Alliance.RED,
                new Point(r1x, r1y),
                new Point(r2x, r2y),
                new Point(r3x, r3y));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // intentional noop
            }
        });

        while (!isStarted() && !isStopRequested()) {
            randomization = pipeline.getRandomization();

            telemetry.addData("Randomization", randomization);
            telemetry.update();
        }
        webcam.closeCameraDevice();

    }
}
