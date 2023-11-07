package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.DriveDistance;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.commands.RotateCommand;
import org.firstinspires.ftc.teamcode.vision.TeamShippingElementPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Config
public class VisionDriveAuto extends OpModeTemplate {

    private TeamShippingElementPipeline.Randomization randomization;
    private Point r1;
    private Point r2;
    private Point r3;
    public static double driveDistanceClose = 55;
    public static double driveDistanceFar = 48;
    public static double driveSpeed = 0.5;
    public static double rotateAngle = 70;
    public static double backStageDistance = 50;
    private final Alliance alliance;

    public VisionDriveAuto(Alliance alliance, Point r1, Point r2, Point r3){
        this.alliance = alliance;
        this.r1 = r1;
        this.r2 = r2;
        this.r3 = r3;
    }

    @Override
    public void initialize() {
        initHardware(true);
        OpenCvWebcam webcam;
        TeamShippingElementPipeline pipeline = new TeamShippingElementPipeline(
                alliance,
                r1,
                r2,
                r3);

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
                FtcDashboard.getInstance().startCameraStream(webcam, 20);
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

        double driveDist;
        double rotateDirection;
        double driveDirPark1 = 0;
        double driveDirPark2 = 0;
        randomization = pipeline.getRandomization();
        switch(randomization){
            case LOCATION_1:
                driveDist = driveDistanceClose;
                rotateDirection = 1;
                driveDirPark1 = -1;
                break;
            case LOCATION_2:
                driveDist = driveDistanceFar;
                rotateDirection = 0;
                driveDirPark2 = 1;
                break;
            case LOCATION_3:
                driveDist = driveDistanceClose;
                rotateDirection = -1;
                driveDirPark1 = 1;
                break;
            default:
                driveDist = driveDistanceClose;
                rotateDirection = 0;
                break;
        }
        schedule(
                new SequentialCommandGroup(
                new DriveDistance(driveDist,1,0,driveSpeed, drive),
                new RotateCommand(rotateAngle, rotateDirection, driveSpeed, drive),
                new IntakeCommand(-0.2, intake)
//                new DriveDistance(backStageDistance, driveDirPark1, driveDirPark2, driveSpeed, drive)
//                new DriveDistance(backStageDistance, 0, alliance.adjust(1), driveSpeed, drive)
        ));
    }
}
