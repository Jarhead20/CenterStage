package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.ArmAngleCommand;
import org.firstinspires.ftc.teamcode.commands.DriveDistance;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommand;
import org.firstinspires.ftc.teamcode.commands.OpModeTemplate;
import org.firstinspires.ftc.teamcode.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.ReleaseCommand;
import org.firstinspires.ftc.teamcode.commands.RotateCommand;
import org.firstinspires.ftc.teamcode.commands.StandbyCommand;
import org.firstinspires.ftc.teamcode.vision.TeamShippingElementPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Config
public class VisionDriveAuto extends OpModeTemplate {

    private TeamShippingElementPipeline.Randomization randomization;
    public static double r1x = 25;
    public static double r1y = 90;
    public static double r2x = 150;
    public static double r2y = 60;
    public static double r3x = 260;
    public static double r3y = 90;
    private Point r1 = new Point(r1x, r1y);
    private Point r2 = new Point(r2x, r2y);
    private Point r3 = new Point(r3x, r3y);

    public static double driveDistanceClose = 55;
    public static double driveDistanceFar = 48;
    public static double driveSpeed = 0.2;
    public static double rotateAngle = 50;
    public static double backStageDistance = 50;
    public static double strafeDistance = 10;
    public static double strafeOffset = 10;
    private final Alliance alliance;

    public VisionDriveAuto(Alliance alliance){
        this.alliance = alliance;
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
        double newRotateAngle;
        double newStrafeDist;
        randomization = pipeline.getRandomization();
        switch(randomization){
            case LOCATION_1:
                driveDist = driveDistanceClose;
                newRotateAngle = rotateAngle;
                newStrafeDist = strafeDistance-strafeOffset;
                break;
            case LOCATION_2:
                driveDist = driveDistanceFar;
                newRotateAngle = 0;
                newStrafeDist = strafeDistance;
                break;
            case LOCATION_3:
                driveDist = driveDistanceClose;
                newRotateAngle = strafeDistance+strafeOffset;
                newStrafeDist = 1;
                break;
            default:
                driveDist = driveDistanceClose;
                newRotateAngle = 0;
                newStrafeDist = strafeDistance;
                break;
        }
        schedule(
                new SequentialCommandGroup(
                    new StandbyCommand(lift, arm),
                    new DriveDistance(driveDist,1,0,driveSpeed, drive),
                    new RotateCommand(Math.toRadians(rotateAngle), 1, driveSpeed, drive),
                    new OuttakeCommand(lift, arm, true),
                    new ReleaseCommand(arm, true), // put the purple pixel in the left gripper
                    //new WaitCommand(500),
                    new LiftCommand(lift, 600),
                    new ArmAngleCommand(arm, 0.82, 0.4, false),
                    new RotateCommand(Math.toRadians(-90-newRotateAngle), alliance.adjust(1), driveSpeed, drive),
                    new DriveDistance(backStageDistance, -1, 0, driveSpeed, drive),
                    new DriveDistance(newStrafeDist, 0, 1, driveSpeed, drive),
                    new ReleaseCommand(arm,false) // put the yellow pixel in the right gripper
        ));
    }
}
