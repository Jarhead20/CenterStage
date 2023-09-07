package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;

public class AprilTagLocalizerSubsystem extends SubsystemBase implements Localizer{

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private final HardwareMap hMap;

    public AprilTagLocalizerSubsystem(HardwareMap hMap){
        this.hMap = hMap;
        initAprilTag();
    }

    private void initAprilTag() {
        AprilTagLibrary library = new AprilTagLibrary.Builder()
                .addTag(583, "MEOW",
                        0.099, new VectorF(0,1,0.2f), DistanceUnit.METER,
                        new Quaternion((float) (Math.PI/2), 0,0,1, 1))
                .addTag(584, "WOOF",
                        0.322, new VectorF(0,1,0.2f), DistanceUnit.METER,
                        Quaternion.identityQuaternion())
                .addTag(585, "OINK",
                        0.166, new VectorF(0,0,0.2f), DistanceUnit.METER,
                        Quaternion.identityQuaternion())
                .build();

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(library)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .setLensIntrinsics(1805.11209646, 1805.11209646, 1020.05252149, 743.423990613)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(1920, 1080));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        builder.enableCameraMonitoring(true);

        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    public void stopStream(){
        visionPortal.stopStreaming();
    }

    public void resumeStreaming(){
        visionPortal.resumeStreaming();
    }

    public void close(){
        visionPortal.close();
    }

    @Override
    public Twist2dDual<Time> update() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double xPos = 0;
        double xVel = 0;

        double yPos = 0;
        double yVel = 0;

        double headingPos = 0;
        double headingVel = 0;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                AprilTagPoseFtc apose = detection.ftcPose;

                float[] vec = detection.metadata.fieldPosition.added(new VectorF((float) detection.ftcPose.x, (float) detection.ftcPose.y, (float) detection.ftcPose.z)).getData();
                float angle = (float) (-detection.ftcPose.yaw);
                float[] tagPose = detection.metadata.fieldPosition.getData();
                double[] cameraPoints = rotatePoint(vec[0], vec[1], tagPose[0], tagPose[1], angle);
                xPos = cameraPoints[0];
                yPos = cameraPoints[1];
                headingPos = angle;
            }
        }   // end for() loop

        return new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (xPos), //x pos
                                (xPos)  //x vel
                                }),
                        new DualNum<Time>(new double[] {
                                (yPos), //y pos
                                (yPos)  //y vel
                        })),
                new DualNum<>(new double[]{
                        (headingPos), //heading pos
                        (headingPos)  //heading vel
                }));
    }

    public double[] rotatePoint(double rX, double rY, double cX, double cY, double angleInDegrees)
    {
        double angleInRadians = Math.toRadians(angleInDegrees);
        double cosTheta = Math.cos(angleInRadians);
        double sinTheta = Math.sin(angleInRadians);
        return new double[]{
                (cosTheta * (rX - cX) -
                        sinTheta * (rY - cY) + cX),
                (sinTheta * (rX - cX) +
                        cosTheta * (rY - cY) + cY)};
    }

}
