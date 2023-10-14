package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagLocalizerSubsystem extends SubsystemBase {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private final HardwareMap hMap;

    public AprilTagLocalizerSubsystem(HardwareMap hMap){
        this.hMap = hMap;
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
                .setLensIntrinsics(1389.80870649, 1389.80870649, 663.268596171, 399.045042197)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableCameraMonitoring(true);

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

}
