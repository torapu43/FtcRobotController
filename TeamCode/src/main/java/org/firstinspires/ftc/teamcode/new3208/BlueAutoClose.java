package org.firstinspires.ftc.teamcode.new3208;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.oldcode.Lululu;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Timer;

@Autonomous
public class BlueAutoClose extends LinearOpMode {

    SampleMecanumDrive  drive = new SampleMecanumDrive(hardwareMap);
    Lululu              robot = new Lululu(this);

    AprilTagProcessor aprilTag;
    TfodProcessor tfod;
    VisionPortal myVisionPortal;

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model_20231121_000004.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Blue hat","Red hat"
    };

    private int objectPos = 3;
    private Timer timer = new Timer();


    static final double webcamMidX = 300;



    @Override
    public void runOpMode(){



        Pose2d startPos = new Pose2d(16.875, 64.4375, 90);

        drive.setPoseEstimate(startPos);

        Trajectory toRight = drive.trajectoryBuilder(startPos,true)
                .splineTo(new Vector2d(5,30),Math.toRadians(210))
                .build();

        Trajectory outRight = drive.trajectoryBuilder(toRight.end())
                .forward(10)

                .build();

        Trajectory scoreRight = drive.trajectoryBuilder(outRight.end())
                .lineToLinearHeading(new Pose2d(48,30,Math.toRadians(180)))
                .build();

        drive.trajectorySequenceBuilder(new Pose2d(16.875, 64.4375, Math.toRadians(90)))

                .setReversed(true)

//                                .splineTo(new Vector2d(24,30),Math.toRadians(-90))
//                                .setReversed(false)
//                                .forward(6)
//                                .lineToLinearHeading(new Pose2d(48,42,Math.toRadians(180)))

                //if middle(2)
                .splineTo(new Vector2d(12,30), Math.toRadians(-90))
                .setReversed(false)
                .forward(6)
                .lineToLinearHeading(new Pose2d(48,36,Math.toRadians(180)))

                //if right side (3)
//                                .splineTo(new Vector2d(5,30),Math.toRadians(210))
//                                .setReversed(false)
//                                .forward(6)
//                                .lineToLinearHeading(new Pose2d(48,30,Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-12,60),Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-12,12))
                .lineToConstantHeading(new Vector2d(-55,12))
                .forward(6)
                .back(7)
                .splineToConstantHeading(new Vector2d(-12,12),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(36,36),Math.toRadians(90))

                .build()

        while(opModeInInit()) {
            objectPos = detectObject();
        }

        telemetry.addData("Ready for W", "");
        telemetry.update();



        waitForStart();


        if(objectPos == 1){

        }
        else{
            drive.followTrajectory(toRight);
            drive.followTrajectory(outRight);
            drive.followTrajectory(scoreRight);
        }

    }



    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------


        myVisionPortal = new org.firstinspires.ftc.vision.VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))

                .addProcessors(tfod, aprilTag)
                .build();

    }   // end initDoubleVision()

    private int detectObject() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.update();

            if(x < 300){
                return 1;
            }
            if(x >300){
                return 2;
            }
        }   // end for() loop
        return 3;

    }
}