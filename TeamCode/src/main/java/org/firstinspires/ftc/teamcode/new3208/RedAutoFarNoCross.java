package org.firstinspires.ftc.teamcode.new3208;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.oldcode.Lululu;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Timer;

@Autonomous
public class RedAutoFarNoCross extends LinearOpMode {



    AprilTagProcessor aprilTag;
    TfodProcessor tfod;
    VisionPortal myVisionPortal;

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model_20231206_154449.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Blue hat","Red hat"
    };

    private int objectPos = 1;

    static final double webcamMidX = 300;
    Lululu              robot = new Lululu(this);

    @Override
    public void runOpMode(){
        SampleMecanumDrive  drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPos = new Pose2d(-40.875, -64.4375, Math.toRadians(-90));

        drive.setPoseEstimate(startPos);

        Trajectory mid1 = drive.trajectoryBuilder(startPos,
                        true)
                .splineTo(new Vector2d(-36,-32),Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory mid2 = drive.trajectoryBuilder(mid1.end())
                .forward(10)
                .build();

        Trajectory mid3 = drive.trajectoryBuilder(mid2.end())
                .lineToLinearHeading(new Pose2d(-55,-45,Math.toRadians(180)))
                .build();

        Trajectory mid4 = drive.trajectoryBuilder(mid3.end())
                .lineToConstantHeading(new Vector2d(-55,-12))
                .build();

        Trajectory mid5 = drive.trajectoryBuilder(mid4.end())
                .lineToConstantHeading(new Vector2d(50,-12))
                .build();

        Trajectory mid6 = drive.trajectoryBuilder(mid5.end(),true)
                .lineToLinearHeading(new Pose2d(55,-42,Math.toRadians(180)))
                .addDisplacementMarker(() ->{
                    drive.setDrivePower(new Pose2d(0,0,0));
                    scorePixel();
                })
                .build();
        Trajectory mid7 = drive.trajectoryBuilder(mid6.end())
                .lineToConstantHeading(new Vector2d(54,-12))
                .build();

        Trajectory left1 = drive.trajectoryBuilder(startPos,true)
                .splineTo(new Vector2d(-47,-40),Math.toRadians(90))
                .build();

        Trajectory left2 = drive.trajectoryBuilder(left1.end())
                .forward(10)
                .build();
        Trajectory left3 = drive.trajectoryBuilder(left2.end())
                .lineToLinearHeading(new Pose2d(-60,-45,Math.toRadians(180)))
                .build();

        Trajectory left4 = drive.trajectoryBuilder(left3.end())
                .lineToConstantHeading(new Vector2d(-55,-12))
                .build();

        Trajectory left5 = drive.trajectoryBuilder(left4.end())
                .lineToConstantHeading(new Vector2d(50,12))
                .build();

        Trajectory left6 = drive.trajectoryBuilder(left5.end(),true)
                .lineToLinearHeading(new Pose2d(55,-36,Math.toRadians(180)))
                .addDisplacementMarker(() ->{
                    drive.setDrivePower(new Pose2d(0,0,0));
                    scorePixel();
                })
                .build();
        Trajectory left7 = drive.trajectoryBuilder(left6.end())
                .lineToConstantHeading(new Vector2d(54,-13))
                .build();

        Trajectory right1 = drive.trajectoryBuilder(startPos,true)
                .lineToConstantHeading(new Vector2d( -40,-55))
                .build();

        Trajectory right2 = drive.trajectoryBuilder(right1.end(),true)
                .lineToLinearHeading(new Pose2d(-28,-38,Math.toRadians(-130)))
                .build();
        Trajectory right3 = drive.trajectoryBuilder(right2.end())
                .forward(10)
                .build();
        Trajectory right4 = drive.trajectoryBuilder(right3.end())
                .lineToLinearHeading(new Pose2d(-55,-40,Math.toRadians(180)))
                .build();
        Trajectory right5 = drive.trajectoryBuilder(right4.end())
                .lineToConstantHeading(new Vector2d(-55,-12))
                .build();
        Trajectory right6 = drive.trajectoryBuilder(right5.end())
                .lineToConstantHeading(new Vector2d(50,-12))
                .build();
        Trajectory right7 = drive.trajectoryBuilder(right6.end(),true)
                .lineToLinearHeading(new Pose2d(56,-45,Math.toRadians(180)))
                .addDisplacementMarker(() ->{
                    drive.setDrivePower(new Pose2d(0,0,0));
                    scorePixel();
                })
                .build();
        Trajectory right8 = drive.trajectoryBuilder(right7.end())
                .forward(4)
                .build();
        Trajectory right9 = drive.trajectoryBuilder(right8.end())
                .lineToConstantHeading(new Vector2d(54,-13))
                .build();

        initDoubleVision();
        robot.init();

        while(opModeInInit()){
            objectPos = detectObject();
        }
        telemetry.addData("Ready for W", "");
        telemetry.update();



        waitForStart();

//        if(objectPos == 3) {
//            drive.followTrajectory(left1);
//            drive.followTrajectory(left2);

//        }
//        else if(objectPos == 2) {
            drive.followTrajectory(mid1);
            drive.followTrajectory(mid2);
//            drive.followTrajectory(mid3);

//        }
//        else{
//            drive.followTrajectory(right1);
//            drive.followTrajectory(right2);
//            drive.followTrajectory(right3);
//
//
//        }
//        robot.setLiftPosition(5000,.3);
//        robot.toScoringPosition();
//        robot.openLowerClaw(true);
//        robot.openLowerClaw(false);
//        robot.neutralPosition();
//        robot.setLiftPosition(0,.3);


//        if(objectPos == 1){
//
//        }
//        else{
//            drive.followTrajectory(toRight);
//            drive.followTrajectory(outRight);
//            drive.followTrajectory(scoreRight);
//        }
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

            if(x < 220){
                return 2;
            }
            if(x >220){
                return 3;
            }
        }   // end for() loop
        return 1;

    }

    public void scorePixel(){
        robot.neutralPosition(true);
        robot.openUpperClaw(false);
        robot.openLowerClaw(false);
        while(Math.abs(Math.abs(robot.getLiftPosition()) - 1200) > 100) {
            robot.setLiftPosition(1200, 1);
        }

        robot.setLiftPower(0);
        robot.toScoringPosition();
        sleep(700);

        robot.openLowerClaw(true);
        robot.openUpperClaw(true);
        sleep(100);

        robot.openUpperClaw(false);
        robot.openLowerClaw(false);
        robot.neutralPosition(true);
        sleep(500);

        while(Math.abs(robot.getLiftPosition()) > 100) {
            robot.setLiftPosition(0, 1);
        }

    }
}