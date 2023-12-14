package org.firstinspires.ftc.teamcode.new3208;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.oldcode.Lululu;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Timer;

@Autonomous
public class RedAutoClose extends LinearOpMode {



    AprilTagProcessor aprilTag;
    TfodProcessor tfod;
    VisionPortal myVisionPortal;

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model_20231206_154449.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "BlueHat","RedHat"
    };

    private int objectPos = 2;

    static final double webcamMidX = 300;
    Lululu              robot = new Lululu(this);
    @Override
    public void runOpMode(){
        SampleMecanumDrive  drive = new SampleMecanumDrive(hardwareMap);


        Pose2d startPos = new Pose2d(16.875, -64.4375, Math.toRadians(-90));
        Pose2d prePark = new Pose2d(53,38,Math.toRadians(-180));

        drive.setPoseEstimate(startPos);

        Trajectory toRight = drive.trajectoryBuilder(startPos,true)
                .splineTo(new Vector2d(15,-50),Math.toRadians(108)
                        //                , SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        //                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineTo(new Vector2d(7,-40),Math.toRadians(150)
                        //                , SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        //                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory outRight = drive.trajectoryBuilder(toRight.end(),false)
                .forward(6)
                .build();

        Trajectory scoreRight = drive.trajectoryBuilder(outRight.end(),false)
                .lineToLinearHeading(new Pose2d(58,-28,Math.toRadians(-180)))
                .addDisplacementMarker(() ->{
                    drive.setDrivePower(new Pose2d(0,0,0));
                    scorePixel();

                })
                .build();
        Trajectory park1Right = drive.trajectoryBuilder(scoreRight.end())
                .forward(3)
                .build();

        Trajectory park2Right = drive.trajectoryBuilder(park1Right.end())
                .lineToConstantHeading(new Vector2d(54,-13))
                .build();

        Trajectory toMiddle = drive.trajectoryBuilder(startPos,true)
                .splineTo(new Vector2d(12,-34),Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory outMiddle = drive.trajectoryBuilder(toMiddle.end())
                .splineToConstantHeading(new Vector2d(15,-45),Math.toRadians(-110))
                .build();

        Trajectory scoreMiddle = drive.trajectoryBuilder(outMiddle.end(),true)
                .lineToLinearHeading(new Pose2d(58,-34,Math.toRadians(-180)))
                .addDisplacementMarker(() ->{
                    drive.setDrivePower(new Pose2d(0,0,0));
                    scorePixel();
                })
                .build();

        Trajectory park1Middle = drive.trajectoryBuilder(scoreMiddle.end())
                .forward(3)
                .build();

        Trajectory park2Middle = drive.trajectoryBuilder(park1Middle.end())
                .lineToConstantHeading(new Vector2d(54,-13))
                .build();

        Trajectory toLeft = drive.trajectoryBuilder(startPos,true)
                .splineTo(new Vector2d(24,-40),Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory outLeft = drive.trajectoryBuilder(toLeft.end())
                .splineTo(new Vector2d(15,-55),Math.toRadians(-120))
                .build();

        Trajectory scoreLeft = drive.trajectoryBuilder(outLeft.end(),true)
                .lineToLinearHeading(new Pose2d(58,-43,Math.toRadians(-180)))
                .addDisplacementMarker(() ->{
                    drive.setDrivePower(new Pose2d(0,0,0));
                    scorePixel();
                })
                .build();
        Trajectory park1Left = drive.trajectoryBuilder(scoreLeft.end())
                .forward(3)
                .build();

        Trajectory park2Left = drive.trajectoryBuilder(park1Left.end())
                .lineToConstantHeading(new Vector2d(54,-13))
                .build();

        Trajectory park = drive.trajectoryBuilder(prePark,true)
                .lineToConstantHeading(new Vector2d(53,-64))

                .build();

        Trajectory park2 = drive.trajectoryBuilder(park.end(),true)
                .splineToLinearHeading(new Pose2d(60,-64),Math.toRadians(-180))
                .build();

        robot.init();
        initDoubleVision();

        while(opModeInInit()) {
            objectPos = detectObject();
        }

        telemetry.addData("Ready for W", "");
        telemetry.update();



        waitForStart();

//
//        drive.followTrajectory(toLeft);
//        drive.followTrajectory(outLeft);
//        drive.followTrajectory(scoreLeft);
//

        if(objectPos == 1){
            drive.followTrajectory(toLeft);
            drive.followTrajectory(outLeft);
            drive.followTrajectory(scoreLeft);
            drive.followTrajectory(park1Left);
            drive.followTrajectory(park2Left);
        }
        else if(objectPos == 2){
            drive.followTrajectory(toMiddle);
            drive.followTrajectory(outMiddle);
            drive.followTrajectory(scoreMiddle);
            drive.followTrajectory(park1Middle);
            drive.followTrajectory(park2Middle);
        }
        else{
            drive.followTrajectory(toRight);
            drive.followTrajectory(outRight);
            drive.followTrajectory(scoreRight);
            drive.followTrajectory(park1Right);
            drive.followTrajectory(park2Right);
        }

        robot.updatePose();
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
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.update();

            if (x < 320) {
                return 2;
            }
            if (x > 320) {
                return 3;
            }
        }   // end for() loop
        return 3;

//    }
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