package org.firstinspires.ftc.teamcode.new3208;

import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Queue;

@Autonomous
public class RedAuto extends LinearOpMode {
    //Constants
    private static final double CLOSE_VISION_THRESHOLD = 320;
    private static final double FAR_VISION_THRESHOLD = 220;

    //Objects
    SampleMecanumDrive drive;
    ScoringMechanisms lu3;
    VisionSystem vision;

    //Match Variables
    boolean closeSide = true;
    boolean underTruss = true;
    int pixelPlacement = 1;
    boolean whitePixelsOnRight = false;
    int randomization = 2;

    public void runOpMode(){
        lu3 = new ScoringMechanisms(this);
        vision = new VisionSystem(this);
        drive = new SampleMecanumDrive(hardwareMap);
        vision.init();
        lu3.init();
        Pose2d startingPosition;
        TrajectorySequence purplePixel;
        if(closeSide){
            startingPosition = new Pose2d(16.875, -64.4375, Math.toRadians(-90));
        }
        else{
            startingPosition = new Pose2d(-40.875, -64.4375, Math.toRadians(-90));
        }
        drive.setPoseEstimate(startingPosition);
        while(opModeInInit()){
            //randomization = detectObject();
            if(gamepad1.dpad_left){
                randomization = 1;
            }
            else if(gamepad1.dpad_down || gamepad1.dpad_up){
                randomization = 2;
            }
            else if(gamepad1.dpad_right){
                randomization = 3;
            }
            telemetry.addData("randomization",randomization);
            telemetry.update();
        }
        switch(randomization) {
            case 1:
                //Hat on left
                if(closeSide) {
                    purplePixel = drive.trajectorySequenceBuilder(startingPosition)
                            .setReversed(true)
                            .lineToSplineHeading(new Pose2d(17, -32, Math.toRadians(180)))
                            .addTemporalMarker(() -> {
                                lu3.wrist.setPosition(lu3.WRIST_INTAKE_POSITION);
                            })
                            .lineToConstantHeading(new Vector2d(12, -32))
                            .addTemporalMarker(() -> {
                                lu3.openLowerClaw(true);
                                lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
                            })
                            .setReversed(false)
                            .build();
                }
                else{
                    purplePixel = drive.trajectorySequenceBuilder(startingPosition)
                            .setReversed(true)
                            .lineToSplineHeading(new Pose2d(-46, -36, Math.toRadians(-80)))
                            .lineToSplineHeading(new Pose2d(-50,-43,Math.toRadians(-90)))
                            .addTemporalMarker(() ->{
                                lu3.openLowerClaw(true);
                                lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
                                lu3.setArmPosition(lu3.ARM_4_PIXEL_HEIGHT);
                            })
                            .lineToLinearHeading(new Pose2d(-57,-36,Math.toRadians(180)))
                            .lineToLinearHeading(new Pose2d(-59,-36,Math.toRadians(180)))
                            .addTemporalMarker(() ->{
                                drive.setDrivePower(new Pose2d(0,0,0));
                                lu3.wrist.setPosition(lu3.WRIST_INTAKE_POSITION);
                                sleep(500);
                                lu3.openLowerClaw(false);
                                sleep(300);
                                lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
                                lu3.disableArm();
                            })
                            .build();

                }
                break;

            default:
                //Hat in middle
                if(closeSide) {
                    purplePixel = drive.trajectorySequenceBuilder(startingPosition)
                            .setReversed(true)
                            .lineToSplineHeading(new Pose2d(27, -24, Math.toRadians(180)))
                            .addTemporalMarker(1, () -> {
                                lu3.wrist.setPosition(lu3.WRIST_INTAKE_POSITION);
                            })
                            .addTemporalMarker(() -> {
                                lu3.openLowerClaw(true);
                                lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);

                            })
                            .setReversed(false)
                            .build();
                }
                else{
                    purplePixel = drive.trajectorySequenceBuilder(startingPosition)
                            .setReversed(true)
                            .lineToLinearHeading(new Pose2d(-40, -32, Math.toRadians(-110)))
                            .addTemporalMarker(() ->{
                                lu3.openLowerClaw(true);
                                lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
                                lu3.setArmPosition(lu3.ARM_4_PIXEL_HEIGHT);
                            })
                            .lineToLinearHeading(new Pose2d(-55,-38,Math.toRadians(180)))
                            .lineToConstantHeading(new Vector2d(-59,-38))
                            .addTemporalMarker(() ->{
                                lu3.wrist.setPosition(lu3.WRIST_INTAKE_POSITION);
                                sleep(700);
                                lu3.openLowerClaw(false);
                                sleep(300);
                                lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
                                lu3.disableArm();
                            })
                            .build();
                }
                break;
            case 3:
                //Hat on right
                if(closeSide) {
                    purplePixel = drive.trajectorySequenceBuilder(startingPosition)
                            .setReversed(true)
                            .lineToSplineHeading(new Pose2d(36, -32, Math.toRadians(180)))
                            .addTemporalMarker(1, () -> {
                                lu3.wrist.setPosition(lu3.WRIST_INTAKE_POSITION);
                            })
                            .addTemporalMarker(() -> {
                                lu3.openLowerClaw(true);
                                lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
                            })
                            .setReversed(false)
                            .build();
                }
                else {
                    purplePixel = drive.trajectorySequenceBuilder(startingPosition)
                            .setReversed(true)
                            .lineToSplineHeading(new Pose2d(-38, -40, Math.toRadians(-130)))
                            .lineToSplineHeading(new Pose2d(-34, -33, Math.toRadians(180)))
                            .addTemporalMarker(() -> {
                                lu3.openLowerClaw(true);
                                lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
                                lu3.setArmPosition(lu3.ARM_4_PIXEL_HEIGHT);
                            })
                            .lineToSplineHeading(new Pose2d(-55, -41,Math.toRadians(180)))
                            .lineToConstantHeading(new Vector2d(-59, -41))
                            .addTemporalMarker(() -> {
                                lu3.wrist.setPosition(lu3.WRIST_INTAKE_POSITION);
                                sleep(700);
                                lu3.openLowerClaw(false);
                                sleep(300);
                                lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
                                lu3.disableArm();
                            })
                            .build();
                }
                break;
        }

        waitForStart();
        if(opModeIsActive()){
            drive.followTrajectorySequence(purplePixel);
            TrajectorySequence scoreYellow;

            if(!closeSide){
                TrajectorySequence cross1;
                if(underTruss) {
                    cross1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-59, -12, Math.toRadians(180)))
                            .lineToConstantHeading(new Vector2d(30, -16))
                            .build();
                }
                else{
                    cross1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-59, -12, Math.toRadians(180)))
                            .lineToConstantHeading(new Vector2d(30, -16))
                            .build();
                }

                drive.followTrajectorySequence(cross1);

                switch(randomization){
                    case 1:
                        scoreYellow = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addTemporalMarker(() ->{
                                    armToBack();
                                })
                                .lineToConstantHeading(new Vector2d(52,-32))
                                .addTemporalMarker(()->{
                                    drive.setWeightedDrivePower(new Pose2d(0,0,0));

                                    sleep(500);
                                    lu3.openLowerClaw(true);
                                    lu3.openUpperClaw(true);
                                    lu3.armToFront();
                                    lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
                                    sleep(500);
                                })
                                .lineToConstantHeading(new Vector2d(54,-62))
                                .lineTo(new Vector2d(60,-62))
                                .build();
                        break;
                    case 2:
                        scoreYellow = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addTemporalMarker(() ->{
                                    armToBack();
                                })
                                .lineToConstantHeading(new Vector2d(52,-36))
                                .addTemporalMarker(()->{
                                    drive.setWeightedDrivePower(new Pose2d(0,0,0));

                                    sleep(500);
                                    lu3.openLowerClaw(true);
                                    lu3.openUpperClaw(true);
                                    lu3.armToFront();
                                    lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
                                    sleep(500);
                                })
                                .lineToConstantHeading(new Vector2d(54,-62))
                                .lineTo(new Vector2d(60,-62))
                                .build();
                        break;
                    default:
                        scoreYellow = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addTemporalMarker(this::armToBack)
                                .lineToConstantHeading(new Vector2d(52,-41))
                                .addTemporalMarker(()->{
                                    drive.setWeightedDrivePower(new Pose2d(0,0,0));

                                    sleep(500);
                                    lu3.openLowerClaw(true);
                                    lu3.openUpperClaw(true);
                                    lu3.armToFront();
                                    lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
                                    sleep(500);
                                })
                                .lineToConstantHeading(new Vector2d(54,-62))
                                .lineTo(new Vector2d(60,-62))
                                .build();
                        break;
                }
            }
            else{
                switch(randomization){
                    case 1:
                        double y;
                        if(pixelPlacement == 1){
                            y = -62;
                        }
                        else{
                            y = -65;
                        }
                        scoreYellow = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addTemporalMarker(() ->{
                                    armToBack();
                                })
                                .lineToConstantHeading(new Vector2d(52,-32))
                                .addTemporalMarker(()->{
                                    drive.setWeightedDrivePower(new Pose2d(0,0,0));

                                    sleep(500);
                                    lu3.openLowerClaw(true);
                                    lu3.openUpperClaw(true);
                                    lu3.armToFront();
                                    lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
                                    sleep(500);
                                })
                                .lineToConstantHeading(new Vector2d(54,-62))
                                .lineTo(new Vector2d(60,-62))
                                .build();
                        break;
                    case 2:
                        scoreYellow = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addTemporalMarker(() ->{
                                    armToBack();
                                })
                                .lineToConstantHeading(new Vector2d(52,-36))
                                .addTemporalMarker(()->{
                                    drive.setWeightedDrivePower(new Pose2d(0,0,0));

                                    sleep(500);
                                    lu3.openLowerClaw(true);
                                    lu3.openUpperClaw(true);
                                    lu3.armToFront();
                                    lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
                                    sleep(500);
                                })
                                .lineToConstantHeading(new Vector2d(54,-62))
                                .lineTo(new Vector2d(60,-62))
                                .build();
                        break;
                    default:
                        scoreYellow = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .addTemporalMarker(() ->{
                                    armToBack();
                                })
                                .lineToConstantHeading(new Vector2d(52,-41))
                                .addTemporalMarker(()->{
                                    drive.setWeightedDrivePower(new Pose2d(0,0,0));

                                    sleep(500);
                                    lu3.openLowerClaw(true);
                                    lu3.openUpperClaw(true);
                                    lu3.armToFront();
                                    lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
                                    sleep(500);
                                })
                                .lineToConstantHeading(new Vector2d(54,-62))
                                .lineTo(new Vector2d(60,-62))
                                .build();
                        break;
                }
            }
            drive.followTrajectorySequence(scoreYellow);
        }

    }
    private void selection(){
        if(gamepad1.a){
            if(gamepad1.right_bumper){
                closeSide = true;
            }
            else if(gamepad1.left_bumper){
                closeSide = false;
            }
        }
        if(gamepad1.b) {
            if (gamepad1.right_bumper) {
                underTruss = true;
            }
            else if (gamepad1.left_bumper) {
                underTruss = false;
            }
        }
        if(gamepad1.y) {
            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                pixelPlacement = 2;
            }
            else if (gamepad1.left_bumper) {
                pixelPlacement = 1;
            }
            else if(gamepad1.right_bumper){
                pixelPlacement = 3;
            }
        }
    }
    private void telemetrySelection(){
        String side;
        String whitePixelSide;
        String path;
        String pixelSide;
        if(closeSide){
            side = "Close";
        }
        else{
            side = "Far";
        }
        if(underTruss){
            path = "Truss";
        }
        else{
            path = "Wall";
        }
        if(whitePixelsOnRight){
            whitePixelSide = "Right";
        }
        else{
            whitePixelSide = "Left";
        }
        switch(pixelPlacement){
            case 1:
                pixelSide = "Left";
            case 3:
                pixelSide = "Right";
            default:
                pixelSide = "Middle";
        }

        telemetry.addData("(A) Side Chosen:", side);
        telemetry.addData("(B) Path Chosen", path);
        telemetry.addData("(Y) Yellow Pixel placement", pixelSide);
        telemetry.addData("Detection", randomization);
        //telemetry.addData("White Pixel Placement", whitePixelSide);
        telemetry.update();
    }
    private void armToBack(){
        lu3.armToBack();
        lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
        sleep(500);
        lu3.wrist.setPosition(lu3.WRIST_SCORING_POSITION);


    }
    private int detectObject(){
        telemetry.addData("Hat location",vision.hatLocation());
        if(closeSide){
            if(vision.hatLocation() < CLOSE_VISION_THRESHOLD){
                return 2;
            }
            else if(vision.hatLocation() == -1){
                return 1;
            }
            else{
                return 3;
            }
        }
        else{
            if(vision.hatLocation() < FAR_VISION_THRESHOLD){
                return 2;
            }
            else if(vision.hatLocation() == -1){
                return 3;
            }
            else{
                return 1;
            }
        }
    }
}
