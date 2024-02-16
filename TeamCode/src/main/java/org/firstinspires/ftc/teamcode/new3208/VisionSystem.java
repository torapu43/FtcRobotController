package org.firstinspires.ftc.teamcode.new3208;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class VisionSystem {
    private OpMode opMode;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private TfodProcessor tfod;
    private TfodProcessor yellow;

    public static final Vector2d redLeft = new Vector2d(62.5,-30);
    public static final Vector2d redMiddle = new Vector2d(62.5,-36);
    public static final Vector2d redRight = new Vector2d(62.5,-42);
    public static final Vector2d blueLeft = new Vector2d(62.5,42);
    public static final Vector2d blueMiddle = new Vector2d(62.5,36);
    public static final Vector2d blueRight = new Vector2d(62.5,30);

    public static final int CAM_MIDDLE = 250;
    public static final double DESIRED_TAG_DISTANCE = 12;

    private static final String TFOD_MODEL_ASSET1 = "model_20231206_154449.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String TFOD_MODEL_ASSET2 = "model_20231206_154449.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "BlueHat","RedHat"
    };
    private static final String[] LABEL2 = {
            "Pixel"
    };
    public VisionSystem(OpMode opMode){
        this.opMode = opMode;
    }
    public void init(){
      aprilTag = new AprilTagProcessor.Builder()
              .setDrawAxes(true)
              .build();

      tfod = new TfodProcessor.Builder()
              .setModelAssetName(TFOD_MODEL_ASSET1)
              .setModelLabels(LABELS)
              .build();

      yellow = new TfodProcessor.Builder()
              .setModelAssetName(TFOD_MODEL_ASSET2)
              .setModelLabels(LABEL2)
              .build();

      visionPortal = new org.firstinspires.ftc.vision.VisionPortal.Builder()
              .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
              .addProcessors(tfod, aprilTag)
              .build();
    }
    public double getTagDistance(){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if(detections.size() != 0) {
            return detections.get(0).ftcPose.range;
        }
        else{
            return -1;
        }
    }
    public double getTagOffset(){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if(detections.size() != 0) {
            return detections.get(0).ftcPose.x;
        }
        else{
            return -1;
        }
    }
    public double getTagDistanceError(){
        return DESIRED_TAG_DISTANCE - getTagDistance();
    }
    public double getTagAngle(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        if(currentDetections.size() != 0) {
            return currentDetections.get(0).ftcPose.yaw;
        }
        else{
            return -1;
        }
    }
    public Pose2d getRobotLocation(){
        Vector2d tagLocation = redLeft;

        double x = tagLocation.getX() + getTagDistance();
        double y = tagLocation.getY();
        return new Pose2d();
    }
    public double getTagAngleError(){
        return 0-getTagAngle();
    }
    /**
     * Detects a team scoring element, returns the horizontal location in the camera view
     *
     * @return a double representing the horizontal location of the hat
     */
    public double hatLocation() {
        List<Recognition> recognitions = tfod.getRecognitions();
        Recognition recognition;
        opMode.telemetry.addData("# Objects Detected", recognitions.size());
        double x;
        if(recognitions.size() != 0) {
            double highestConfidence = recognitions.get(0).getConfidence();
            // Step through the list of recognitions and display info for each one
            recognition = recognitions.get(0);
            for (int i = 0; i < recognitions.size(); i++) {
                if (recognitions.get(i).getConfidence() > highestConfidence) {
                     recognition = recognitions.get(i);
                }
            }
            x = (recognition.getLeft() + recognition.getRight()) / 2;
        }
        else{
            x = -1;
        }

        return x;
    }
    /**
     *  Returns a boolean whether or not a yellow pixel is on the right side
     *
     * @return A boolean whether or not a yellow pixel is on the right side
     */
    public boolean pixelOnRight(){
        if(yellow.getRecognitions().size() != 0) {
            Recognition pixel = yellow.getRecognitions().get(0);
            double x = (pixel.getRight() + pixel.getLeft()) / 2;
            if(x > CAM_MIDDLE){
                return true;
            }
            else{
                return false;
            }
        }
        return true;
    }
    public void disableYellow(){
        visionPortal.setProcessorEnabled(yellow, false);
    }
    public void enableYellow(){
        visionPortal.setProcessorEnabled(yellow, true);
    }

    public void disableTfod(){
        visionPortal.setProcessorEnabled(tfod, false);
    }
    public void enableTfod(){
        visionPortal.setProcessorEnabled(tfod, true);
    }
    public void disableAprilTag(){
        visionPortal.setProcessorEnabled(aprilTag, false);
    }
    public void enableAprilTag(){
        visionPortal.setProcessorEnabled(aprilTag, true);
    }
}
