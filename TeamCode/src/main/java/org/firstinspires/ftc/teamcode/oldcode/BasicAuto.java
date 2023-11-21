package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Timer;

@Autonomous
public class BasicAuto extends LinearOpMode {

    Lululu robot = new Lululu(this);

    AprilTagProcessor   aprilTag;
    TfodProcessor       tfod;
    VisionPortal        myVisionPortal;

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "model_20231121_000004.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Blue hat","Red hat"
    };

    public int state = 0;
    private int objectPos = 3;
    private Timer timer = new Timer();


    static final double webcamMidX = 300;

    @Override
    public void runOpMode() {
        robot.init();
        initDoubleVision();

        objectPos = objectPosition();

        robot.armLeft.setPwmDisable();
        robot.armRight.setPwmDisable();
        robot.wrist.setPosition(0);

        waitForStart();
        while(opModeIsActive()) {

            if(state == 0) {
                robot.driveToPosition(1000, 1000, 0);
                state = 1;
            }

            telemetry.addData("object position:",objectPosition());
            telemetry.addData("timer",getRuntime());
            objectPos = objectPosition();

            telemetry.update();

            if(objectPosition() != 3 || getRuntime() > 10){
                state = 1;
            }

            if (state == 1) {
                if(objectPos == 1){

                }
                if(objectPos == 2){

                }
                else{

                }
            }
//
//            //detect objects in each section
//            if (objectPosition() == 1 && state == 1){
//                robot.driveCurve(-1000, -2000, .5,1);
//            }
//            else if(objectPosition() == 2 && state == 1){
//                robot.driveToPosition(0, -1000, 0, .5);
//            }
//            else if(state == 1){
//                robot.driveCurve(-2000,-1000,1,.5);
//            }




            //push pixel

            //position toward board

            //drive to apriltag

            //place pixel
        }



    }

    private boolean driveEncoder(int fwd, int strafe, double power){
        //drive to targeted position
        robot.driveToPosition(fwd,strafe,0,power);

        //if the robot is no longer moving, return true
        return !robot.isBusy();

    }

//    private boolean driveCurved()
//    {
//
//        robot.driveCurve()
//    }

    private int objectPosition() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        double objectX = -1000;
        double objectY = -1000;

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);

            objectX = x;
            objectY = y;

            telemetry.addData("X coord", objectX);

        }
        if (objectX > webcamMidX ) {
            return 1;
        }
        else if (objectX <= webcamMidX && objectX != -1000) {
            return 2;
        }
        else{
            return 3;
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

    private void telemetryTfod() {
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
            if(x > 300){
                telemetry.addData("Position:", "Right side");
            }
            else {
                telemetry.addData("Position:", "Left side");
            }
            telemetry.update();
        }   // end for() loop

    }   // end method telemetryTfod()


}

