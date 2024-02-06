package org.firstinspires.ftc.teamcode.new3208;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class NewTeleOp extends LinearOpMode {
    ScoringMechanisms lu3 = new ScoringMechanisms(this);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    ControlScheme controls = new ControlScheme(this);

    boolean armOut = false;
    boolean clawDown = false;
    boolean upperClawOpen = false;
    boolean lowerClawOpen = false;

    boolean lastArmButtonState;
    boolean lastWristButton;
    boolean liftHoming;

    ElapsedTime flipDelay = new ElapsedTime();

    @Override
    public void runOpMode(){
        controls.assign();
        lu3.init();

        waitForStart();
        while(opModeIsActive()) {

            driveFieldCentric();

            //toggle armOut if button is pressed
            if(controls.flipArm && !lastArmButtonState){
                armOut = !armOut;
            }
            lastArmButtonState = controls.flipArm;

            if(controls.closeClaws || lu3.isPixelIn()){
                clawDown = false;
            }



            //execute while the arm is set to be out
            if(armOut){
                //set arm to go out
                lu3.flipArm(true);
                lu3.wristOuttakeposition();

                //open claws when buttons pressed
                if(controls.openTop){
                    upperClawOpen = true;
                }
                if(controls.openBottom){
                    lowerClawOpen = true;
                }

                //when both pixels have been dropped, home the lift and flip the arm
                if(upperClawOpen && lowerClawOpen){
                    armOut = false;
                    lu3.homeLift();
                    clawDown = false;
                }
            }

            //execute while the arm is set to be in
            else{
                //set arm to go in
                lu3.flipArm(false);

                //close claws and flip up when button pressed
                if(clawDown) {
                    lu3.wristIntakePosition(false);
                    //add delay here so pixels dont get stuck
                    upperClawOpen = true;
                    lowerClawOpen = true;
                }
                else{
                    upperClawOpen = false;
                    lowerClawOpen = false;
                    //add delay here so wrist doesnt go up before gripping pixels
                    lu3.wristIntakePosition(true);
                }
            }

            lu3.openLowerClaw(lowerClawOpen);
            lu3.openUpperClaw(upperClawOpen);

            if(liftHoming){
                liftHoming = lu3.homeLift();
            }
            else {
                lu3.setLiftPower(controls.lift);
            }
        }
    }

    public void driveFieldCentric(){
        // Read pose
        Pose2d poseEstimate = RobotPose.currentPose;

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                controls.drive,
                -controls.strafe
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -controls.strafe
                )
        );

        // Update everything. Odometry. Etc.
        drive.update();
    }
}
