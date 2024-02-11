/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.new3208;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */

public class ScoringMechanisms {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.


    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotorEx slideLeft, slideRight   = null;
    public ServoImplEx armRight, armLeft, wrist, lowerClaw, upperClaw, drone;
    private DigitalChannel limitSwitch;
    private DistanceSensor clawDetector;

    private AprilTagProcessor aprilTag;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;



    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode

    static final double armOutPosition = 0;
    static final double armInPosition = 1;
    static final double upperClawOpen = 1;
    static final double upperClawClosed = 0;
    static final double lowerClawOpen = 0;
    static final double lowerClawClosed = 1;
    static final double WRIST_INTAKE_POSITION = 0;
    static final double WRIST_UPWARD_POSITION = .25;
    static final double WRIST_SCORING_POSITION = 1;

    //SampleMecanumDrive drive = new SampleMecanumDrive(myOpMode.hardwareMap);





    // Define a constructor that allows the OpMode to pass a reference to itself.
    public ScoringMechanisms (LinearOpMode opmode) {

        myOpMode = opmode;
    }
    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).

        slideLeft   = myOpMode.hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight  = myOpMode.hardwareMap.get(DcMotorEx.class, "slideRight");


        upperClaw   = myOpMode.hardwareMap.get(ServoImplEx.class, "upperClaw");
        lowerClaw   = myOpMode.hardwareMap.get(ServoImplEx.class, "lowerClaw");
        armRight    = myOpMode.hardwareMap.get(ServoImplEx.class, "armRight");
        armLeft     = myOpMode.hardwareMap.get(ServoImplEx.class, "armLeft");
        wrist       = myOpMode.hardwareMap.get(ServoImplEx.class, "wrist");
        drone       = myOpMode.hardwareMap.get(ServoImplEx.class,"drone");

        limitSwitch = myOpMode.hardwareMap.get(DigitalChannel.class,"limitSwitch");
        clawDetector= myOpMode.hardwareMap.get(DistanceSensor.class, "colorSensor");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.


        disableArm();
        openLowerClaw(false);
        openUpperClaw(false);
        homeLift();
        wrist.setPosition(0);
        launchDrone(false);
        resetLiftEncoder();


        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public boolean isPixelIn(){

        return clawDetector.getDistance(DistanceUnit.CM) <= 1.8;
    }

    public void setArmPosition(double position){
        armRight.setPosition(position);
        armLeft.setPosition(1 - position);
    }
    public double getArmPosition() {
        return armRight.getPosition();
    }


    public void setLiftPosition(int position, double maxPower){
        int error = position - slideRight.getCurrentPosition();
        double kp = 0.01;

        double power = kp * error;

        if(Math.abs(power) > maxPower && power >= 0){
            slideLeft.setPower(-maxPower);
            slideRight.setPower(maxPower);
        }
        else if(Math.abs(power) > maxPower && power < 0){
            slideLeft.setPower(maxPower);
            slideRight.setPower(-maxPower);
        }
        else{
            slideLeft.setPower(-power);
            slideRight.setPower(power);
        }
    }

    public void resetLiftEncoder(){
        slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getLiftPosition(){
        return slideRight.getCurrentPosition();
    }

    public void setLiftPower(double power){
        slideLeft.setPower(power);
        slideRight.setPower(-power);
    }

    public void openUpperClaw(boolean isOpen){
        if (isOpen) {
            upperClaw.setPosition(upperClawOpen);
        }
        else{
            upperClaw.setPosition(upperClawClosed);
        }
    }

    public void openLowerClaw(boolean isOpen){
        if(isOpen){
            lowerClaw.setPosition(lowerClawOpen);
        }
        else{
            lowerClaw.setPosition(lowerClawClosed);
        }
    }

    public void disableArm(){
        armRight.setPwmDisable();
        armLeft.setPwmDisable();
    }

    public void enableArm(){
        armRight.setPwmEnable();
        armLeft.setPwmEnable();
    }

    public void flipArm(boolean toScoringPosition){
        if(toScoringPosition){
            armRight.setPosition(armOutPosition);
            armLeft.setPosition(1-armOutPosition);
            wrist.setPosition(1);
        }
        else{
            armRight.setPosition(armInPosition);
            armLeft.setPosition(1-armInPosition);
        }
    }
    public void wristIntakePosition(boolean toIntakePosition){
        if(toIntakePosition){
            wrist.setPosition(0.35);
        }
        else{
            wrist.setPosition(.1);
        }
    }
    public void launchDrone(boolean launch){
        if(launch){
            drone.setPosition(1);
        }
        else{
            drone.setPosition(0);
        }
    }

    public void neutralPosition(boolean wristUp){
        setArmPosition(1);

        if(wristUp){
            wrist.setPosition(0.1);
        }
        else{
            wrist.setPosition(0.35);
        }

    }

    //return true if lift is at home? not sure please check which one it is
    public boolean homeLift() {
        if(!limitSwitch.getState()){
            setLiftPower(-.5);
        }
        else{
            setLiftPower(0);
            resetLiftEncoder();

        }
        return !limitSwitch.getState();
    }

    public void wristOuttakeposition() {
        //wrist.setPosition( insert scoring position );
    }
}
