package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.new3208.ScoringMechanisms;

@TeleOp
public class testServos extends LinearOpMode {
    ScoringMechanisms robot = new ScoringMechanisms(this);
    boolean enabled = true;
    boolean uClawOpen = false;
    boolean prevAButtonState = false;
    ElapsedTime delay = new ElapsedTime();
    ElapsedTime delay2 = new ElapsedTime();

    @Override
    public void runOpMode(){

        robot.init();




        waitForStart();
        while(opModeIsActive()){



//            if(gamepad1.a) {
//                robot.launchDrone(true);
//            }
//            else{
//                robot.launchDrone(false);
//            }
//            robot.drone.setPosition(gamepad1.right_trigger);

            //robot.prong.setPosition(gamepad1.left_trigger);
            if(!gamepad1.a) {
                robot.wrist.setPosition(0);
                if(delay2.time() < 1.5) {
                    robot.setArmPosition(1);

                }
                else{
                    robot.disableArm();
                    robot.openLowerClaw(true);
                    robot.openUpperClaw(true);
                }
                delay.reset();
            }
            else{
                robot.openLowerClaw(false);
                robot.openUpperClaw(false);
                if(delay.time() > 1) {
                    robot.wrist.setPosition(1);
                }
                else{
                    robot.wrist.setPosition(.3);
                }
                robot.setArmPosition(.1);
                delay2.reset();
            }

            telemetry.addData("wrist position:", robot.wrist.getPosition());
            telemetry.addData("arm position:", robot.getArmPosition());
//        robot.setArmPosition(1- gamepad1.left_trigger);
//        telemetry.addData("arm position", 1 - gamepad1.left_trigger);
            telemetry.update();
//            robot.openLowerClaw(gamepad1.b);
//            robot.openUpperClaw(gamepad1.a);
//
//            if(gamepad1.left_bumper){
//                robot.setArmPosition(1);
//            }
//            else if(gamepad1.right_bumper){
//                robot.setArmPosition(0);
//            }
//
//            Servo servo = robot.wrist;
//            robot.testServo(servo, 1- (gamepad1.right_trigger/2));
//            telemetry.addData("servo position", servo.getPosition());
//            telemetry.update();

            //vertical up is 1 for wrist
            //hori = .9
            //-60 deg = .8

//            if(gamepad2.a && !prevAButtonState){
//                uClawOpen = !uClawOpen;
//            }
//            prevAButtonState = gamepad2.a;
//            robot.openUpperClaw(uClawOpen);
//
//
//
//            if (gamepad1.right_bumper) {
//                        robot.wrist.setPosition(1);
//                    }
//                    else if (gamepad1.left_bumper) {
//                    robot.wrist.setPosition(.8);
//                    }
//
//
//
//
//            robot.setLiftPower(gamepad1.right_stick_y/2);
//
//            robot.setArmPosition(1- gamepad1.right_trigger);
//
//            robot.openUpperClaw(gamepad1.a);
//            robot.openLowerClaw(gamepad1.b);


        }

    }
}
