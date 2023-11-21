package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testServos extends LinearOpMode {
    Lululu robot = new Lululu(this);
    boolean enabled = true;
    boolean uClawOpen = false;
    boolean prevAButtonState = false;

    @Override
    public void runOpMode(){

        robot.init();



        waitForStart();
        while(opModeIsActive()){

        robot.setArmPosition(1- gamepad1.left_trigger);
        telemetry.addData("arm position", 1 - gamepad1.left_trigger);
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
