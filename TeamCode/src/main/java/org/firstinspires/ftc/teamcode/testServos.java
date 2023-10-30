package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testServos extends LinearOpMode {
    Lululu robot = new Lululu(this);

    @Override
    public void runOpMode(){

        robot.init();



        waitForStart();
        while(opModeIsActive()){
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
//            Servo servo = robot.lowerClaw;
//            robot.testServo(servo, gamepad1.right_trigger);
//            telemetry.addData("servo position", servo.getPosition());
//            telemetry.update();

            robot.setArmPosition(1- gamepad1.right_trigger);


            robot.openUpperClaw(gamepad1.a);
            robot.openLowerClaw(gamepad1.b);


        }

    }
}
