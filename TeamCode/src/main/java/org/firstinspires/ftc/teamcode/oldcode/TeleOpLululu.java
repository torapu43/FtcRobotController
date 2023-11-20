package org.firstinspires.ftc.teamcode.ogCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class TeleOpLululu extends LinearOpMode{
    
    Lululu robot = new Lululu(this);
    
    @Override
    public void runOpMode() {
        
        double drive;
        double strafe;
        double turn;
        double speed;
        double angle;
        boolean fieldCentric = true;

        boolean uClawOpen = false;
        boolean prevAButtonState = false;

        boolean lClawOpen = false;
        boolean prevBButtonState = false;

        //controls
        boolean upperClawButton = gamepad2.a;
        boolean lowerClawButton = gamepad2.b;

        boolean prevDButtonState = false;
        boolean armEnabled = true;




        robot.init();
        
        
        waitForStart();
        
        while (opModeIsActive()){
            drive   = -gamepad1.left_stick_y;
            strafe  = gamepad1.left_stick_x;
            turn    = gamepad1.right_stick_x;

            angle   = robot.getYaw();

            if(gamepad1.left_bumper)
                speed = .3;
            else if(gamepad1.right_bumper)
                speed = .7;
            else
                speed = .5;

            if(gamepad1.dpad_up){
                fieldCentric = !fieldCentric;
            }

            if(fieldCentric) {
                robot.driveFieldCentric(drive, strafe, turn, speed);
            }
            else{
                robot.driveByPower(drive,strafe,turn,speed);
            }

            if (gamepad1.dpad_down){
                robot.resetImu();
            }

            robot.setLiftPower(gamepad2.left_stick_y);

            //if a is pressed and it was not pressed last loop, toggle claw
            if(gamepad2.a && !prevAButtonState){
                uClawOpen = !uClawOpen;
            }
            prevAButtonState = gamepad2.a;
            robot.openUpperClaw(uClawOpen);

            //same but for the other claw
            if(gamepad2.b && !prevBButtonState){
                lClawOpen = !lClawOpen;
            }
            prevBButtonState = gamepad2.b;
            robot.openLowerClaw(lClawOpen);

            //idk if this toggle code works lmao
//            if(gamepad2.dpad_down && !prevDButtonState){
//                armEnabled = !armEnabled;
//            }
//            prevDButtonState = gamepad2.dpad_down;
//
            if(gamepad2.dpad_down){
                armEnabled = false;
            }
            else{
                armEnabled = true;
            }


            //holding left bumper puts arm and claw in pixel scoring position
            //holding dpad down disables arm (for use when arm is in down position)
            if(armEnabled) {
                robot.enableArm();
                if (gamepad2.left_bumper) {
                    robot.setArmPosition(0);
                    robot.wrist.setPosition(.8);
                } else {
                    robot.setArmPosition(1);
                    robot.wrist.setPosition(0.9);
                }
            }
            else{
                robot.disableArm();
            }
            robot.setLiftPower((gamepad2.left_stick_y/2) + 0.05);

            telemetry.addData("Robot Angle (fs)", angle);
            telemetry.update();
        }
    }


}