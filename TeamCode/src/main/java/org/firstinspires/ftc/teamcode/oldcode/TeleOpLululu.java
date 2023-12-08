package org.firstinspires.ftc.teamcode.oldcode;

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
        boolean prevYButtonState = false;

        boolean lClawOpen = false;
        boolean prevAButtonState = false;

        //controls
        boolean upperClawButton = gamepad2.a;
        boolean lowerClawButton = gamepad2.b;



        boolean prevDButtonState = false;
        boolean prevBumperState = false;
        boolean scoringPosition = false;
        boolean prevDupButtonState = false;

        boolean armEnabled = false;
        boolean wristUp = true;


        robot.init();


        waitForStart();

        while (opModeIsActive()) {
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;

            angle = robot.getYaw();

            if (gamepad1.left_bumper)
                speed = .3;
            else if (gamepad1.right_bumper)
                speed = .7;
            else
                speed = .5;


            if (gamepad1.dpad_up && !prevDupButtonState) {
                fieldCentric = !fieldCentric;
            }
            prevDupButtonState = gamepad1.dpad_up;


            if (fieldCentric) {
                robot.driveFieldCentric(drive, strafe, turn, speed);
            } else {
                robot.driveByPower(drive, strafe, turn, speed);
            }

            if (gamepad1.dpad_down) {
                robot.resetPose();
            }

            robot.setLiftPower(gamepad2.left_stick_y);

            //if a is pressed and it was not pressed last loop, toggle claw
            if (gamepad2.y && !prevYButtonState) {
                uClawOpen = !uClawOpen;
            }
            prevYButtonState = gamepad2.y;
            robot.openUpperClaw(uClawOpen);

            //same but for the other claw
            if (gamepad2.a && !prevAButtonState) {
                lClawOpen = !lClawOpen;
            }
            prevAButtonState = gamepad2.a;
            robot.openLowerClaw(lClawOpen);

            //toggle whether the arm is disabled when dpad down is pressed
            if (gamepad2.dpad_down && !prevDButtonState) {
                armEnabled = !armEnabled;
            }
            prevDButtonState = gamepad2.dpad_down;

            if(gamepad2.left_stick_y != 0){
                armEnabled = true;
            }


            //holding left bumper puts arm and claw in pixel scoring position
            //holding dpad down disables arm (for use when arm is in down position)
            if (armEnabled) {
                robot.enableArm();
                if(gamepad2.left_bumper && !prevBumperState){
                    scoringPosition = !scoringPosition;
                    uClawOpen = false;


                    lClawOpen = false;
                }
                if (scoringPosition) {
                    robot.toScoringPosition();
                } else {

                    if (gamepad2.b || (!uClawOpen && !lClawOpen)){
                        wristUp = true;
                    }
                    else{
                        wristUp = false;
                    }
                    robot.neutralPosition(wristUp);


                }
                prevBumperState = gamepad2.left_bumper;
            }
            else {
                robot.disableArm();

                if (gamepad2.b || (!uClawOpen && !lClawOpen)){
                    robot.wrist.setPosition(0);
                }
                else{
                    robot.wrist.setPosition(.35);
                }
            }
            robot.setLiftPower((gamepad2.left_stick_y / 2));

            robot.updatePose();
            robot.addLiftPositions();
            telemetry.update();
        }
    }


}