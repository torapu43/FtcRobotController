package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class liftPid extends LinearOpMode {
    Lululu robot = new Lululu(this);
    public void runOpMode(){
        robot.init();
        robot.neutralPosition(true);
        robot.resetLiftEncoder();
        robot.launchDrone(false);

        waitForStart();

        scorePixel();

        while(opModeIsActive()){

        }

    }

    public void scorePixel(){
            robot.neutralPosition(true);
            robot.openUpperClaw(false);
            robot.openLowerClaw(false);
            while(Math.abs(Math.abs(robot.getLiftPosition()) - 3400) > 100) {
                robot.setLiftPosition(3400, 1);
            }

            robot.setLiftPower(0);
            robot.toScoringPosition();
            sleep(700);

            robot.openLowerClaw(true);
            robot.openUpperClaw(true);
            sleep(100);

            robot.openUpperClaw(false);
            robot.openLowerClaw(false);
            robot.neutralPosition(true);
            sleep(500);

            while(Math.abs(robot.getLiftPosition()) > 100) {
                robot.setLiftPosition(0, 1);
            }

    }
}
