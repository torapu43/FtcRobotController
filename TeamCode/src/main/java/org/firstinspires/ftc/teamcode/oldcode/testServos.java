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
    int state = 0;
    ElapsedTime delay2 = new ElapsedTime();
    int scoring = 0;

    @Override
    public void runOpMode(){

        robot.init();




        waitForStart();
        while(opModeIsActive()){


            robot.setArmPosition(1-(.3*gamepad1.left_trigger));
            robot.openLowerClaw(true);
            robot.openUpperClaw(false);
            robot.wrist.setPosition(robot.WRIST_INTAKE_POSITION);

            telemetry.addData("wrist position:", robot.wrist.getPosition());
            telemetry.addData("arm position:", robot.getArmPosition());
            telemetry.update();

        }

    }
    public void scorePixel(){

    }

}
