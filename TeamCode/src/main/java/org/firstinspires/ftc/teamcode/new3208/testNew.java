package org.firstinspires.ftc.teamcode.new3208;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class testNew extends LinearOpMode{
    ScoringMechanisms lu3 = new ScoringMechanisms(this);
    @Override
    public void runOpMode(){
        lu3.init();
        waitForStart();
        while(opModeIsActive()){
                lu3.setLiftPower(gamepad1.left_stick_y);
            if(gamepad1.a){
                lu3.homeLift();
            }
                lu3.setArmPosition(0.1);

            lu3.climbRight.setPower(gamepad2.right_trigger + gamepad2.right_stick_y);
            lu3.climbLeft.setPower(gamepad2.left_trigger + gamepad2.right_stick_y);

            telemetry.addData("button pushed", lu3.getLimitSwitch());
            telemetry.update();
        }

    }
}
