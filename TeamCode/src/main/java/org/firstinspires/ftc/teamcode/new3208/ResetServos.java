package org.firstinspires.ftc.teamcode.new3208;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ResetServos extends LinearOpMode {
    ScoringMechanisms lu3 = new ScoringMechanisms(this);
    boolean lastWristButton = false;
    boolean lastArmButton = false;
    boolean wristUp = false;
    boolean armOut = false;
    int loops = 0;
    ElapsedTime s = new ElapsedTime();
    int lps = 0;
    @Override
    public void runOpMode(){
        lu3.init();
        waitForStart();
        while(opModeIsActive()) {
            loops++;

            if(!lastWristButton && gamepad1.b){
                wristUp = !wristUp;
            }

            if (!lastArmButton && gamepad1.a){
                armOut = !armOut;
            }

            if(armOut){
                lu3.setArmPosition(0);
            }
            else{
                lu3.setArmPosition(1);
            }

            if(wristUp){
                lu3.wrist.setPosition(lu3.WRIST_UPWARD_POSITION);
            }
            else{
                lu3.wrist.setPosition(lu3.WRIST_INTAKE_POSITION);
            }
            if(s.time() >= 1){
                s.reset();
                lps = loops;
                loops = 0;
            }
            telemetry.addData("arm out", armOut);
            telemetry.addData("button",gamepad1.a);
            telemetry.addData("kase",lastArmButton);
            telemetry.addData("looptime", lps);
            telemetry.update();
            lastWristButton = gamepad1.b;
            lastArmButton = gamepad1.a;

        }
    }
}
