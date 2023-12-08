package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class testLimSwitch extends LinearOpMode {

    DigitalChannel limitSwitch;
    @Override
    public void runOpMode(){
        limitSwitch = hardwareMap.get(DigitalChannel.class,"limitSwitch");

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Limit Switch pressed:",limitSwitch.getState());
            telemetry.update();
        }

    }
}
