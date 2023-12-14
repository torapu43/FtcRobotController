package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class testLimSwitch extends LinearOpMode {
    private ColorSensor color;
    private boolean pixelIn;
    private boolean bottomPixelIn;
//    public testLimSwitch(ColorSensor color) {
//        this.color = color;
//    }
    @Override
    public void runOpMode(){
        color = hardwareMap.get(ColorSensor.class,"colorSensor");


        waitForStart();
        while(opModeIsActive()){
            pixelIn = color.blue() + color.green() + color.red() >= 1000;
            bottomPixelIn = color.blue() + color.green() + color.red() >= 500;
            telemetry.addData("blue", color. blue());
            telemetry.addData("green", color. green());
            telemetry.addData("red", color. red());
            telemetry.addData("pixel in",pixelIn);
            telemetry.addData("bottom pixel",bottomPixelIn);
            telemetry.update();
        }
    }
}
