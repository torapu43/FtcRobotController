package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class testLimSwitch extends LinearOpMode {
    static final double upperClawOpen = 1;
    static final double upperClawClosed = 0;
    static final double lowerClawOpen = 0;
    static final double lowerClawClosed = 1;

    private Servo lowerClaw;
    private Servo upperClaw;
    private Servo wrist;

    private ColorSensor color;
    private DistanceSensor distance;
    private boolean pixelIn;
    private boolean bottomPixelIn;

    private ElapsedTime delay = new ElapsedTime();
//    public testLimSwitch(ColorSensor color) {
//        this.color = color;
//    }
    @Override
    public void runOpMode(){
        color = hardwareMap.get(ColorSensor.class,"colorSensor");
        distance = hardwareMap.get(DistanceSensor.class,"colorSensor");
        upperClaw   = hardwareMap.get(ServoImplEx.class, "upperClaw");
        lowerClaw   = hardwareMap.get(ServoImplEx.class, "lowerClaw");
        wrist       = hardwareMap.get(ServoImplEx.class, "wrist");

        double pixelDist;
        boolean bothClosed;

        waitForStart();
        while(opModeIsActive()){
            pixelDist = distance.getDistance(DistanceUnit.CM);
            telemetry.addData("distance" , pixelDist);
            pixelIn = pixelDist <= 1;
            bottomPixelIn = pixelDist <= 2;

            bothClosed = pixelIn && !gamepad1.a && !gamepad1.b;
            openLowerClaw(!bottomPixelIn || gamepad1.b);
            openUpperClaw(!pixelIn || gamepad1.y);


            if(bothClosed || gamepad1.left_bumper){
                if(delay.time() > .5) {
                    openUpperClaw(false);
                    openLowerClaw(false);
                    wrist.setPosition(.2);
                }
            }
            else{
                delay.reset();
                wrist.setPosition(0);
            }

            telemetry.addData("blue", color. blue());
            telemetry.addData("green", color. green());
            telemetry.addData("red", color. red());
            telemetry.addData("pixel in",pixelIn);
            telemetry.addData("bottom pixel",bottomPixelIn);
            telemetry.update();
        }
    }
    public void openUpperClaw(boolean isOpen){
        if (isOpen) {
            upperClaw.setPosition(upperClawOpen);
        }
        else{
            upperClaw.setPosition(upperClawClosed);
        }
    }

    public void openLowerClaw(boolean isOpen){
        if(isOpen){
            lowerClaw.setPosition(lowerClawOpen);
        }
        else{
            lowerClaw.setPosition(lowerClawClosed);
        }
    }
}
