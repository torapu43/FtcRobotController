package org.firstinspires.ftc.teamcode.oldcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp

public class testMotors extends LinearOpMode{

    Lululu robot = new Lululu(this);

    @Override
    public void runOpMode() {

        double drive;
        double strafe;
        double turn;
        double speed;
        double angle;
        boolean fieldCentric = false;

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

        boolean armEnabled = true;


        robot.init();


        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a){
                robot.setMotorPower(.5, (DcMotorEx) hardwareMap.dcMotor.get("bl"));
            }
            else{
                robot.setMotorPower(0,(DcMotorEx) hardwareMap.dcMotor.get("bl"));
            }

            if(gamepad1.b){
                robot.setMotorPower(.5, (DcMotorEx) hardwareMap.dcMotor.get("br"));
            }
            else{
                robot.setMotorPower(0,(DcMotorEx) hardwareMap.dcMotor.get("br"));
            }


            if(gamepad1.x){
                robot.setMotorPower(.5, (DcMotorEx) hardwareMap.dcMotor.get("fl"));
            }
            else{
                robot.setMotorPower(0,(DcMotorEx) hardwareMap.dcMotor.get("fl"));
            }

            if(gamepad1.y){
                robot.setMotorPower(.5, (DcMotorEx) hardwareMap.dcMotor.get("fr"));
            }
            else{
                robot.setMotorPower(0,(DcMotorEx) hardwareMap.dcMotor.get("fr"));
            }

        }
    }


}