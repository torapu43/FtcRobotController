package org.firstinspires.ftc.teamcode.new3208;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ControlScheme{
    private LinearOpMode opMode = null;
    public double drive;
    public double strafe;
    public double turn;
    public double lift;

    public boolean closeClaws;
    public boolean openTop;
    public boolean openBottom;

    public boolean flipArm;
    public boolean grabStack;

    public ControlScheme (LinearOpMode opMode) {
        this.opMode = opMode;

        drive       = -opMode.gamepad1.left_stick_y;
        strafe      = opMode.gamepad1.left_stick_x;
        turn        = opMode.gamepad1.right_stick_x;
        grabStack   = opMode.gamepad1.a;


        lift        = -opMode.gamepad2.left_stick_y;
        closeClaws  = opMode.gamepad2.b;
        openTop     = opMode.gamepad2.y;
        openBottom  = opMode.gamepad2.a;
        flipArm     = opMode.gamepad2.left_bumper;

    }
    public void update(){
        drive       = -opMode.gamepad1.left_stick_y;
        strafe      = opMode.gamepad1.left_stick_x;
        turn        = opMode.gamepad1.right_stick_x;
        grabStack   = opMode.gamepad1.a;


        lift        = -opMode.gamepad2.left_stick_y;
        closeClaws  = opMode.gamepad2.b;
        openTop     = opMode.gamepad2.y;
        openBottom  = opMode.gamepad2.a;
        flipArm     = opMode.gamepad2.left_bumper;

    }


}
