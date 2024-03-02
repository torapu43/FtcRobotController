package org.firstinspires.ftc.teamcode.avery;

import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Queue;

@TeleOp
public class LocalizerTest extends LinearOpMode {

    //Objects
    SampleMecanumDrive drive;

    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);

        while(opModeInInit()){
          drive.setPoseEstimate(new Pose2d(0,0,0));
        }

        waitForStart();
        if(opModeIsActive()){
          while(opModeIsActive()){
            drive.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x));
            telemetry.addData("location", drive.getPoseEstimate());
          }
        }

    }
}
