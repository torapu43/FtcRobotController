package org.firstinspires.ftc.teamcode.avery;

import com.acmerobotics.dashboard.message.redux.StopOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Queue;

@Autonomous
public class FollowerTest extends LinearOpMode {

    //Objects
  SampleMecanumDrive drive;
  Line path;

  public void runOpMode(){
    drive = new SampleMecanumDrive(hardwareMap);
//    path = new Line()
//      .withStart(0, 0)
//      .withEnd(24, 24);
    path = new Line(
            new Vector2D(0, 0),
            new Vector2D(24, 24)
    );
    telemetry.addData("line", path.getControlPoints()[0].x);
    telemetry.update();

    while(opModeInInit()){
      drive.setPoseEstimate(new Pose2d(0,0,0));
    }

    waitForStart();
    if(opModeIsActive()){
      while(opModeIsActive()){
        drive.update();

        telemetry.addData("line", path.getControlPoints()[0].x);
        telemetry.addData("location", drive.getPoseEstimate());
        telemetry.update();
        
        Pose2d est = drive.getPoseEstimate();
        Pose2d follow = path.vector(est);
        drive.setWeightedDrivePower(follow);

      }
    }

  }
}
