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
public class SequenceTest extends LinearOpMode {

    //Objects
  SampleMecanumDrive drive;
  PathSequence path;

  public void runOpMode(){
    drive = new SampleMecanumDrive(hardwareMap);
    
    path = new PathSequence(0, 0)
      .LineTo(24, 0)
      .SplineTo(new Spline()
                .withEnd(36, 12)
                .withControlPoint(1, 24, 0)
                .withControlPoint(2, 36, 12));

    drive.setPoseEstimate(new Pose2d(0,0,0));


    
    waitForStart();
    if(opModeIsActive()){
      while(opModeIsActive()){
        Pose2d est = drive.getPoseEstimate();
        Pose2d follow = path.follow(est);
        drive.setWeightedDrivePower(follow);
        telemetry.addData("location", drive.getPoseEstimate());
      }
    }

  }
}
