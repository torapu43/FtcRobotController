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
public class LocalizerTest extends LinearOpMode {

    //Objects
  SampleMecanumDrive drive;
  Line path;

  public void runOpMode(){
    drive = new SampleMecanumDrive(hardwareMap);
    path = new Line()
      .withStart(0, 0)
      .withEnd(24, 24);

    while(opModeInInit()){
      drive.setPoseEstimate(new Pose2d(0,0,0));
    }

    waitForStart();
    if(opModeIsActive()){
      while(opModeIsActive()){
        Pose2D est = drive.getPoseEstimate();
        Vector2D vector = path.vector(est.getX(), est.getY());
        drive.setWeightedDrivePower(new Pose2D(vector.x, vector.y, 0));
        telemetry.addData("location", drive.getPoseEstimate());
      }
    }

  }
}
