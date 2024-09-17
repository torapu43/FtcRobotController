package org.firstinspires.ftc.teamcode.avery;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous

public class GVFSplineTest extends LinearOpMode {
    //Objects
  SampleMecanumDrive drive;
  Spline path;

  public void runOpMode(){
    drive = new SampleMecanumDrive(hardwareMap);
    path = new Spline(
      new Vector2D(0, 0),
      new Vector2D(0, 10),
      new Vector2D(30, 30),
      new Vector2D(24, 24)
      );

    while(opModeInInit()){
      drive.setPoseEstimate(new Pose2d(0,0,0));
    }

    waitForStart();
    if(opModeIsActive()){
      while(opModeIsActive()){
        Pose2d est = drive.getPoseEstimate();
        Pose2d follow = path.vector(est);
        drive.setWeightedDrivePower(follow);
        telemetry.addData("location", drive.getPoseEstimate());
      }
    }

  }
}
