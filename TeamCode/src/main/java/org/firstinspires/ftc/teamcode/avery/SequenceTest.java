package org.firstinspires.ftc.teamcode.avery;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class SequenceTest extends LinearOpMode {

    //Objects
  SampleMecanumDrive drive;
  PathSequence sequence;

  public void runOpMode(){
    drive = new SampleMecanumDrive(hardwareMap);
    
    sequence = new PathSequence(new Path[] {new Line(new Vector2D(0, 0),new Vector2D(24, 0)), new Spline(new Vector2D(24, 0),new Vector2D(24, 0),new Vector2D(36, 24), new Vector2D(36, 12))});

    drive.setPoseEstimate(new Pose2d(0,0,0));


    
    waitForStart();
    if(opModeIsActive()){
      while(opModeIsActive()){
        drive.update();
        Pose2d est = drive.getPoseEstimate();
        Pose2d follow = sequence.follow(est);
        drive.setWeightedDrivePower(follow);
        telemetry.addData("location", drive.getPoseEstimate());
        telemetry.update();
      }
    }

  }
}
