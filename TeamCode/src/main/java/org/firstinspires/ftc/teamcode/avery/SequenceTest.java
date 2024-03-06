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
    
    sequence = new PathSequence(0, 0)
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
        Pose2d follow = sequence.follow(est);
        drive.setWeightedDrivePower(follow);
        telemetry.addData("location", drive.getPoseEstimate());
      }
    }

  }
}
