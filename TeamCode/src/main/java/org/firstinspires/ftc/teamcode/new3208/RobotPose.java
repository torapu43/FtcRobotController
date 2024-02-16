package org.firstinspires.ftc.teamcode.new3208;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;

public class RobotPose {
    static Pose2d currentPose = new Pose2d();

    public static Pose2d get(){
        return currentPose;
    }
    public static void update(MecanumDrive drive){
        currentPose = drive.getPoseEstimate();
    }
}
