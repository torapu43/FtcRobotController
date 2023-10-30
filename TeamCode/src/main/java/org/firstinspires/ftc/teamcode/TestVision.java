package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class TestVision extends LinearOpMode {
    Lululu driveTrain = new Lululu();
    VisionPortal visionPortal = new VisionPortal();



    @Override
    public void runOpMode(){
        visionPortal.telemetryTfod();
    }



    private void approachTag(int tagId){
        visionPortal.findAprilTag(tagId);
        driveTrain.driveByPower(visionPortal.getDrive(), visionPortal.getStrafe(),
                visionPortal.getTurn(), 1);
    }
}
