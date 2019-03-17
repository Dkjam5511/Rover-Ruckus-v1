package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVariables;

@Autonomous(name = "Tfod Silver Autonomous Flex", group = "TfodAutonomous")
public class TfodSilverAutonomousFlex extends Nav_Routines {

    int LCR;
    int mineralheading = 7;
    int mineraldistance = 4;
    double distancetraveledtodepot = 0;
    
    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        LCR = checktfod2();
        winchup();
        go_sideways(270,0,.3,4);
        go_forward(4,0,.4,false);
        winchdown();
        if (LCR == 1){
            mineralheading = 340;
            mineraldistance = 6;
        } else if (LCR == 3){
            mineralheading = 30;
            mineraldistance = 8;
        }
        turn_to_heading(mineralheading);
        mineralintakeservo.setPower(0);
        deploymineralarm();
        extendmienralarm();
        go_forward(mineraldistance, mineralheading, .3,false);
        raisemineralarm();
        go_forward(mineraldistance + 4, mineralheading, -.3, false);
        turn_to_heading(10);
        dumpmineral();
        mineralintakeservo.setPower(.5);
        go_forward(14.5,0,.4,false);
        gosidewaysretract(270,0, .55, 30);
        turn_to_heading(45);
        // go to depot
        go_sideways_to_wall(45,.45,3.5, true);
        wallfollow(28, 45, -.45, 3.5, true, false);
        distancetraveledtodepot = wallfollow(14, 45, -.35, 3.5, true, true);

        //drop the marker
        deploymarker2();

        // go to crater
        wallfollow(43 - distancetraveledtodepot, 45, .45, 3.5, true, false);
        go_forward(14, 45, .4, true);

        mineralintakeservo.setPower(0);
        deploymineralarm();
        extendmienralarm();
    }
}
