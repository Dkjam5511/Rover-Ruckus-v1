package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled//(name = "Tfod Silver Autonomous", group = "TfodAutonomous")
public class TfodSilverAutonomous extends Nav_Routines {

    boolean goldfound = false;
    int leftcenterright = 1; // 1 = left, 2 = center, 3 = right
    double distancetraveledtodepot = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        winchup();
        go_forward(3, 0, -.2, false);
        go_sideways(90, 0, .5, 15);
        winchdown();
        go_forward(4,0,.4,false);
        goldfound = checktfod();
        if (!goldfound) {
            go_forward(13, 0, -.4, false);
            goldfound = checktfod();
            if (!goldfound) {
                go_forward(29, 0, .4, false);
            } else {
                leftcenterright = 3;
            }
        } else {
            leftcenterright = 2;
        }

        go_sideways(90, 0, .3, 5);  // Knock off the mineral
        sleep(500);
        go_sideways(270, 0, .3, 4);  // back out

        deactivateTfod();

        go_forward(14 + (12 * leftcenterright), 0, .35, false);

        turn_to_heading(315);
        go_sideways_to_wall(315, .5, 5, false);
        // go to depot
        wallfollow(28, 315, .4, 5, false, false);
        distancetraveledtodepot = wallfollow(14, 315, .2, 5, false, true);

        //drop the marker
        deploymarker2();

        // go to crater
        wallfollow(43 - distancetraveledtodepot, 315, -.4, 5, false, false);
        go_forward(14, 315, -.4, true);


    }
}
