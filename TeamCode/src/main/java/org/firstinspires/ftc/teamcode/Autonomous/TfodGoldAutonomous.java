package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Tfod Gold Autonomous", group = "TfodAutonomous")
public class TfodGoldAutonomous extends Nav_Routines {

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
            go_forward(14, 0, -.4, false);
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

        turn_to_heading(135);
        go_sideways_to_wall(135, .5, 5, true);
        // go to depot
        wallfollow(28, 135, .4, 5, true, false );
        distancetraveledtodepot = wallfollow(14, 135, .4, 5, true, true);

        //drop the marker
        deploymarker2();

        // go to crater
        wallfollow(43 - distancetraveledtodepot, 135, -.4, 5, true, false);
        go_forward(14, 135, -.4, true);


    }
}
