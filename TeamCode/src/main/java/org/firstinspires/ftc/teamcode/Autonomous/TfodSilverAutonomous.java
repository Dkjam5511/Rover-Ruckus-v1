package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Tfod Silver Autonomous", group = "TfodAutonomous")
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
        mineralknockservo.setPosition(.5);
        sleep(500);
        mineralknockservo.setPosition(1);
        go_sideways(270, 0, .3, 4);  // back out

        tfod.deactivate();

        go_forward(14 + (12 * leftcenterright), 0, .35, false);

        turn_to_heading(315);
        go_sideways_to_wall(315, .5, 3, false);
        // go to depot
        wallfollow(28, 315, .4, 3, false, false);
        distancetraveledtodepot = wallfollow(14, 315, .2, 3, false, true);

        //drop the marker
        deploymarker();

        // go to crater
        wallfollow(43 - distancetraveledtodepot, 315, -.4, 3, false, false);
        go_forward(11, 315, -.4, true);


    }
}
