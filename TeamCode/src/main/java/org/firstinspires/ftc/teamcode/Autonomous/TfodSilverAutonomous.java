package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Scanner;

@Autonomous(name = "Tfod Silver Autonomous", group = "TfodAutonomous")
public class TfodSilverAutonomous extends Nav_Routines {

    boolean goldfound = false;
    int leftcenterright = 1; // 1 = left, 2 = center, 3 = right

    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        winchup();
        go_forward(3, 0, -.2, false, false, 0);
        go_sideways(90, 0, .5, 15, false);
        winchdown();
        go_forward(11, 0, -.4, false, false, 0);
        goldfound = checktfod();
        if (!goldfound) {
            go_forward(12, 0, .4, false, false, 0);
            goldfound = checktfod();
            if (!goldfound) {
                go_forward(12, 0, .4, false, false, 0);
            } else {
                leftcenterright = 2;
            }
        } else {
            leftcenterright = 3;
        }

        go_sideways(90, 0, .3, 5, false);  // Knock off the mineral
        mineralknockservo.setPosition(.5);
        sleep(500);
        mineralknockservo.setPosition(1);
        go_sideways(270, 0, .3, 4, false);  // back out

        tfod.deactivate();

        go_forward(14 + (12 * leftcenterright), 0, .35, false, false, 0);

        turn_to_heading(315);
        go_sideways_to_wall(315, .5, 7, false, false);
        // go to depot
        wallfollow(28, 315, .4, 6, false, false);
        wallfollow(18, 315, .2, 6, false, false);

        //drop the marker
        deploymarker();

        // go to crater
        wallfollow(42, 315, -.4, 6, false, true);
        wallfollow(7, 315, -.2, 6, false, true);


    }
}
