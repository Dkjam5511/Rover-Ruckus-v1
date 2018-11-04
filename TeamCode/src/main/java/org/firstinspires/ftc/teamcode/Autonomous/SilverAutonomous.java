package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVariables;

@Autonomous(name = "Silver Autonomous", group = "Autonomous")
public class SilverAutonomous extends Nav_Routines {
    double timegoingsideways;

    boolean alphafound;
    boolean yellowfound;

    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        winchup();
        go_forward(3, 0, -.2, false, false, 0);
        go_sideways(90, 0, .5, 1.5, false);
        go_forward(12, 0, -.3, false, false, 0);
        winchdown();
        basealpha = (mineralcs.alpha());
        basealpha2 = (mineralcs2.alpha());
        alphafound = go_forward(8, 0, .2, false, true, 0);

        if (alphafound) {
            timegoingsideways = go_sideways(90, 0, .3, 1.5, true) * .8;
            yellowfound = mineralknock();
            go_sideways(270, 0, .3, timegoingsideways, false);
        }

        if (yellowfound) {
            go_forward(46, 0, .35, false, false, 0);
        } else {
            alphafound = go_forward(20, 0, .2, false, true, 6);
            if (alphafound) {
                timegoingsideways = go_sideways(90, 0, .3, 1.5, true) * .8;
                yellowfound = mineralknock();
                go_sideways(270, 0, .3, timegoingsideways, false);
            }
            if (yellowfound) {
                go_forward(28, 0, .35, false, false, 0);
            } else {
                alphafound = go_forward(20, 0, .2, false, true, 6);
                if (alphafound) {
                    timegoingsideways = go_sideways(90, 0, .3, 1.5, true)* .8;
                    yellowfound = mineralknock();
                    go_sideways(270, 0, .2, timegoingsideways, false);
                }
                go_forward(10, 0, .35, false, false, 0);
            }
        }

        turn_to_heading(315);
        go_sideways_to_wall(315,.5,4,false,false);
        // go to depot
        wallfollow(28,315,.4,4, false, false);
        wallfollow(18,315,.2,4, false, false);

        //drop the marker
        //deploymarker();

        // go to crater
        wallfollow(47,315,-.4,4, false, true);
        wallfollow(7,315,-.2,4, false, true);
    }
}
