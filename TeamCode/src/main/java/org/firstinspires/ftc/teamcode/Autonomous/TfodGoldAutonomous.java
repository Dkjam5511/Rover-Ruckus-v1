package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Tfod Gold Autonomous", group = "TfodAutonomous")
public class TfodGoldAutonomous extends Nav_Routines {

    double angletogold = -500; // -500 because its a impossible value
    int leftcenterright = 2; // 1 = left, 2 = center, 3 = right
    double leftcenterrightinches = 21;

    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
       // winchup();
        go_forward(3, 0, -.2, false, false, 0);
        go_sideways(90, 0, .5, 5, false);
        winchdown();
        angletogold = turn_to_heading_tfod(30);   // spin one way
        if (angletogold == -500) {
            angletogold = turn_to_heading_tfod(325);  // if not found spin the other
        }
        if (angletogold != -500) {
            if (angletogold < 340 && angletogold > 305) { // the left mineral
                leftcenterright = 1;
                leftcenterrightinches = 26;
                angletogold = angletogold + 3;
            }
            if (angletogold > 15 && angletogold < 50) {  // the right mineral
                leftcenterright = 3;
                leftcenterrightinches = 26;
                angletogold = angletogold - 3;
            }
            go_sideways(90, angletogold, .3, leftcenterrightinches, false);
            go_sideways(270, angletogold, .3, 10, false);
        } else {
            turn_to_heading(0);
            go_sideways(90, 0, .3, 19, false);
        }

        tfod.deactivate();

        turn_to_heading(0);
        go_forward(14 + (12 * leftcenterright), 0, .35, false, false, 0);

        turn_to_heading(135);
        go_sideways_to_wall(135,.5,7,true,false);
        // go to depot
        wallfollow(25,135,.4,6, true, false);
        wallfollow(18,135,.2,6, true, false);

        //drop the marker
        deploymarker();

        // go to crater
        wallfollow(42,135,-.4,6, true, true);
        wallfollow(7,135,-.2,6, true, true);
    }
}
