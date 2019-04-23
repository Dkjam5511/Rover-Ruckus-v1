package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalVariables;

@Autonomous(name = "Nav Routines Test", group = "Tests")
public class NavRoutinesTest extends Nav_Routines {
    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        //winchup();
       deploymineralarmextended(0);
       sleep(500);
       raisemineralarm();
       sleep(1000);
       deploymineralarmextended(GlobalVariables.LIFT_DROP_TICKS * .73);
       sleep(1000);
       raisemineralarm();
        sleep(1000);
       deploymineralarmextended(GlobalVariables.LIFT_DROP_TICKS * .73);
    }
}
