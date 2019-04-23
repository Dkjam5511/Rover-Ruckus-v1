package org.firstinspires.ftc.teamcode.Autonomous;

import android.provider.Settings;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GlobalVariables;

@Autonomous(name = "SilverAutonomousFlexMore", group = "TfodAutonomous")
public class SilverAutonomousFlexMore extends Nav_Routines {

    int LCR;
    int mineralheading = 12;
    int mineraldistance = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        Nav_Init();
        LCR = checktfod2();
        if (LCR == 1){
            mineralheading = 345;
            mineraldistance = 4;
        } else if (LCR == 3){
            mineralheading = 40;
            mineraldistance = 9;
        }

        //Winch down and get away from the Lander
        winchup();  // note "winchup" lets the robot down and vice/versa
        go_sideways(270,0,.3,6);
        go_forward_retract(7,0,.7,false, 1, 0);

        //First cycle - sample mineral
        turn_to_heading(mineralheading);
        mineralintakeservo.setPower(-.88);
        deploymineralarm();
        sleep(100);  // gives the box time to turn
        extendmineralarm(mineraldistance * GlobalVariables.LIFT_TICKS_PER_INCH);
        raisemineralarm();
        turn_to_heading(0);
        go_forward(7,0, -.4, false);
        dumpmineral(false);

        //Second Cycle
        go_forward(11, 5, .4, false);
        deploymineralarmextended(GlobalVariables.LIFT_DROP_TICKS * .73);
        extendmineralarm(GlobalVariables.LIFT_DROP_TICKS * .73 + 5 * GlobalVariables.LIFT_TICKS_PER_INCH);
        raisemineralarm();
        go_forward(11, 7 ,-.4, false);
        dumpmineral(false);

        //Third Cycle
        go_forward(11, 355, .4, false);
        deploymineralarmextended(GlobalVariables.LIFT_DROP_TICKS * .73);
        extendmineralarm(GlobalVariables.LIFT_DROP_TICKS * .73 + 6 * GlobalVariables.LIFT_TICKS_PER_INCH);
        raisemineralarm();
        go_forward(11, 7 ,-.4, false);
        dumpmineral(false);

        //Go to depot
        go_forward(5, 0, .4, false);
        mineralintakeservo.setPower(0);
        gosidewaysretract(270,90,.7,28);
        go_forward_retract(30, 90, -1, false, 1, 0);
        go_forward_retract(30, 45, -1, false, 1, 26);
        sleep(300);

        //Go to crater
        go_forward_retract(43, 44, 1, false, 1, 0);
        mineralintakeservo.setPower(GlobalVariables.MINERAL_INTAKE_FORWARD_SPEED);
        deploymineralarm();
        extendmineralarm(GlobalVariables.LIFT_DROP_TICKS / 4);

        //Just in case shut down
        mil1.setPower(0);
        mil2.setPower(0);
        liftmotor.setPower(0);
        winchmotor.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        mineralintakeservo.setPower(0);
    }
}