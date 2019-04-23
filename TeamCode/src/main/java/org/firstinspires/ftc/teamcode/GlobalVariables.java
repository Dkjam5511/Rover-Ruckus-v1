package org.firstinspires.ftc.teamcode;

public class GlobalVariables{

    public static final boolean logging_10435 = true;

    public static final double MINERAL_BOX_ADJUSTMENT = -.01; // When new box servo is installed, should be the only variable that needs to be changed.  Accounts for difference where the servo spline grooves line up.
                                                             // Servo should be set to defaults for full range of motion (no right/left programming)
                                                             // It should be zero if box is perfectly square with the ground while servo tester is at middle pos.
                                                             // If you want the front edge of the box to tip up more, lower this
    public static final double MINERAL_BOX_GROUND_LEVEL = .50 - MINERAL_BOX_ADJUSTMENT; //the pos where the mineral box is square with the ground.  Everything based off of this position
    public static final double MINERAL_BOX_FULL_DROP = MINERAL_BOX_GROUND_LEVEL + .36;   // highest drop angle possible without tumbling the cubes
    public static final double MINERAL_BOX_HALF_DROP = MINERAL_BOX_GROUND_LEVEL + .28;   // enough angle for the ball to roll out, but the cube stays
    public static final double MINERAL_BOX_INTAKE_POS = MINERAL_BOX_GROUND_LEVEL + .02;  // the angle the box is at when we are intaking minerals
    public static final double MINERAL_BOX_BEGINLIFT_POS = MINERAL_BOX_GROUND_LEVEL + .35;  // the angle the box turns to when B is pressed
    public static final double MINERAL_BOX_LIFTED_POS = MINERAL_BOX_GROUND_LEVEL - .13;  // the angle the box ends up when the mineral arm has turned fully to the drop position
    public static final double MINERAL_BOX_BEGINSCOOP_POS = MINERAL_BOX_GROUND_LEVEL - .27;  // the angle the box is at when lowering into the crater
    public static final double MINERAL_BOX_HALFDROP_TURN_TIME = (MINERAL_BOX_HALF_DROP - MINERAL_BOX_LIFTED_POS);  // the number of seconds it takes to spin the the box to the half drop level
    public static final double MINERAL_BOX_HALFDROP_PAUSE_TIME = .5;  // the number of seconds to pause before full drop

    public static final double MINERAL_INTAKE_FORWARD_SPEED = -.88;  // VEX motors don't take the full range, so .88 seems to be the max
    public static final double MINERAL_INTAKE_REVERSE_SPEED = .88;  // VEX motors don't take the full range, so .88 seems to be the max

    public static final int MAX_MIL1_TICKS = 1880; // The average number of ticks when the mineral box is resting on the ground.  Can be +/- 10 ticks or so but MIL1_SCOOPING_TICKS (i.e. 96% of this) must be reached.

    public static final double MIL1_DROP_LEVEL_TICKS = MAX_MIL1_TICKS * .17;   // Arm angle for dropping minerals
    public static final double MIL1_SCOOPING_TICKS = MAX_MIL1_TICKS * .965;  // When Arm is lowering, angle where box scooping starts
    public static final double MIL1_FULL_SPEED_DEPLOY_PERCENTAGE = .29;  // Lowering the arm at full power until we reach this angle
    public static final double MIL1_FULL_SPEED_DEPLOY_TICKS = MAX_MIL1_TICKS * MIL1_FULL_SPEED_DEPLOY_PERCENTAGE;  // Lowering the arm at full power until we reach this angle
    public static final double MIL1_MEDIUM_SPEED_DEPLOY_TICKS = MAX_MIL1_TICKS * .8;  // Lowering the arm at lower power until we reach this angle
    public static final double MIL1_MEDIUM_SPEED_DEPLOY_POWER = -.35;  // Lowering the arm at lower power until we reach this angle
    public static final double MIL1_LOW_SPEED_DEPLOY_POWER = -.4;  // Lowering t/he arm at lower power until we reach this angle
    public static final double MIL1_BRAKING_DEPLOY_PERCENTAGE = .92;  // Begin braking the arm at this angle
    public static final double MIL1_BRAKING_DEPLOY_TICKS = MAX_MIL1_TICKS * MIL1_BRAKING_DEPLOY_PERCENTAGE;  // Begin braking the arm at this angle
    public static final double MIL1_MAX_TICKS_PER_SECOND = MAX_MIL1_TICKS * 1.2;  // Brake the arm at any time when it exceeds this speed
    public static final double MIL1_RAISING_FULL_SPEED_TICKS = MAX_MIL1_TICKS * .5;  // Raise the arm at full power until this angle is reached
    public static final double MIL1_RAISING_SLOWER_TICKS = MAX_MIL1_TICKS * .4;  //  Raise arm at medium power until this angle reached
    public static final double MIL1_RAISING_SLOWER_POWER = .4;  //  Raise arm at medium power until this angle reached
    public static final double MIL1_RAISING_SLOWEST_TICKS = MAX_MIL1_TICKS * .202;  // Raise the arm at low power until this angle is reached, then power it to hold it in place
    public static final double MIL1_RAISING_SLOWEST_POWER = .2;  // Raise the arm at low power until this angle is reached, then power it to hold it in place
    public static final double MIL1_HOLD_DROP_POSITION_POWER = .3;  // Raise the arm at low power until this angle is reached, then power it to hold it in place
    public static final double MIL1_TICKS_TO_TURN_BOX = MAX_MIL1_TICKS - MIL1_DROP_LEVEL_TICKS; // box must be fully turned between droplevelticks and droplevelticks + tickstoturnbox
    public static final double MIL1_CLOSE_SLIDE_BLOCK_TICKS = MAX_MIL1_TICKS * .637;  // When arm is lowered past this point, slide block servo will close

    public static final double SLIDE_BLOCK_OPEN_POS = .5;
    public static final double SLIDE_BLOCK_CLOSED_POS = .96;

    public static final double FILTER_SERVO_NORMAL = .74;
    public static final double FILTER_SERVO_DEPOT = .28;

    public static final int LIFT_DROP_TICKS = 1150;  // How high the lift must be extended so we can drop minerals.  This sould match the reading when the magneticl limit switch closes.
    public static final int LIFT_DEPOT_DROP_TICKS = 1350;
    public static final int LIFT_AUTO_RETRACT_TICKS = 200;  // How far to retract the lift while going forward or turning during autonomous
    public static final int LIFT_RAISING_RETRACT_TICKS = LIFT_DROP_TICKS / 2;  // retract lift this far when lift angle is raising
    public static final int LIFT_LOWERING_RETRACT_TICKS = LIFT_DROP_TICKS / 4;  // retract lift this far when lift angle is lowering
    public static final int LIFT_LOWERING_RETRACT_TICKS_BASE = LIFT_DROP_TICKS / 4;  // Don't change this.  Used for calculating the braking adjustment
    public static final int LIFT_TICKS_PER_INCH = 55;  // retract lift this far when lift angle is lowering
    public static final int WINCH_RETRACT_TICKS = 500;


}
