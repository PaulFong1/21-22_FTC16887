package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
// Created for 16887.
@TeleOp(name="Test TeleOp", group="_Other")
//@Disabled
public class TestTeleOp extends BaseRobot {
    @Override
    public void init() {
        super.init();
    }
    @Override
    public void start() {
        super.start();
        DEBUG = true;                  // Start in debug mode
    }
    @Override
    public void loop() {
/*        if (gamepad1.left_bumper) {    // Using the power adjustment factors to balance the motors
            if (gamepad1.x) leftBack.setPower(ConstantVariables.K_LB_ADJUST);
            else leftBack.setPower(0);
            if (gamepad1.y) rightBack.setPower(ConstantVariables.K_RB_ADJUST);
            else rightBack.setPower(0);
            if (gamepad1.a) leftFront.setPower(ConstantVariables.K_LF_ADJUST);
            else leftFront.setPower(0);
            if (gamepad1.b) rightFront.setPower(ConstantVariables.K_RF_ADJUST);
            else rightFront.setPower(0);
        } else {
            if (gamepad1.x) leftBack.setPower(1);
            else leftBack.setPower(0);
            if (gamepad1.y) rightBack.setPower(1);
            else rightBack.setPower(0);
            if (gamepad1.a) leftFront.setPower(1);
            else leftFront.setPower(0);
            if (gamepad1.b) rightFront.setPower(1);
            else rightFront.setPower(0);
        }

 */

        // Turn the top spinning wheel: left for clockwise and right for anticlockwise
        if (gamepad1.left_bumper)       topSpin.setPower(1);
        else if (gamepad1.right_bumper) topSpin.setPower(-1);
        else                            topSpin.setPower(0);

        if (gamepad1.left_stick_button) DEBUG = !DEBUG; // Toggle the debug flag
        super.loop();
/*
        //open servo (UP)
        if (gamepad1.a) open_servos(); //find double through trial and error; set in constant variables
        //close servo (DOWN)
        if (gamepad1.b)  close_servos(); //find double through trial and error; set in constant variables
        if (gamepad1.left_stick_button) DEBUG = !DEBUG; // Toggle the debug flag
        super.loop();
*/
    }
}
