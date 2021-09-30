package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// Created for 16887.
@TeleOp(name="Main TeleOp", group="TeleOp")
//@Disabled
public class MainTeleOp extends BaseRobot {
    @Override
    public void init() { super.init(); }
    @Override
    public void start() { super.start(); }
    @Override
    public void loop() {

        //        tankanum_original(gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_x);
        tankanum_drive(gamepad1.right_stick_y, gamepad1.left_stick_y, gamepad1.right_stick_x);

        // Mini movements
        if ((gamepad1.right_stick_y == 0) && (gamepad1.left_stick_y == 0) && (gamepad1.right_stick_x == 0)) {
            if (gamepad1.dpad_up) auto_drive(0.75, 0.5);
            else if (gamepad1.dpad_down) auto_drive(-0.75, 0.5);
            else if (gamepad1.dpad_left) auto_mecanum(-0.75, 0.5);
            else if (gamepad1.dpad_right) auto_mecanum(0.75, 0.5);
//            reset_drive_encoders();
        }
        // lift motor
        if (gamepad1.left_trigger > 0.0)        set_lift1_target_pos(ConstantVariables.K_LIFT_UP);   // UP
        else if (gamepad1.right_trigger > 0.0)  set_lift1_target_pos(ConstantVariables.K_LIFT_DOWN);    // DOWN (initial position)
//        else
//            lift1.setPower(0);

        if (gamepad1.left_bumper)       spin1.setPower(1);
        else if (gamepad1.right_bumper) spin1.setPower(-1);
        else                            spin1.setPower(0);
// spin2 is opposite to spin1
        if (gamepad1.left_bumper)       spin2.setPower(-1);
        else if (gamepad1.right_bumper) spin2.setPower(1);
        else                            spin2.setPower(0);

        if (gamepad1.left_stick_button) DEBUG = !DEBUG; // Toggle the debug flag
        super.loop();

//if (gamepad1.left_bumper)     lift1.setPower(-1.0);
//  else if (gamepad1.right_bumper) lift1.setPower(1.0);
//  else                            lift1.setPower(0.0);
        // open servo
    /*    if (gamepad1.a)
            open_servos(); //find double through trial and error; set in constant variables
        // close servo
        if (gamepad1.b)
            close_servos(); //find double through trial and error; set in constant variables
        if (gamepad1.left_stick_button) DEBUG = !DEBUG; // Toggle the debug flag
   */
    }
}