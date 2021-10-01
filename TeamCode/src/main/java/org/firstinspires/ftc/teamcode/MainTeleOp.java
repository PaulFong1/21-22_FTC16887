package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// Testing 1234556
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
        // Turn the top spinning wheel: left for clockwise and right for anticlockwise

        if (gamepad1.left_bumper)       topSpin.setPower(1);
        else if (gamepad1.right_bumper) topSpin.setPower(-1);
        else                            topSpin.setPower(0);
        //turn the motor for the linear slide
        if (gamepad1.left_trigger>0)       linearSlide.setPower(1);
        else if (gamepad1.right_trigger>0) linearSlide.setPower(-1);
        else                            linearSlide.setPower(0);


        if (gamepad1.left_stick_button) DEBUG = !DEBUG; // Toggle the debug flag
        super.loop();

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
