package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
// Created  for 16887.
@Autonomous(name="BlueFoundation", group="AUTO")
//@Disabled
public class BlueFoundation extends BaseRobot {
    private int stage = 0;
    @Override
    public void init() {
        super.init();
        DEBUG = true;
        timer.reset();
    }
    @Override
    public void start() {
        super.start();
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);     // LIFT was set to ENCODER by super
        set_lift1_target_pos(ConstantVariables.K_LIFT_UP_FOUND);  // Raise the rack to get ready
      //  open_servos(ConstantVariables.K_LEFT_SERVO_OPEN+0.1,ConstantVariables.K_RIGHT_SERVO_OPEN+0.1);  // Close a little to avoid hit the edge of the foundation
    }
    @Override
    public void loop() {
        // Assumptions:
        // 1. the servos and arms are open
        // 2. the rack is up
        switch (stage) {
            case 0:             // Move forward to avoid hitting the wall
                if (auto_drive(1.0, 5.0)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 1:             // Strafe LEFT to face the center of the foundation
                if (auto_mecanum(-1.0, 37.0)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 2:             // Move SLOWLY forward to hit the foundation
                if (auto_drive(0.5, 48.0)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 3:             // Lower racks to touch the foundation
//                if (set_lift1_target_pos(ConstantVariables.K_LIFT_DOWN_FOUND)) stage++;
                // LIFT motor is not functioning properly.  It could take a long time to move down
                lift1.setTargetPosition(ConstantVariables.K_LIFT_DOWN_FOUND);
                if (lift1.getCurrentPosition() < 25) stage++;
                break;
            case 4:             // Pull the foundation back to the build site
                if (auto_drive(-1.0, 58.0)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 5:             // Lift racks to release the foundation
//                if (set_lift1_target_pos(ConstantVariables.K_LIFT_UP_FOUND)) stage++;
                set_lift1_target_pos(ConstantVariables.K_LIFT_UP_FOUND);
                if (Math.abs(lift1.getCurrentPosition() - ConstantVariables.K_LIFT_UP_FOUND) < 60)
                    stage++;
                break;
            case 6:             // Move backward a little to avoid hitting the foundation during mecanum
                if (auto_drive(-1.0, 0.5)) {    // Move backward away from the foundation
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 7:             // First of two strafes to PARK
                if (auto_mecanum(1, 50)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 8:             // Lower the rack to avoid hitting the foundation
//                if (set_lift1_target_pos(ConstantVariables.K_LIFT_DOWN_FOUND)) {
//                    stage++;
//                }
                set_lift1_target_pos(ConstantVariables.K_LIFT_DOWN_FOUND);
                stage++;
                break;
            case 9:             // Final of two strafes to PARK
                if (auto_mecanum(1.0, 38.0)) {    // Strafe RIGHT
                    reset_drive_encoders();
                    front_sensor.enableLed(false);
                    stage=106901;   // Mission accomplished
                }
                break;
             default:
                 break;
        }
        if (gamepad1.left_stick_button) DEBUG = !DEBUG; // Toggle the debug flag
        if (DEBUG) telemetry.addData("Blue Foundation: ", stage);
        super.loop();
    }
}