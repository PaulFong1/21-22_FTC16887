package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import static com.qualcomm.robotcore.hardware.DistanceSensor.distanceOutOfRange;
// Created  for 16887.
@Autonomous(name="BlueSkyStone", group="AUTO")
//@Disabled
public class BlueSkyStone extends BaseRobot {
    private int stage = 0;
    private double distance_mec1 = 0.0;     // distance moved to find the 1st black skystone
    private boolean found_stone1 = false;   // Found 1st black skystone
    private double detected_dist;           // Distance detected by the sensor in CM
    private boolean touches = false;       // Almost touching the skystone
    @Override
    public void init() {
        super.init();
        DEBUG = true;
    }
    @Override
    public void start() {
        super.start();
        lift1.setPower(-0.05);            // Don't need LIFT for this mode
        lift1.setTargetPosition(0);     // -5 to prevent hitting the skybridge
    }
    @Override
    public void loop() {
        // Assumptions:
        // 1. the servos and arms are open FULLY
        // 2. the rack is down and not needed for this mode
        switch (stage) {
            case 0:          // SLOWLY Forward 45 inches or touches the skystone
                //detected_dist = distance_sensor.getDistance(DistanceUnit.CM);
                //if (!(detected_dist == distanceOutOfRange)) {   // Within sensor range
                //    touches = detected_dist < 25.0;              // Almost touches the skystone
                //}
                if (auto_drive(0.75, 44.0)) {//0 to 2; 2 to 3; 3 to 4
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 1:          // SLOWLY Strafe RIGHT up to 60 inches until a black skystone is found
                found_stone1 = is_black(front_sensor.alpha(), front_sensor.red(),front_sensor.blue());
                if (auto_mecanum(0.25, 75.0) || found_stone1) {
//if (found_stone1) {             // Found the 1st black skystone
//   auto_mecanum(1.0, 60.0);  // Strafe from the edge to the center of the black skystone
// }else {                        // Did not find a black skystone; Just pick up any skystone
// // Detect the yellow skystone at the end?
// }
                    distance_mec1 = get_rightFront_motor_enc() / ConstantVariables.K_PPIN_DRIVE; // Remember the distance moved to find the 1st skystone and convert ticks to inches
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 2:         // CAREFULLY Move forward 2.0 inch to start grabbing
                if (auto_drive(0.2, 2.75)) {//2.0 to 2.5; 2.5 to 2.75
                    reset_drive_encoders();
                    stage++;
                }
            case 3:         // Close Servos: push back the two stones on the two sides and pick up the middle black stone
     //           if (close_servos())
                           stage++;
                break;
            case 4:         // Back slowly to avoid hitting the skybridge during mecanum
                if (auto_drive(-0.75, 10.0)) { // 8 to 10
                    reset_drive_encoders();
                    stage++;
                }
                break;

            case 5:         // Move LEFT 30 inches + distance moved to find the black skystone
                if (auto_mecanum(-1.0, (86.0 - distance_mec1))) {    // Strafe LEFT  //0 to 4; 4 to 6
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 6:         // Open Servos to release the skystone
 //               if (open_servos()) stage++;
   //             open_servos();
                stage++;
                break;
            case 7:         // Move closer to the Skybridge and shake off the skystone
                if (auto_drive(1.0,4.0)) { //5 to four
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 8:
                if (auto_drive(-1,4.0)) {//avoid bridge
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 9:         // Strafe RIGHT 16 inches to PARK
                if (auto_mecanum(1.0, 19.0)) { // Strafe RIGHT to PARK //16 to 18
                    reset_drive_encoders();
                    front_sensor.enableLed(false);          // Turn off LED
                    stage=106901;                           // Mission accomplished
                }
            default: break;
        }
        if (gamepad1.left_stick_button) DEBUG = !DEBUG; // Toggle the debug flag
        if (DEBUG) telemetry.addData("Blue Skystone: ", stage);
        super.loop();
    }
}
