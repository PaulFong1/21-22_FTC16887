package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// Created  for 16887.
@Autonomous(name="Forward 5, then Left", group="Simple")
//@Disabled
public class Forward1_Left extends BaseRobot {
    private int stage = 0;
    @Override
    public void init() {
        super.init();
        DEBUG = true;
    }
    @Override
    public void start() {
        super.start();
        lift1.setPower(0.0);            // Don't need LIFT for this mode
        lift1.setTargetPosition(0);
    }
    @Override
    public void loop() {
        // Assumptions: the rack is down
        switch (stage) {
            case 0:          // SLOWLY Forward 5 inches
                if (auto_drive(0.5, 3.0)) {//5 to 3
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 1:          // SLOWLY Strafe LEFT up to 12 inches to park
                if (auto_mecanum(-0.5, 33.0)) {//24 to 30; 33
                    reset_drive_encoders();
                    stage++;
                }
                break;
            default: break;
        }
        if (DEBUG) telemetry.addData("Forward 5, then LEFT: ", stage);
        super.loop();
    }
}
