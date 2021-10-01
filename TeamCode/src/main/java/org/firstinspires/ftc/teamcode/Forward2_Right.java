package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// Created  for 16887.
@Autonomous(name="Forward 50, then Right", group="Simple")
//@Disabled
public class Forward2_Right extends BaseRobot {
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
                if (auto_drive(0.5, 45.0)) {//50 to 45
                    reset_drive_encoders();
                    stage++;
                }
                break;
            case 1:          // SLOWLY Strafe RIGHT up to 12 inches to park
                if (auto_mecanum(0.5, 33.0)) {
                    reset_drive_encoders();
                    stage++;
                }
                break;
            default: break;
        }
        if (DEBUG) telemetry.addData("Forward 50, then Right", stage);
        super.loop();
    }
}