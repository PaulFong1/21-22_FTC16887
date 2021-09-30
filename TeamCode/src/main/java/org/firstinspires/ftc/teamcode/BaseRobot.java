package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static com.qualcomm.robotcore.hardware.DistanceSensor.distanceOutOfRange;

//created by Paul Fong for 16887
public class BaseRobot extends OpMode {
    public DcMotor leftBack, rightBack, leftFront, rightFront, lift1, spin1, spin2; //lift2; 1 is right, 2 is left
 //   public Servo left_servo, right_servo;
    public ColorSensor front_sensor;
    public DistanceSensor distance_sensor;
    public ElapsedTime timer = new ElapsedTime();
    public boolean DEBUG=false;                     // Debug flag
//    public Boolean autonmous = false;

    @Override
    public void init() {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");
        lift1      = hardwareMap.get(DcMotor.class, "lift1");
        spin1      = hardwareMap.get(DcMotor.class, "spin1");
        spin2      = hardwareMap.get(DcMotor.class, "spin2");
        front_sensor = hardwareMap.get(ColorSensor.class, "front_sensor");
        distance_sensor = hardwareMap.get(DistanceSensor.class, "front_sensor");
        front_sensor.enableLed(false);               // turn off the sensor LED to save power

        // ZeroPowerBehavior of the motors
        telemetry.addData("INI Front ZeroP behavior:", "Left=%s, Right=%s", leftFront.getZeroPowerBehavior(), rightFront.getZeroPowerBehavior());
        telemetry.addData("INI Back ZeroP behavior: ", "Left=%s, Right=%s", leftBack.getZeroPowerBehavior(), rightBack.getZeroPowerBehavior());
        telemetry.addData("INI SPIN ZeroP behavior: ", "spin1=%s, spin2=%s", spin1.getZeroPowerBehavior(), spin2.getZeroPowerBehavior());
        telemetry.addData("INI LIFT1 position", lift1.getCurrentPosition());
        telemetry.addData("INI Sen: ", "%d/ %d/ %d/ %d/ %d", front_sensor.alpha(), front_sensor.red(), front_sensor.green(), front_sensor.blue(), front_sensor.argb());
        telemetry.addData("INI Distance (cm)", distance_sensor.getDistance(DistanceUnit.CM));
       // telemetry.addData("INI Servo dir: ", "LEFT=%s, RIGHT=%s", left_servo.getDirection(), right_servo.getDirection());
        //telemetry.addData("INI Servo pos: ", "LEFT=%.2f, RIGHT=%.2f", left_servo.getPosition(), right_servo.getPosition());

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        lift1.setDirection(DcMotorSimple.Direction.REVERSE);        // Because of the way the motor is mounted, this will avoid using negative numbers
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //spin1.setDirection(DcMotorSimple.Direction.REVERSE);        // Because of the way the motor is mounted, this will avoid using negative numbers
        spin1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //spin2.setDirection(DcMotorSimple.Direction.REVERSE);        // Because of the way the motor is mounted, this will avoid using negative numbers
        spin2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Positive is UP and negative in down.  It resets to zero at start().
    }
    @Override
    public void start() {
        timer.reset();
        reset_drive_encoders();         // reset all four motors: set encoders to zero and set modes
        reset_lift1_encoder();          // reset lift motor: set encoders to zero and set modes
        reset_spin1_encoder();
        reset_spin2_encoder();
    //    open_lift1();                  // open the left and right servos
set_lift1_target_pos(ConstantVariables.K_LIFT_UP);
        front_sensor.enableLed(true);   // turn on the sensor LED
        telemetry.addData("START Front ZeroP behavior:", "Left=%s, Right=%s", leftFront.getZeroPowerBehavior(), rightFront.getZeroPowerBehavior());
        telemetry.addData("START Back ZeroP behavior: ", "Left=%s, Right=%s", leftBack.getZeroPowerBehavior(), rightBack.getZeroPowerBehavior());
        telemetry.addData("START LIFT1 position", lift1.getCurrentPosition());
        telemetry.addData("START Sen: ", "%d/ %d/ %d/ %d/ %d", front_sensor.alpha(), front_sensor.red(), front_sensor.green(), front_sensor.blue(), front_sensor.argb());
        telemetry.addData("START Distance (cm)", distance_sensor.getDistance(DistanceUnit.CM));
      //  telemetry.addData("START Servo dir: ", "LEFT=%s, RIGHT=%s", left_servo.getDirection(), right_servo.getDirection());
        //telemetry.addData("START Servo pos: ", "LEFT=%.2f, RIGHT=%.2f", left_servo.getPosition(), right_servo.getPosition());
    }
    @Override
    public void stop() {
        front_sensor.enableLed(false);   // turn off the sensor LED to save power
    }
    @Override
    public void loop() {
        if (DEBUG) {
            String detected_color = "";
            if (is_black(front_sensor.alpha(), front_sensor.red(), front_sensor.blue())) detected_color = detected_color + " Black ";
            if (is_yellow(front_sensor.alpha(), front_sensor.red(), front_sensor.green(), front_sensor.blue())) detected_color = detected_color + " Yellow ";

//            telemetry.addData("Timer ", "%.1f", timer.seconds());
            telemetry.addData("LIFT1 motor current pos: ", "%d, Color = %s", get_lift1_motor_enc(), detected_color);
            telemetry.addData("Front curr pos:", "Left=%d, Right=%d", get_leftFront_motor_enc(), get_rightFront_motor_enc());
            telemetry.addData("Back  curr pos:", "Left=%d, Right=%d", get_leftBack_motor_enc(), get_rightBack_motor_enc());
            telemetry.addData("Front power: ", "Left=%.2f, Right=%.2f", leftFront.getPower(), rightFront.getPower());
            telemetry.addData("Back  power: ", "Left=%.2f, Right=%.2f", leftBack.getPower(), rightBack.getPower());
          //  telemetry.addData("Servo pos: ", "Left=%.2f, Right=%.2f", left_servo.getPosition(), right_servo.getPosition());
            telemetry.addData("Sen: ", " %d/ %d/ %d/ %d/ %d", front_sensor.alpha(), front_sensor.red(), front_sensor.green(), front_sensor.blue(), front_sensor.argb());
//            telemetry.addData("Red：", front_sensor.red());
//            telemetry.addData("Green：", front_sensor.green());
//            telemetry.addData("Blue：", front_sensor.blue());
            double detected_dist = distance_sensor.getDistance(DistanceUnit.CM);
            if (detected_dist == distanceOutOfRange) {
                telemetry.addData("Distance: out of range", distance_sensor.getDistance(DistanceUnit.CM));
            } else {
                telemetry.addData("Distance: ", "%.2fcm", distance_sensor.getDistance(DistanceUnit.CM));
            }
        }
    }
    /* @param power:   the speed to turn at. Negative for reverse
     * @param dist_inches:  move "inches" inches.  "inches" positive for forward
     * @return Whether the target "inches" has been reached.
     */
    public boolean auto_drive(double power, double dist_inch) {
        int TARGET_ENC = (int) (ConstantVariables.K_PPIN_DRIVE * dist_inch);
        double left_speed = -power;                 // Left motors are running in reverse direction
        double right_speed = power;
//        double error = -get_leftFront_motor_enc() - get_rightFront_motor_enc();
//        error /= ConstantVariables.K_DRIVE_ERROR_P;
//        left_speed += error; right_speed -= error;

        left_speed = Range.clip(left_speed, -1, 1);
        right_speed = Range.clip(right_speed, -1, 1);
        leftFront.setPower(left_speed);
        leftBack.setPower(left_speed);
        rightFront.setPower(right_speed);
        rightBack.setPower(right_speed);

        if (DEBUG) telemetry.addData("Auto_D - Target_enc: ", "%d/%d", get_rightFront_motor_enc(), TARGET_ENC);
        if (Math.abs(get_rightFront_motor_enc()) >= TARGET_ENC) { // Arrived at target destination
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            return true;
        }
        return false;
    }
    /* @param power:   the speed to turn at. Negative for left.
     * @param degrees: the number of degrees to turn.
     * @return Whether the target angle has been reached.
     */
    public boolean auto_turn(double power, double degrees) {
        double TARGET_ENC = Math.abs(ConstantVariables.K_PPDEG_DRIVE * degrees);  // degrees to turns
        double speed = Range.clip(power, -1, 1);
        leftFront.setPower(-speed);
        leftBack.setPower(-speed);
        rightFront.setPower(-speed);
        rightBack.setPower(-speed);

        if (DEBUG) telemetry.addData("AUTO_T - TURNING TO ENC: ", TARGET_ENC);
        if (Math.abs(get_rightFront_motor_enc()) >= TARGET_ENC) {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            return true;
        } else {
            return false;
        }
    }
    // Positive for right, negative for left
    // Convert from inches to number of ticks per revolution
    public boolean auto_mecanum(double power, double inches) {
        double TARGET_ENC = ConstantVariables.K_PPIN_DRIVE * inches;
        double leftFrontPower = Range.clip(0 - power, -1.0, 1.0);
        double leftBackPower = Range.clip(0 + power, -1.0, 1.0);
        double rightFrontPower = Range.clip(0 - power, -1.0, 1.0);
        double rightBackPower = Range.clip(0 + power, -1.0, 1.0);

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
        if (DEBUG) telemetry.addData("MEC - Target_enc: ", "%.2f", TARGET_ENC);

        if (Math.abs(get_rightFront_motor_enc()) >= TARGET_ENC) {
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);
            return true;
        } else {
            return false;
        }
    }
    public void tankanum_original(double rightPwr, double leftPwr, double lateralpwr) {
        rightPwr *= -1;

        double leftFrontPower = Range.clip(leftPwr - lateralpwr, -1.0, 1.0);
        double leftBackPower = Range.clip(leftPwr + lateralpwr, -1.0, 1.0);
        double rightFrontPower = Range.clip(rightPwr - lateralpwr, -1.0, 1.0);
        double rightBackPower = Range.clip(rightPwr + lateralpwr, -1.0, 1.0);

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }
    // lateralpwr: pos for right, neg for left
    // When strafing, rightPwr and leftPwr is assumed to be zero
    // With the change, there is no diagonal movement
    public void tankanum_drive(double rightPwr, double leftPwr, double lateralpwr) {
        rightPwr *= -1;                                 // rightPwr is in reverse
        double leftFrontPower, leftBackPower, rightFrontPower, rightBackPower;
        // When lateralpwr is very small, the robot moves purely forward or backward
        if (lateralpwr > -0.05 && lateralpwr < 0.05) {  // Purely forward or backward
            leftFrontPower  = leftPwr;
            leftBackPower   = leftPwr;
            rightFrontPower = rightPwr;                 // left = right's opposite
            rightBackPower  = rightPwr;
        } else {                                        // Strafe
            leftFrontPower  = -lateralpwr;              // leftFront = rightBack's opposite
            leftBackPower   = lateralpwr;               // leftBack  = rightFront's opposite
            rightFrontPower = -lateralpwr;
            rightBackPower  = lateralpwr;
        }
// to adjust the power among the motors so that they have almost equal ACTUAL PHYSICAL powers
leftFrontPower = Range.clip(leftFrontPower * ConstantVariables.K_LF_ADJUST, -1.0, 1.0);
leftBackPower = Range.clip(leftBackPower * ConstantVariables.K_LB_ADJUST, -1.0, 1.0);
rightFrontPower = Range.clip(rightFrontPower * ConstantVariables.K_RF_ADJUST, -1.0, 1.0);
rightBackPower = Range.clip(rightBackPower * ConstantVariables.K_RB_ADJUST, -1.0, 1.0);

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
        if (DEBUG) telemetry.addData("TANM- Lateral: ", lateralpwr);
    }
    public void tank_drive(double leftPwr, double rightPwr) {
        rightPwr *= -1;                     // rightPwr is in reverse
        double leftPower = Range.clip(leftPwr, -1.0, 1.0);
        double rightPower = Range.clip(rightPwr, -1.0, 1.0);

        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);    // right is opposite to left
        rightBack.setPower(rightPower);     // right is opposite to left
        if (DEBUG) telemetry.addData("TAND- Power: ", "Left=%.2f, Right=%.2f", leftPower, rightPower);
    }
    // alpha is light intensity
//    public boolean is_black(int alpha, int red, int blue) { return ((alpha > 350) && (blue > red*(3.0/4.0))); }
//    public boolean is_yellow(int alpha, int red, int green, int blue) { return ((alpha > 350) && (red > 2*blue) && (green > 2*blue)); }
    public boolean is_black(int alpha, int red, int blue) { return ((blue > red*(3.0/4.0))); }
    public boolean is_yellow(int alpha, int red, int green, int blue) { return ((red > 2*blue) && (green > 2*blue)); }

    public boolean set_lift1_target_pos(int target_pos) {
        if (lift1.getCurrentPosition() == target_pos) return true;
        lift1.setTargetPosition(target_pos);
//        lift1.setPower(ConstantVariables.K_LIFT_MAX_PWR);
//        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);  // Assume that the motor was reset.  Before setting the mode, the position has to be set correctly.
        return(Math.abs(lift1.getCurrentPosition() - target_pos) < ConstantVariables.K_LIFT_ERROR);
    }
    // get lift encoder
    public int get_lift1_motor_enc() {      // RUN_TO_POSITION is more accurate
//        if (lift1.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
//            lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
        return lift1.getCurrentPosition();
    }
    public void reset_drive_encoders() {
        // The motor is to set the current encoder position to zero.
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // The motor is to do its best to run at targeted velocity.
//        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// The motor is simply to run at whatever velocity is achieved by apply a particular power level to the motor.
leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void reset_lift1_encoder() {
        // The motor is to set the current encoder position to zero.
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // The motor is to attempt to rotate in whatever direction is necessary to cause
        // the encoder reading to advance or retreat from its current setting to the setting
        // which has been provided through the setTargetPosition() method.
        lift1.setTargetPosition(0);                        // Set to zero before RUN_TO_POSITION
        lift1.setPower(0.5*ConstantVariables.K_LIFT_MAX_PWR);  // Half power is sufficient
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void reset_spin1_encoder() {
        // The motor is to set the current encoder position to zero.
        spin1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin1.setPower(ConstantVariables.K_LIFT_MAX_PWR);
        spin1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void reset_spin2_encoder() {
        // The motor is to set the current encoder position to zero.
        spin2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin2.setPower(ConstantVariables.K_LIFT_MAX_PWR);
        spin2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //get leftBack encoder
    public int get_leftBack_motor_enc() {
//        if (leftBack.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
//            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
        return leftBack.getCurrentPosition();
    }
    //get leftFront encoder
    public int get_leftFront_motor_enc() {
//        if (leftFront.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
//            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
        return leftFront.getCurrentPosition();
    }
    //get rightBack encoder
    public int get_rightBack_motor_enc() {
//        if (rightBack.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
//            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
        return rightBack.getCurrentPosition();
    }
    //get rightFront encoder
    public int get_rightFront_motor_enc() {
//       if (rightFront.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
//            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
        return rightFront.getCurrentPosition();
    }
}
