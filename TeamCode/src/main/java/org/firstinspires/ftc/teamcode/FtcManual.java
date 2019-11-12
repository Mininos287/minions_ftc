package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "org.firstinspires.ftc.teamcode.FtcManual",group = "FTC")

public class FtcManual extends OpMode {
    private DcMotor left_back_motor = null;
    private DcMotor left_front_motor = null;
    private DcMotor right_back_motor = null;
    private DcMotor right_front_motor = null;
    private DcMotor arm_motor = null;
    private TouchSensor touch_sensor ;

    double move_power = 0 ;
    double diagonal_power = 0 ;
    double side_power = 0 ;
    double spin_power = 0 ;
    double arm_power = 0 ;
    double stop_power = 0 ;


    HelperClass helper_class_object = new HelperClass();

    @Override
    public void init() {
        left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
        left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");
        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
        touch_sensor = hardwareMap.get(TouchSensor.class, "touch_sensor");


        left_back_motor.setDirection(DcMotor.Direction.FORWARD);
        left_front_motor.setDirection(DcMotor.Direction.FORWARD);
        right_back_motor.setDirection(DcMotor.Direction.FORWARD);
        right_front_motor.setDirection(DcMotor.Direction.FORWARD);
        arm_motor.setDirection(DcMotor.Direction.FORWARD);



    }

    @Override
    public void init_loop() {

        telemetry.addData("Status", "init_loop");

        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */
    }


    @Override
    public void start() {
        telemetry.addData("Status", "start");


        /*
         * Code to run ONCE when the driver hits PLAY
         */
    }


    @Override
    public void loop() {


        while (! touch_sensor.isPressed()){
            helper_class_object.move_arm_without_encoder(arm_motor, arm_power ,'U');

        }


        if (gamepad1.right_bumper) {

              move_power = 100 ;
              diagonal_power = 100 ;
              side_power = 100 ;
              spin_power = 100 ;
              arm_power = 15;

        } else {

            move_power = 20 ;
            diagonal_power = 20 ;
            side_power = 20 ;
            spin_power = 20 ;
            arm_power = 10;


        }

        if (gamepad1.dpad_up) {
            //move forward
            helper_class_object.move_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,
                    move_power, 'F');
        }

        else if (gamepad1.dpad_down) {
            //move backward
            helper_class_object.move_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,
                    -move_power, 'B');
        }


        else if (gamepad1.dpad_right) {
            //move right
            helper_class_object.move_side_without_encoder(left_back_motor, right_front_motor,
                    left_front_motor, right_back_motor,
                    side_power, 'R');
        }
        else if (gamepad1.dpad_left) {
            //move left
            helper_class_object.move_side_without_encoder(left_back_motor, right_front_motor,
                    left_front_motor, right_back_motor,
                    side_power, 'L');

        }

        else if (gamepad1.b) {
            //spin clockwise
            helper_class_object.spin_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, spin_power, 'C');
        }

        else if (gamepad1.x) {
            //spin anti-clockwise
            helper_class_object.spin_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, spin_power, 'A');

        }else{
            helper_class_object.move_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,
                    stop_power, 'F');
        }

        if(gamepad1.y){
             helper_class_object.move_arm_without_encoder(arm_motor, arm_power ,'U');
        }else if(gamepad1.a){
            helper_class_object.move_arm_without_encoder(arm_motor, -arm_power ,'D');

        }else{
            helper_class_object.move_arm_without_encoder(arm_motor, stop_power ,'U');
        }

        if(gamepad2.dpad_up){
            // 3abla up

        }
        else if(gamepad2.dpad_down){
            //  3abla down
        }

        else if(gamepad2.dpad_right){
            //
        }

    }
}