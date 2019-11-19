package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

@TeleOp(name = "Test_Encoder",group = "FTC")
public class Test_Encoder extends OpMode {

    private DcMotor left_back_motor = null;
    private DcMotor left_front_motor = null;
    private DcMotor right_back_motor = null;
    private DcMotor right_front_motor = null;

    double move_power = 0 ;
    double diagonal_power = 0 ;
    double side_power = 0 ;
    double spin_power = 0 ;
    double arm_power = 0 ;
    double stop_power = 0 ;
    // private DcMotor arm_motor = null;
    // private TouchSensor touch_sensor ;

    HelperClass helper_class_object = new HelperClass();

    @Override
    public void init() {
        left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
        left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");
        left_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.right_bumper && gamepad1.left_bumper){
            move_power = 100 ;
            diagonal_power = 100 ;
            side_power = 100 ;
            spin_power = 100 ;
            arm_power = 50;
        }
        else if (gamepad1.right_bumper) {

            move_power = 50 ;
            diagonal_power = 50 ;
            side_power = 50 ;
            spin_power = 50 ;
            arm_power = 25;

        } else {

            move_power = 25 ;
            diagonal_power = 25 ;
            side_power = 25 ;
            spin_power = 25 ;
            arm_power = 15;


        }


        if (gamepad1.dpad_up && gamepad1.dpad_right){
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            helper_class_object.move_diagonal_with_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,20,100,0,diagonal_power,5,'R',TRUE);

        }else if(gamepad1.dpad_up && gamepad1.dpad_left){
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            helper_class_object.move_diagonal_with_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,20,100,0,diagonal_power,5,'L',TRUE);
        }else if(gamepad1.dpad_down &&gamepad1.dpad_right){
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            helper_class_object.move_diagonal_with_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,20,100,0,-diagonal_power,5,'L',TRUE);
        }else if(gamepad1.dpad_down && gamepad1.dpad_left){
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            helper_class_object.move_diagonal_with_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,20,100,0,-diagonal_power,5,'R',TRUE);
        }

        else if (gamepad1.dpad_up) {
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            //move forward
            helper_class_object.move_with_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,20,100,0,move_power,5);
        }

        else if (gamepad1.dpad_down) {
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            //move backward
            helper_class_object.move_with_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,20,100,0,-move_power,5);
        }


        else if (gamepad1.dpad_right) {
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.FORWARD);
            right_front_motor.setDirection(DcMotor.Direction.FORWARD);
            //move right
            helper_class_object.side__with_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,20,100,0,side_power,5,'R',FALSE);
        }
        else if (gamepad1.dpad_left) {
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.FORWARD);
            right_front_motor.setDirection(DcMotor.Direction.FORWARD);
            //move left
            helper_class_object.side__with_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,20,100,0,side_power,5,'L',FALSE);


        }

        else if (gamepad1.b) {
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            //spin clockwise
            helper_class_object.spin_with_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,spin_power,90,360,0,5,'C');
        }

        else if (gamepad1.x) {
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            //spin anti-clockwise
            helper_class_object.spin_with_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,spin_power,90,360,0,5,'A');

        }else{
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            helper_class_object.move_with_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,
                    0, 0,0,stop_power,0);
        }




    }
}
