package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class HelperClass {






    /**
     * METHOD: servo_motor_degrees_adapter
     *
     * Used to convert 0 to 180 (servo degree) to 0 and 1
     *
     * parameters: double degree
     * return double
     */
    private double servo_motor_degrees_adapter(double degree) {
        double new_degrees = degree / 180;
        return new_degrees;
    }





    /**
     * METHOD: lower_to_higher_servo_degrees
     *
     * Move the servo from the smallest number given to the biggest number given by the user
     *
     * parameters: double from , double for
     *
     * return double new_position
     */
    public void lower_to_higher_servo_degrees(Servo servo_motor, double from, double to) {

        for (double position = from; position <= to; position += 1){
            double new_position = servo_motor_degrees_adapter(position);
            servo_motor.setPosition(new_position);
        }

    }






    /**
     *METHOD: higher_to_lower_servo_degrees
     *
     * Move the servo from the biggest number given to the smallest number given by the user
     *
     * parameters: double from , double for
     *
     * return double new_position
     */
    public void higher_to_lower_servo_degrees(Servo servo_motor, double from, double to) {

        for (double position = from; position >= to; position -= 1) {
            double new_position = servo_motor_degrees_adapter(position);
            servo_motor.setPosition(new_position);


        }

    }






    /**
     * METHOD: dc_motor_power_adapter
     *
     * Used to convert from -100 to 100 (power) to -1 and 1
     *
     * parameter: double power
     * return double
     */
    private double dc_motor_power_adapter(double power) {
        double new_power = (power / 100);
        return new_power;
    }








    /**
     * METHOD: set_left_side_power
     *
     * Provide the two wheels on the left side with the same power
     *
     * parameters: double power
     * return double
     */
    private void set_left_side_power(DcMotor left_back_wheel, DcMotor left_front_wheel, double power){
        left_back_wheel.setPower(dc_motor_power_adapter(power));
        left_front_wheel.setPower(dc_motor_power_adapter(power));
    }






    /**
     * METHOD: set_right_side_power
     *
     * Provide the two wheels on the right side with the same power
     *
     * parameters: right_back_wheel,right_front_wheel, double power
     *
     * return double
     */
    private void set_right_side_power(DcMotor right_back_wheel, DcMotor right_front_wheel, double power){
        right_back_wheel.setPower(dc_motor_power_adapter(power));
        right_front_wheel.setPower(dc_motor_power_adapter(power));
    }






    /**
     * METHOD: move_tank_without_encoder
     *
     * Used to give the same power to the right side and the same power to the left side
     * parameters: power
     *
     * return double
     */
    public void move_tank_without_encoder(DcMotor left_back_wheel, DcMotor left_front_wheel,
                                          DcMotor right_back_wheel, DcMotor right_front_wheel,
                                          double left_side_power, double right_side_power){

        set_left_side_power(left_back_wheel, left_front_wheel, left_side_power);

        set_left_side_power(right_back_wheel, right_front_wheel, right_side_power);
    }





















}






