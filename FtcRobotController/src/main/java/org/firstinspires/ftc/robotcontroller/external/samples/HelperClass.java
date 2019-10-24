package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class HelperClass {

    private int num_of_ticks = 1440 ;
    private double PI = 22 / 7;
    private double wheel_radius = 10.16;
    private double wheel_circumference = (2 * PI * wheel_radius);
    private double robot_radius = 10.16; // habda
    private double robot_spin_circumference = (2 * PI * robot_radius) ;




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
     * METHOD: cm_to_ticks
     * <p>
     * Used to convert cm to ticks
     * <p>
     * parameters: double distance
     *
     * return int
     */
    private int cm_to_ticks(double distance) {

        int ticks_to_go = (int)((distance * num_of_ticks) / wheel_circumference);
        return ticks_to_go;
    }





    /**
     *METHOD: left_side_position
     *
     * set the position of motors of the left side motors
     *
     * parameters:DcMotor left_back_motor ,DcMotor left_front_motor , double distance
     *
     * return void
     */

    public void left_side_position (DcMotor left_back_wheel , DcMotor left_front_wheel ,double distance )
    {

        left_back_wheel.setTargetPosition(cm_to_ticks(distance));
        left_front_wheel.setTargetPosition(cm_to_ticks(distance));

        left_back_wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_front_wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }





    /**
     *METHOD: right_side_position
     *
     *
     *
     * set the position of motors of the right side motors
     *
     * parameters:DcMotor right_back_motor ,DcMotor right_front_motor , double distance
     *
     * return void
     */

    public void right_side_position (DcMotor right_back_wheel , DcMotor right_front_wheel,double distance)
    {
        right_back_wheel.setTargetPosition(cm_to_ticks(distance));
        right_front_wheel.setTargetPosition(cm_to_ticks(distance));


        right_back_wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_wheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }








    /**
     *METHOD: left_is_busy
     *
     * check if the left side motors have reached their pre set position or not
     *
     * parameters:DcMotor left_back_motor ,DcMotor left_front_motor
     *
     * return boolean
     */
    public boolean left_is_busy (DcMotor left_back_wheel , DcMotor left_front_wheel) //wait
    {
        return(left_back_wheel.isBusy() && left_front_wheel.isBusy());
    }







    /**
     *METHOD: right_is_busy
     *
     * check if the right side motors have reached their pre set position or not
     *
     * parameters:DcMotor right_back_motor ,DcMotor right_front_motor
     *
     * return boolean
     */

    public boolean right_is_busy (DcMotor right_back_wheel , DcMotor right_front_wheel)// wait
    {
        return(right_back_wheel.isBusy() && right_front_wheel.isBusy());
    }






    /**
     *METHOD: move_tank_with_encoder
     *
     * move using the encoder
     *
     * parameters: DcMotor left_back_wheel , DcMotor left_front_wheel,
     *                                        DcMotor right_back_wheel, DcMotor right_front_wheel,
     *                                        double left_side_power, double right_side_power,double distance
     *
     * return void
     */
    public void move_tank_with_encoder(DcMotor left_back_wheel , DcMotor left_front_wheel,
                                       DcMotor right_back_wheel, DcMotor right_front_wheel,
                                       double left_side_power, double right_side_power,double distance){

        left_back_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        left_side_position(left_back_wheel,left_front_wheel,distance);
        right_side_position(right_back_wheel,right_front_wheel,distance);

        set_left_side_power(left_back_wheel,left_front_wheel,left_side_power);
        set_right_side_power(right_back_wheel,right_front_wheel,right_side_power);

        while(left_is_busy( left_back_wheel , left_front_wheel) && right_is_busy(right_back_wheel ,right_front_wheel)) {

        }

        set_left_side_power(left_back_wheel,left_front_wheel,0);
        set_right_side_power(right_back_wheel,right_front_wheel,0);



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




    /**
     *METHOD: Hankash_spin
     *
     ' Spin the robot
     *
     * parameters:DcMotor right_back_motor ,DcMotor right_front_motor, left_back_wheel, left_front_wheel,
     *                               power , Given_Degree , direction
     *
     * return void
     *
     */

    public void Hankash_spin(DcMotor right_back_wheel, DcMotor right_front_wheel,
                             DcMotor left_back_wheel, DcMotor left_front_wheel,
                             double power , int Given_Degree , char direction){
        double num_of_rotations = (robot_spin_circumference/wheel_circumference);
        double degrees_per_rotation = 360 / num_of_rotations ;
        double distance = (Given_Degree*wheel_circumference/degrees_per_rotation);

        if(direction == 'R'){
            move_tank_with_encoder(right_back_wheel,right_front_wheel,left_back_wheel, left_front_wheel,power,-power,distance );
        }
        else if(direction == 'L'){
            move_tank_with_encoder(right_back_wheel,right_front_wheel,left_back_wheel, left_front_wheel,-power,power,distance );
        }
    }



     /*

    // for memory

    public double degress_of_turning (double degress )
    {
        return(degress * distance_between_wheels * degconv );
    }




    public double degress_of_turningat (double degress )
    {
      return ( 2 * 3.14 * distance_between_wheels * degress / 360  )

    }

*/























}






