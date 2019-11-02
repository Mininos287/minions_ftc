package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Boolean.FALSE;

public class HelperClass {

    private int num_of_ticks = 1440 ;
    private double PI = 22 / 7;
    private double wheel_radius = 10.16;
    private double wheel_circumference = (2 * PI * wheel_radius);
    private double robot_radius = 10.16; // habda
    private double robot_spin_circumference = (2 * PI * robot_radius) ;
    private double robot_turn_circumference = (2 * PI * (2*robot_radius)) ;




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
     * METHOD: side_power
     *
     * Provide the two wheels on the same side with the same power
     *
     * parameters: double power , DcMotor first_motor , DcMotor second_motor
     * return void
     */
    private void side_power(DcMotor first_motor, DcMotor second_motor, double power){
        first_motor.setPower(dc_motor_power_adapter(power));
        second_motor.setPower(dc_motor_power_adapter(power));
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
     *METHOD: side_position
     *
     * set the position of motors of the same side motors
     *
     * parameters:DcMotor first_motor , DcMotor second_motor, double distance
     *
     * return void
     */

    public void side_position (DcMotor first_motor , DcMotor second_motor ,double distance )
    {

        first_motor.setTargetPosition(cm_to_ticks(distance));
        second_motor.setTargetPosition(cm_to_ticks(distance));

        first_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        second_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }







    /**
     *METHOD: side_is_busy
     *
     * check if the same side motors have reached their pre set position or not
     *
     * parameters:DcMotor first_motor , DcMotor second_motor
     *
     * return boolean
     */
    public boolean side_is_busy (DcMotor first_motor , DcMotor second_motor) //wait
    {
        return(first_motor.isBusy() && second_motor.isBusy());
    }








    /**
     *METHOD: move_tank_with_encoder
     *
     * move using the encoder
     *
     * parameters: DcMotor left_back_wheel , DcMotor left_front_wheel,
     *                                        DcMotor right_back_wheel, DcMotor right_front_wheel,
     *                                        double side_power,double distance
     *
     * return void
     */
    public void move_tank_with_encoder(DcMotor left_back_wheel , DcMotor left_front_wheel,
                                       DcMotor right_back_wheel, DcMotor right_front_wheel,
                                       double left_side_power, double right_side_power,double distance,boolean break_at_end){

        left_back_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        side_position(left_back_wheel,left_front_wheel,distance);
        side_position(right_back_wheel,right_front_wheel,distance);

        side_power(left_back_wheel,left_front_wheel,left_side_power);
        side_power(right_back_wheel,right_front_wheel,right_side_power);

        while(side_is_busy( left_back_wheel , left_front_wheel) && side_is_busy(right_back_wheel ,right_front_wheel)) {

        }



        if(break_at_end == 1) {
            side_power(left_back_wheel, left_front_wheel, 0);
            side_power(right_back_wheel, right_front_wheel, 0);
        }


    }







    /**
     * METHOD: move_tank_without_encoder
     *
     * Used to give the same power to the right side and the same power to the left side
     * parameters: DcMotor left_back_wheel, DcMotor left_front_wheel,
     *                                           DcMotor right_back_wheel, DcMotor right_front_wheel,
     *                                            side_power
     *
     * return double
     */
    public void move_tank_without_encoder(DcMotor left_back_wheel, DcMotor left_front_wheel,
                                          DcMotor right_back_wheel, DcMotor right_front_wheel,
                                          double left_side_power, double right_side_power){

        side_power(left_back_wheel, left_front_wheel, left_side_power);

        side_power(right_back_wheel, right_front_wheel, right_side_power);
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







    /**
     *METHOD: Hankash_Turn
     *
     *  Turn the robot
     *
     * parameters: right_back_wheel, right_front_wheel,
     *                              left_back_wheel, left_front_wheel,
     *                              power , Given_Degree , direction
     *
     * return void
     */

    public void Hankash_Turn(DcMotor right_back_wheel, DcMotor right_front_wheel,
                             DcMotor left_back_wheel, DcMotor left_front_wheel,
                             double power , int Given_Degree , char direction)
    {
        double num_of_rotations = (robot_turn_circumference/wheel_circumference);
        double degrees_per_rotation = 360 / num_of_rotations ;
        double distance = (Given_Degree*wheel_circumference/degrees_per_rotation);
        if(direction == 'R') {
            move_tank_with_encoder(left_back_wheel, left_front_wheel, right_back_wheel, right_front_wheel, power, 0, distance);
        }else if(direction == 'L'){
            move_tank_with_encoder(left_back_wheel, left_front_wheel, right_back_wheel, right_front_wheel, 0, power, distance);
        }
    }


    /**
     *METHOD: acceleration
     *
     *  accelerate the power of robot
     *
     * parameters: right_back_wheel, right_front_wheel,
     *                              left_back_wheel, left_front_wheel,
     *                              power, distance, number of stages
     *
     * return void
     */

    public void acceleration(DcMotor left_back_wheel,DcMotor left_front_wheel,DcMotor right_back_wheel,DcMotor right_front_wheel, double power,double distance,int number_of_stages){
        double distance_in_stage = distance/number_of_stages;
        double power_in_stage = power/number_of_stages;
        double total_power =0;
        for(int done_stages=0; done_stages<number_of_stages;done_stages++){
            total_power+=power_in_stage;
            move_tank_with_encoder(left_back_wheel, left_front_wheel, right_back_wheel, right_front_wheel,total_power, total_power, distance_in_stage,FALSE);

        }
    }



    /**
     *METHOD: deceleration
     *
     *  decelerate the power of robot
     *
     * parameters: right_back_wheel, right_front_wheel,
     *                              left_back_wheel, left_front_wheel,
     *                              power, distance, number of stages
     *
     * return void
     */


    public void deceleration(DcMotor left_back_wheel,DcMotor left_front_wheel,DcMotor right_back_wheel,DcMotor right_front_wheel,double power,double distance,int number_of_stages) {
        double distance_in_stage = distance / number_of_stages;
        double power_in_stage = power / number_of_stages;
        double total_power = power
        for (int done_stages = 0; done_stages < number_of_stages; done_stages++) {
            total_power -= power_in_stage;
            move_tank_with_encoder(left_back_wheel, left_front_wheel, right_back_wheel, right_front_wheel,total_power, total_power, distance_in_stage,FALSE);

        }

        /**
         *METHOD: acceleration_move_deceleration
         *
         *  used to decrease the moment of inertia
         *
         * parameters: right_back_wheel, right_front_wheel,
         *                              left_back_wheel, left_front_wheel,
         *                              power,acceleration_distance,deceleration_distance,
         *                              total_1`
         *                              -+
         *                          distance, number of stages
         *
         * return void
         */


        public void acceleration_move_deceleration(DcMotor left_back_wheel,DcMotor left_front_wheel,DcMotor right_back_wheel,DcMotor right_front_wheel,double acceleration_distance,double total_distance,double deceleration_distance,double power,int number_of_stages){
         acceleration( left_back_wheel, left_front_wheel, right_back_wheel,  right_front_wheel,power,acceleration_distance,number_of_stages);
         move_tank_with_encoder(left_back_wheel, left_front_wheel, right_back_wheel,  right_front_wheel,power,power,total_distance,FALSE);
         deceleration(left_back_wheel, left_front_wheel, right_back_wheel,  right_front_wheel,power,deceleration_distance,number_of_stages);


        }





}






