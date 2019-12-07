package org.firstinspires.ftc.teamcode;

 import com.qualcomm.hardware.bosch.BNO055IMU;
 import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
 import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.hardware.TouchSensor;
 import com.qualcomm.robotcore.util.Range;

 import org.firstinspires.ftc.robotcore.external.Func;
 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
 import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
 import org.firstinspires.ftc.robotcore.external.navigation.Position;
 import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
 import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

 import java.util.Locale;

 import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;
 import static java.lang.Math.pow;
 import static java.lang.Math.round;

public class HelperClass   {

    private int num_of_ticks = 1440 ;
    private double PI = 22 / 7;
    private double wheel_radius = 10.16/2;
    private double wheel_circumference = (2 * PI * wheel_radius);
    private double robot_radius = 10.16; // habda
    private double robot_spin_circumference = (2 * PI * robot_radius) ;
    private double robot_turn_circumference = (2 * PI * (2*robot_radius)) ;
    double  gyro_angel ;


    public int my_motor_is_busy(DcMotor my_motor,double distance,double power ){

        if(power > 0){

            if (my_motor.getCurrentPosition() < cm_to_ticks(distance)){
                return 1 ;
            }else{
                return 0 ;
            }
        }else if (power < 0 ){

            int current = my_motor.getCurrentPosition() ;
            if(current > 0 ){
                current = current * (-1) ;
            }
            if( current < -cm_to_ticks(distance)  ){
                return 0 ;
            }else{
                return  1 ;
            }

        }else{

        return 0 ;

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
    public double dc_motor_power_adapter(double power) {
       double new_power = (power / 100) ;
        // double new_power = power ;

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

        first_motor.setPower(dc_motor_power_adapter(power)  );
        second_motor.setPower(dc_motor_power_adapter(power) );

    }




    /**
     * METHOD: move_holonomic_without_encoder
     *
     * Used to give the same power to the right side and the same power to the left side
     * parameters: DcMotor left_back_wheel, DcMotor left_front_wheel,
     *                                           DcMotor right_back_wheel, DcMotor right_front_wheel,
     *                                            side_power
     *
     * return void
     */
    public void move_holonomic_without_encoder(DcMotor first_motor, DcMotor second_motor,
                                               DcMotor third_motor, DcMotor fourth_motor,
                                               double first_side_power, double second_side_power){

        side_power(first_motor, second_motor, first_side_power);

        side_power(third_motor, fourth_motor, second_side_power);
    }



    /**
     *METHOD: move_without_encoder
     *
     *  used to move the robot backward/forward
     *
     * parameters: left_back_wheel, right_front_wheel,
     *                     left_front_wheel,right_back_wheel,
     *                     first_side_power,second_side_power
     *
     * return void
     */

    public void move_without_encoder(DcMotor left_back_wheel, DcMotor left_front_wheel,
                                     DcMotor right_back_wheel, DcMotor right_front_wheel,
                                     double power ,char direction )
    {
        if(direction == 'F')
        {
            move_holonomic_without_encoder(left_back_wheel, right_front_wheel,
                    left_front_wheel, right_back_wheel,
                    power,power);
        }else if(direction == 'B')
        {
            move_holonomic_without_encoder(left_back_wheel, right_front_wheel,
                    left_front_wheel, right_back_wheel,
                    power,power);
        }



    }

    /**
     *METHOD: move_side_without_encoder
     *
     *  used to move the robot right/left
     *
     * parameters: ( left_back_wheel,  left_front_wheel,
     *                               right_back_wheel,  right_front_wheel,
     *                               power,  direction
     *
     * return void
     */

    public void move_side_without_encoder(DcMotor left_back_wheel, DcMotor left_front_wheel,
                                          DcMotor right_back_wheel, DcMotor right_front_wheel,
                                          double power, char direction )

    {
        if(direction == 'R') {
            move_holonomic_without_encoder(left_back_wheel, right_front_wheel,
                    left_front_wheel, right_back_wheel,
                    -power, power );
        }else if(direction == 'L') {
            move_holonomic_without_encoder(left_back_wheel, right_front_wheel,
                    left_front_wheel, right_back_wheel,
                    power, -power );
        }

    }

    /**
     *METHOD: move_diagonal //without encoder
     *
     *  used to move the robot diagonally right backward/forward
     *
     * parameters: left_back_wheel, right_front_wheel,
     *                 left_front_wheel,right_back_wheel,
     *                 power,direction
     *
     * return void
     */

    public void move_diagonal_without_encoder(DcMotor left_back_wheel, DcMotor left_front_wheel,
                                              DcMotor right_back_wheel, DcMotor right_front_wheel,
                                              double power, char  direction)
    {
        if(direction == 'R'){
            move_holonomic_without_encoder(left_back_wheel, right_front_wheel, left_front_wheel, right_back_wheel,
                    0, power);
        }else if(direction == 'L'){
            move_holonomic_without_encoder(left_back_wheel, right_front_wheel,left_front_wheel,right_back_wheel,
                    power,0);
        }
    }




    /**
     *METHOD:  spin_without_encoder
     *
     *  used to spin the robot without encoder clockwise or anti-clockwise
     *
     * parameters: left_back_wheel, left_front_wheel,
     *                right_back_wheel,right_front_wheel,
     *                 power,direction
     *
     * return void
     */

    public void spin_without_encoder(DcMotor left_back_wheel, DcMotor left_front_wheel,
                                     DcMotor right_back_wheel, DcMotor right_front_wheel,
                                     double power ,char direction )
    {
        if(direction == 'C') {
            move_holonomic_without_encoder(left_back_wheel, left_front_wheel,
                    right_back_wheel, right_front_wheel,
                    power,-power);
        }else if(direction == 'A') {
            move_holonomic_without_encoder(left_back_wheel, left_front_wheel,
                    right_back_wheel, right_front_wheel,
                    -power, power);
        }
    }




    ////////////////////////////////////////////////////////////////////////////////////






    /**
     * METHOD: cm_to_ticks
     * <p>
     * Used to convert cm to ticks
     * <p>
     * parameters: double distance
     *
     * return int
     */
    public int cm_to_ticks(double distance) {

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

        first_motor.setTargetPosition(cm_to_ticks(distance)+first_motor.getCurrentPosition());
        second_motor.setTargetPosition(cm_to_ticks(distance)+second_motor.getCurrentPosition());

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
     *METHOD: move_holonomic_with_encoder
     *
     * move using the encoder
     *
     * parameters: DcMotor left_back_wheel , DcMotor left_front_wheel,
     *                                        DcMotor right_back_wheel, DcMotor right_front_wheel,
     *                                        double side_power,double distance
     *
     * return void
     */
    public void move_holonomic_with_encoder(DcMotor first_motor , DcMotor second_motor,
                                            DcMotor third_motor, DcMotor fourth_motor,
                                            double first_power, double second_power,double distance,boolean break_at_end){



//        first_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        second_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        third_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fourth_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            side_position(first_motor, second_motor, distance);
            side_position(third_motor,fourth_motor,distance);



        side_power(first_motor,second_motor,first_power);
        side_power(third_motor,fourth_motor,second_power);


            while (side_is_busy(first_motor, second_motor) && side_is_busy(third_motor, fourth_motor)) {
            }



            side_power(first_motor, second_motor, 0);
            side_power(third_motor, fourth_motor, 0);



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

    public void acceleration(DcMotor first_motor,DcMotor second_motor,DcMotor third_motor,DcMotor fourth_motor,
                             double first_power,double second_power,double distance,int number_of_stages)
    {

        double first_side_total_power =0;
        double second_side_total_power =0;

        double distance_in_stage = distance/(double) number_of_stages;
        double first_power_in_stage = first_power /(double)number_of_stages;
        double second_power_in_stage = second_power/(double)number_of_stages;


        for(int done_stages=0; done_stages<number_of_stages;++done_stages){

            first_side_total_power = first_side_total_power + first_power_in_stage;
            second_side_total_power = second_side_total_power  +second_power_in_stage;



                move_holonomic_with_encoder(first_motor, second_motor, third_motor, fourth_motor,first_side_total_power,
                        second_side_total_power, distance_in_stage,FALSE);

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

    public void deceleration(DcMotor first_motor,DcMotor second_motor,DcMotor third_motor,DcMotor fourth_motor,
                             double first_power,double second_power,double distance,int number_of_stages){
        double distance_in_stage = distance/(double)number_of_stages;
        double first_power_in_stage = first_power/(double)number_of_stages;
        double second_power_in_stage = second_power/(double)number_of_stages;

        double first_side_total_power =first_power;
        double second_side_total_power =second_power;

        for(int done_stages=0; done_stages<number_of_stages;done_stages++)
        {


            if(done_stages == (number_of_stages-1))
            {
                move_holonomic_with_encoder(first_motor, second_motor, third_motor, fourth_motor,first_side_total_power,
                        second_side_total_power, distance_in_stage,TRUE);
            }else
            { move_holonomic_with_encoder(first_motor, second_motor, third_motor, fourth_motor,first_side_total_power,
                    second_side_total_power, distance_in_stage,FALSE);
            }

            first_side_total_power-=first_power_in_stage;
            second_side_total_power-=second_power_in_stage;

        }
    }










    /**
     *METHOD:  move_with_encoder
     *
     *  used to move the robot forword and backword
     *
     * parameters: left_back_wheel, left_front_wheel,
     *                right_back_wheel,right_front_wheel,
     *                 power,direction distance,break_at_end
     *
     * return void
     */



    public void move_with_encoder(DcMotor left_back_wheel, DcMotor left_front_wheel,
                                  DcMotor right_back_wheel, DcMotor right_front_wheel,
                                  double acceleration_distance, double move_distance, double deceleration_distance ,
                                  double power,int number_of_stages)
    {



        /*acceleration(left_back_wheel, right_front_wheel,
                left_front_wheel, right_back_wheel,power,power,acceleration_distance,number_of_stages);
        */

        move_holonomic_with_encoder(left_back_wheel, right_front_wheel,
                left_front_wheel,right_back_wheel,
                power,power , move_distance , FALSE);


        /*
        deceleration(left_back_wheel, right_front_wheel,
                left_front_wheel, right_back_wheel,power,power,deceleration_distance,number_of_stages);

        */
    }







    /**
     *METHOD:  move_diagonal_with_encoder
     *
     *  used to move the robot right diagonal and left diagonal
     *
     * parameters: left_back_wheel, left_front_wheel,
     *                right_back_wheel,right_front_wheel,
     *                 power,direction distance,break_at_end
     *
     * return void
     */


    public void move_diagonal_with_encoder(DcMotor left_back_wheel,DcMotor left_front_wheel,DcMotor right_back_wheel,DcMotor right_front_wheel,
                                           double acceleration_distance,double move_distance,double deceleration_distance,
                                           double power,int number_of_stages, char direction,boolean break_at_end)
    {

        stop_and_reset(left_back_wheel, right_front_wheel,
                left_front_wheel, right_back_wheel);
        if(direction == 'R')
        {
           // acceleration(left_back_wheel, right_front_wheel,
                  //  left_front_wheel, right_back_wheel,power,0,acceleration_distance,number_of_stages);

            move_holonomic_with_encoder(left_back_wheel, right_front_wheel,
                    left_front_wheel, right_back_wheel,
                    0, power, move_distance,break_at_end);
           // deceleration(left_back_wheel, right_front_wheel,
                  //  left_front_wheel, right_back_wheel,power,0,deceleration_distance,number_of_stages);
        }else  if(direction == 'L')
        {
          //  acceleration(left_back_wheel, right_front_wheel,
              //      left_front_wheel, right_back_wheel,0,power,acceleration_distance,number_of_stages);

            move_holonomic_with_encoder(left_back_wheel, right_front_wheel,
                    left_front_wheel, right_back_wheel,
                    0,power, move_distance,break_at_end);
           // deceleration(left_back_wheel, right_front_wheel,
               //     left_front_wheel, right_back_wheel,power,0,deceleration_distance,number_of_stages);
        }




    }




    public void side__with_encoder(DcMotor left_back_wheel,DcMotor left_front_wheel,DcMotor right_back_wheel,
                                   DcMotor right_front_wheel,
                                   double acceleration_distance,double move_distance,double deceleration_distance,
                                   double power,int number_of_stages, char direction,boolean break_at_end)
    {
        stop_and_reset(left_back_wheel, right_front_wheel,
                left_front_wheel,right_back_wheel);

        if(direction == 'L'){
          /*  acceleration(left_back_wheel, right_front_wheel,
                    left_front_wheel, right_back_wheel,power,-power,acceleration_distance,number_of_stages);
*/

            move_holonomic_with_encoder(left_back_wheel, right_front_wheel,
                    left_front_wheel,right_back_wheel,
                    power,-power ,move_distance , break_at_end);
/*
            deceleration(left_back_wheel, right_front_wheel,
                    left_front_wheel, right_back_wheel,power,-power,deceleration_distance,number_of_stages);
       */
        }else if(direction == 'R'){
           /*
            acceleration(left_back_wheel, right_front_wheel,
                    left_front_wheel, right_back_wheel,-power,power,acceleration_distance,number_of_stages);
*/
            move_holonomic_with_encoder(left_back_wheel, right_front_wheel,
                    left_front_wheel,right_back_wheel,
                    -power,power , move_distance , break_at_end);
/*
            deceleration(left_back_wheel, right_front_wheel,
                    left_front_wheel, right_back_wheel,-power,power,deceleration_distance,number_of_stages);
        */
        }

    }


////////////////////////////////////////////////////////////////////////////////////////////////////

    public void move_arm_without_encoder (DcMotor arm_motor ,double power, char direction)
    {
        if (direction == 'U'){
            arm_motor.setPower(dc_motor_power_adapter(power));

        }else if (direction== 'D'){
            arm_motor.setPower(dc_motor_power_adapter(power));
        }
    }











    /**
     *METHOD:  Hankash_spin_with_encoder
     *
     *  used to spin the robot right(clockwise) and left (anti-clockwise)
     *
     * parameters: left_back_wheel, left_front_wheel,
     *                right_back_wheel,right_front_wheel,
     *                 power,direction distance,break_at_end
     *
     * return void
     */


    public void Hankash_spin_with_encoder(DcMotor left_back_wheel, DcMotor left_front_wheel,
                                          DcMotor right_back_wheel, DcMotor right_front_wheel,
                                          double power , int Given_Degree , char direction,boolean break_at_end){
        double num_of_rotations = (robot_spin_circumference/wheel_circumference);
        double degrees_per_rotation = 360 / num_of_rotations ;
        double distance = (Given_Degree*wheel_circumference/degrees_per_rotation);
        stop_and_reset(left_back_wheel,left_front_wheel,right_back_wheel, right_front_wheel);
        if(direction == 'C'){
            move_holonomic_with_encoder(left_back_wheel,left_front_wheel,right_back_wheel, right_front_wheel,
                    power,-power,distance,break_at_end );
        }
        else if(direction == 'A'){
            move_holonomic_with_encoder(left_back_wheel,left_front_wheel,right_back_wheel, right_front_wheel,
                    -power,power,distance,break_at_end );
        }
    }

    /**
     *METHOD:  spin_acceleration
     *
     *  to accelerate the spinning power
     *
     * parameters: left_back_wheel, left_front_wheel,
     *             right_back_wheel, right_front_wheel,
     *             power,int given_degrees, direction , number_of_stages
     *
     * return void
     */



    public void spin_acceleration(DcMotor left_back_wheel, DcMotor left_front_wheel,
                                  DcMotor right_back_wheel, DcMotor right_front_wheel,
                                  double power,int given_degrees,char direction ,int number_of_stages) {
        int degrees_in_stage = given_degrees / number_of_stages;
        double power_in_stage = power / number_of_stages;

        double total_power = 0;

        for (int current_stages = 0; current_stages < number_of_stages; current_stages++) {

            total_power += power_in_stage;
            Hankash_spin_with_encoder(right_back_wheel, right_front_wheel,
                    left_back_wheel, left_front_wheel, total_power, degrees_in_stage, direction, FALSE);


        }




    }
    /**
     *METHOD:  spin_deceleration
     *
     *  to decelerate the spinning power
     *
     * parameters: left_back_wheel, left_front_wheel,
     *             right_back_wheel, right_front_wheel,
     *             power,int given_degrees, direction , number_of_stages
     *
     * return void
     */

    public void spin_deceleration(DcMotor left_back_wheel, DcMotor left_front_wheel,
                                  DcMotor right_back_wheel, DcMotor right_front_wheel,
                                  double power,int given_degrees,char direction ,int number_of_stages) {
        int degrees_in_stage = given_degrees / number_of_stages;
        double power_in_stage = power / number_of_stages;

        double total_power = power;

        for (int current_stages = 0; current_stages < number_of_stages; current_stages++) {
            total_power -= power_in_stage;
            if(current_stages==number_of_stages-1){
                Hankash_spin_with_encoder(right_back_wheel, right_front_wheel,
                        left_back_wheel, left_front_wheel, total_power, degrees_in_stage, direction, TRUE);

            }
            else{
                Hankash_spin_with_encoder(right_back_wheel, right_front_wheel,
                        left_back_wheel, left_front_wheel, total_power, degrees_in_stage, direction, FALSE);
            }

        }


    }



    public void spin_with_encoder(DcMotor left_back_wheel, DcMotor left_front_wheel,
                                  DcMotor right_back_wheel, DcMotor right_front_wheel,
                                  double power ,int acceleration_degrees, int spin_Degree ,
                                  int deceleration_degrees,int number_of_stages, char direction){

       // spin_acceleration(right_back_wheel,  right_front_wheel, left_back_wheel,
          //      left_front_wheel,power,acceleration_degrees,direction,number_of_stages);

        Hankash_spin_with_encoder(right_back_wheel,  right_front_wheel, left_back_wheel,
                left_front_wheel,power,spin_Degree,direction,FALSE);

       // spin_deceleration(right_back_wheel,  right_front_wheel, left_back_wheel,
           //     left_front_wheel,power,deceleration_degrees,direction,number_of_stages);

    }





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

        for (double position = from; position <= to; position += 1.0){
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

        for (double position = from; position >= to; position -= 1.0) {
            double new_position = servo_motor_degrees_adapter(position);
            servo_motor.setPosition(new_position);


        }

    }

















    public void move_arm_with_emcoder(DcMotor arm_motor,double power , char direction,double distance)
    {
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_motor.setTargetPosition(cm_to_ticks(distance));

        if(direction== 'U'){

            arm_motor.setPower(dc_motor_power_adapter(power));
        }else if(direction=='D'){

            arm_motor.setPower(dc_motor_power_adapter(-power));
        }
        while (arm_motor.isBusy());
        arm_motor.setPower(0);
    }

public void stop_and_reset(DcMotor first_motor, DcMotor second_motor,
                           DcMotor third_motor, DcMotor fourth_motor)
{
    first_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    second_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    third_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    fourth_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

}







     public void acceleration_deceleration_manual (DcMotor first_motor, DcMotor second_motor,
                                                  DcMotor third_motor, DcMotor fourth_motor, double power_acceleration,
                                                  double power_decceleration ,double power,double distanc_acceleration
                                                            , double diistance , double distance_deceleration)
    {

        first_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        second_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        third_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fourth_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        first_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        second_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        third_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourth_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        first_motor.setTargetPosition(cm_to_ticks(distanc_acceleration)/3);
        second_motor.setTargetPosition(cm_to_ticks(distanc_acceleration)/3);
        third_motor.setTargetPosition(cm_to_ticks(distanc_acceleration)/3);
        fourth_motor.setTargetPosition(cm_to_ticks(distanc_acceleration)/3);

        first_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        second_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        third_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fourth_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        first_motor.setPower(dc_motor_power_adapter(power_acceleration*(1/3)));
        second_motor.setPower(dc_motor_power_adapter(power_acceleration*(1/3)));
        third_motor.setPower(dc_motor_power_adapter(power_acceleration*(1/3)));
        fourth_motor.setPower(dc_motor_power_adapter(power_acceleration*(1/3)));


        while (first_motor.isBusy()&&second_motor.isBusy()&&third_motor.isBusy()&&fourth_motor.isBusy())
        {

        }
        first_motor.setPower(dc_motor_power_adapter(0));
        second_motor.setPower(dc_motor_power_adapter(0));
        third_motor.setPower(dc_motor_power_adapter(0));
        fourth_motor.setPower(dc_motor_power_adapter(0));

        //0000000000000000000000000000000000000000000000000000000001
/*

        first_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        second_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        third_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourth_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        first_motor.setTargetPosition(cm_to_ticks(distanc_acceleration)/3);
        second_motor.setTargetPosition(cm_to_ticks(distanc_acceleration)/3);
        third_motor.setTargetPosition(cm_to_ticks(distanc_acceleration)/3);
        fourth_motor.setTargetPosition(cm_to_ticks(distanc_acceleration)/3);


        first_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        second_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        third_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fourth_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        first_motor.setPower(dc_motor_power_adapter(power_acceleration*(2/3)));
        second_motor.setPower(dc_motor_power_adapter(power_acceleration*(2/3)));
        third_motor.setPower(dc_motor_power_adapter(power_acceleration*(2/3)));
        fourth_motor.setPower(dc_motor_power_adapter(power_acceleration*(2/3)));


        while (first_motor.isBusy()&&second_motor.isBusy()&&third_motor.isBusy()&&fourth_motor.isBusy())
        {

        }
        first_motor.setPower(dc_motor_power_adapter(0));
        second_motor.setPower(dc_motor_power_adapter(0));
        third_motor.setPower(dc_motor_power_adapter(0));
        fourth_motor.setPower(dc_motor_power_adapter(0));
        //0000000000000000000000000000000000000000000000000000000002

        first_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        second_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        third_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourth_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        first_motor.setTargetPosition(cm_to_ticks(distanc_acceleration)/3);
        second_motor.setTargetPosition(cm_to_ticks(distanc_acceleration)/3);
        third_motor.setTargetPosition(cm_to_ticks(distanc_acceleration)/3);
        fourth_motor.setTargetPosition(cm_to_ticks(distanc_acceleration)/3);


        first_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        second_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        third_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fourth_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        first_motor.setPower(dc_motor_power_adapter(power_acceleration));
        second_motor.setPower(dc_motor_power_adapter(power_acceleration));
        third_motor.setPower(dc_motor_power_adapter(power_acceleration));
        fourth_motor.setPower(dc_motor_power_adapter(power_acceleration));


        while (first_motor.isBusy()&&second_motor.isBusy()&&third_motor.isBusy()&&fourth_motor.isBusy())
        {

        }
        first_motor.setPower(dc_motor_power_adapter(0));
        second_motor.setPower(dc_motor_power_adapter(0));
        third_motor.setPower(dc_motor_power_adapter(0));
        fourth_motor.setPower(dc_motor_power_adapter(0));
        //0000000000000000000000000000000000000000000000000000000003

        first_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        second_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        third_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourth_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        first_motor.setTargetPosition(cm_to_ticks(diistance));
        second_motor.setTargetPosition(cm_to_ticks(diistance));
        third_motor.setTargetPosition(cm_to_ticks(diistance));
        fourth_motor.setTargetPosition(cm_to_ticks(diistance));

        first_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        second_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        third_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fourth_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        first_motor.setPower(dc_motor_power_adapter(power));
        second_motor.setPower(dc_motor_power_adapter(power));
        third_motor.setPower(dc_motor_power_adapter(power));
        fourth_motor.setPower(dc_motor_power_adapter(power));


        while (first_motor.isBusy()&&second_motor.isBusy()&&third_motor.isBusy()&&fourth_motor.isBusy())
        {

        }
        first_motor.setPower(dc_motor_power_adapter(0));
        second_motor.setPower(dc_motor_power_adapter(0));
        third_motor.setPower(dc_motor_power_adapter(0));
        fourth_motor.setPower(dc_motor_power_adapter(0));
        //0000000000000000000000000000000000000000000000000000000004

        first_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        second_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        third_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourth_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        first_motor.setTargetPosition(cm_to_ticks(distance_deceleration)/3);
        second_motor.setTargetPosition(cm_to_ticks(distance_deceleration)/3);
        third_motor.setTargetPosition(cm_to_ticks(distance_deceleration)/3);
        fourth_motor.setTargetPosition(cm_to_ticks(distance_deceleration)/3);

        first_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        second_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        third_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fourth_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        first_motor.setPower(dc_motor_power_adapter(power_decceleration));
        second_motor.setPower(dc_motor_power_adapter(power_decceleration));
        third_motor.setPower(dc_motor_power_adapter(power_decceleration));
        fourth_motor.setPower(dc_motor_power_adapter(power_decceleration));


        while (first_motor.isBusy()&&second_motor.isBusy()&&third_motor.isBusy()&&fourth_motor.isBusy())
        {

        }
        first_motor.setPower(dc_motor_power_adapter(0));
        second_motor.setPower(dc_motor_power_adapter(0));
        third_motor.setPower(dc_motor_power_adapter(0));
        fourth_motor.setPower(dc_motor_power_adapter(0));
        //0000000000000000000000000000000000000000000000000000000005

        first_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        second_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        third_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourth_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        first_motor.setTargetPosition(cm_to_ticks(distance_deceleration)/3);
        second_motor.setTargetPosition(cm_to_ticks(distance_deceleration)/3);
        third_motor.setTargetPosition(cm_to_ticks(distance_deceleration)/3);
        fourth_motor.setTargetPosition(cm_to_ticks(distance_deceleration)/3);

        first_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        second_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        third_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fourth_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        first_motor.setPower(dc_motor_power_adapter(power_decceleration*(2/3)));
        second_motor.setPower(dc_motor_power_adapter(power_decceleration*(2/3)));
        third_motor.setPower(dc_motor_power_adapter(power_decceleration*(2/3)));
        fourth_motor.setPower(dc_motor_power_adapter(power_decceleration*(2/3)));


        while (first_motor.isBusy()&&second_motor.isBusy()&&third_motor.isBusy()&&fourth_motor.isBusy())
        {

        }
        first_motor.setPower(dc_motor_power_adapter(0));
        second_motor.setPower(dc_motor_power_adapter(0));
        third_motor.setPower(dc_motor_power_adapter(0));
        fourth_motor.setPower(dc_motor_power_adapter(0));
        //0000000000000000000000000000000000000000000000000000000006

        first_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        second_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        third_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourth_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        first_motor.setTargetPosition(cm_to_ticks(distance_deceleration)/3);
        second_motor.setTargetPosition(cm_to_ticks(distance_deceleration)/3);
        third_motor.setTargetPosition(cm_to_ticks(distance_deceleration)/3);
        fourth_motor.setTargetPosition(cm_to_ticks(distance_deceleration)/3);

        first_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        second_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        third_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fourth_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        first_motor.setPower(dc_motor_power_adapter(power_decceleration*(1/3)));
        second_motor.setPower(dc_motor_power_adapter(power_decceleration*(1/3)));
        third_motor.setPower(dc_motor_power_adapter(power_decceleration*(1/3)));
        fourth_motor.setPower(dc_motor_power_adapter(power_decceleration*(1/3)));


        while (first_motor.isBusy()&&second_motor.isBusy()&&third_motor.isBusy()&&fourth_motor.isBusy())
        {

        }

        first_motor.setPower(dc_motor_power_adapter(0));
        second_motor.setPower(dc_motor_power_adapter(0));
        third_motor.setPower(dc_motor_power_adapter(0));
        fourth_motor.setPower(dc_motor_power_adapter(0));
        //0000000000000000000000000000000000000000000000000000000007
*/
    }



    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }






    void spin_with_gyro(DcMotor left_back_motor,DcMotor left_front_motor,DcMotor right_back_motor,DcMotor right_front_motor,
                        double gyro_start ,double angel,double power ,char direction){

    if(direction == 'C'){


        spin_without_encoder(left_back_motor,left_front_motor,right_back_motor,right_front_motor,power,direction);
    }else if(direction == 'A'){
        spin_without_encoder(left_back_motor,left_front_motor,right_back_motor,right_front_motor,power,direction);
    }

    }







    public void spin(DcMotor left_back_motor,DcMotor left_front_motor,DcMotor right_back_motor,DcMotor right_front_motot , double gevin_degre){

    }



    public void move_with_encoderr(DcMotor left_back_motor,DcMotor left_front_motor,
                                   DcMotor right_back_motor,DcMotor right_front_motor
                                ,double distance , double power , boolean break_at_end)
    {

        if (power > 0 )
        {
            left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() + cm_to_ticks(distance));
            left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() + cm_to_ticks(distance));
            right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() + cm_to_ticks(distance));
            right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() + cm_to_ticks(distance));


            int target_ticks = left_back_motor.getCurrentPosition() + cm_to_ticks(distance);



            left_back_motor.setPower(dc_motor_power_adapter(power));
            left_front_motor.setPower(dc_motor_power_adapter(power));
            right_back_motor.setPower(dc_motor_power_adapter(power));
            right_front_motor.setPower(dc_motor_power_adapter(power));


            while ((left_back_motor.getCurrentPosition() < target_ticks) && (left_front_motor.getCurrentPosition() < target_ticks)
                    && (right_back_motor.getCurrentPosition() < target_ticks) && (right_front_motor.getCurrentPosition() < target_ticks))
                ;


            left_back_motor.setPower(dc_motor_power_adapter(0));
            left_front_motor.setPower(dc_motor_power_adapter(0));
            right_back_motor.setPower(dc_motor_power_adapter(0));
            right_front_motor.setPower(dc_motor_power_adapter(0));

        }else if(power < 0){

            left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() - cm_to_ticks(distance));
            left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() - cm_to_ticks(distance));
            right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() - cm_to_ticks(distance));
            right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() - cm_to_ticks(distance));


            int target_ticks = left_back_motor.getCurrentPosition() - cm_to_ticks(distance);


            left_back_motor.setPower(dc_motor_power_adapter(power));
            left_front_motor.setPower(dc_motor_power_adapter(power));
            right_back_motor.setPower(dc_motor_power_adapter(power));
            right_front_motor.setPower(dc_motor_power_adapter(power));


            while ((left_back_motor.getCurrentPosition() > target_ticks) && (left_front_motor.getCurrentPosition() > target_ticks)
                    && (right_back_motor.getCurrentPosition() > target_ticks) && (right_front_motor.getCurrentPosition() > target_ticks))
                ;


            left_back_motor.setPower(dc_motor_power_adapter(0));
            left_front_motor.setPower(dc_motor_power_adapter(0));
            right_back_motor.setPower(dc_motor_power_adapter(0));
            right_front_motor.setPower(dc_motor_power_adapter(0));

        }else if (power ==0){
            left_back_motor.setPower(dc_motor_power_adapter(0));
            left_front_motor.setPower(dc_motor_power_adapter(0));
            right_back_motor.setPower(dc_motor_power_adapter(0));
            right_front_motor.setPower(dc_motor_power_adapter(0));
        }


    }








    public void move_diagonale_left_with_encoderr(DcMotor left_back_motor,DcMotor left_front_motor,
                                                  DcMotor right_back_motor,DcMotor right_front_motor
                                                    ,double distance , double power , boolean break_at_end)
    {

        if (power > 0 ) {
            left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() + cm_to_ticks(distance));

            right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() + cm_to_ticks(distance));


            int target_ticks = left_back_motor.getCurrentPosition() + cm_to_ticks(distance);


            left_back_motor.setPower(dc_motor_power_adapter(power));

            right_front_motor.setPower(dc_motor_power_adapter(power));


            while ((left_back_motor.getCurrentPosition() < target_ticks) && (right_front_motor.getCurrentPosition() < target_ticks))
                ;


            left_back_motor.setPower(dc_motor_power_adapter(0));

            right_front_motor.setPower(dc_motor_power_adapter(0));

        }else if(power < 0){

            left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() - cm_to_ticks(distance));

            right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() - cm_to_ticks(distance));


            int target_ticks = left_back_motor.getCurrentPosition() - cm_to_ticks(distance);


            left_back_motor.setPower(dc_motor_power_adapter(power));

            right_front_motor.setPower(dc_motor_power_adapter(power));


            while ((left_back_motor.getCurrentPosition() > target_ticks) && (right_front_motor.getCurrentPosition() > target_ticks))
                ;


            left_back_motor.setPower(dc_motor_power_adapter(0));

            right_front_motor.setPower(dc_motor_power_adapter(0));

        }else if (power ==0){
            left_back_motor.setPower(dc_motor_power_adapter(0));

            right_front_motor.setPower(dc_motor_power_adapter(0));
        }


    }






    public void move_diagonale_right_with_encoderr(DcMotor left_back_motor,DcMotor left_front_motor,
                                                   DcMotor right_back_motor,DcMotor right_front_motor
                                            ,double distance , double power , boolean break_at_end){

        if (power > 0 ) {

            left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() + cm_to_ticks(distance));
            right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() + cm_to_ticks(distance));



            int target_ticks = left_front_motor.getCurrentPosition() + cm_to_ticks(distance);



            left_front_motor.setPower(dc_motor_power_adapter(power));
            right_back_motor.setPower(dc_motor_power_adapter(power));



            while ((left_front_motor.getCurrentPosition() < target_ticks) && (right_back_motor.getCurrentPosition() < target_ticks))
                ;



            left_front_motor.setPower(dc_motor_power_adapter(0));
            right_back_motor.setPower(dc_motor_power_adapter(0));


        }else if(power < 0){


            left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() - cm_to_ticks(distance));
            right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() - cm_to_ticks(distance));



            int target_ticks = left_front_motor.getCurrentPosition() - cm_to_ticks(distance);


            left_front_motor.setPower(dc_motor_power_adapter(power));
            right_back_motor.setPower(dc_motor_power_adapter(power));


            while ((left_front_motor.getCurrentPosition() > target_ticks) && (right_back_motor.getCurrentPosition() > target_ticks))
                ;



            left_front_motor.setPower(dc_motor_power_adapter(0));
            right_back_motor.setPower(dc_motor_power_adapter(0));


        }else if (power ==0){

            left_front_motor.setPower(dc_motor_power_adapter(0));
            right_back_motor.setPower(dc_motor_power_adapter(0));

        }

    }



    public void move_side_with_encoderr(DcMotor left_back_motor,DcMotor left_front_motor,DcMotor right_back_motor,DcMotor right_front_motor
            ,double distance , double power , boolean break_at_end){

        if (power > 0 ) {
            left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() - cm_to_ticks(distance));
            left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() + cm_to_ticks(distance));
            right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() + cm_to_ticks(distance));
            right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() - cm_to_ticks(distance));


            int target_ticks = left_back_motor.getCurrentPosition() - cm_to_ticks(distance);
            int target_ticks2 = left_front_motor.getCurrentPosition() + cm_to_ticks(distance);


            left_back_motor.setPower(dc_motor_power_adapter(-power));
            left_front_motor.setPower(dc_motor_power_adapter(power));
            right_back_motor.setPower(dc_motor_power_adapter(power));
            right_front_motor.setPower(dc_motor_power_adapter(-power));


            while ((left_back_motor.getCurrentPosition() > target_ticks) && (left_front_motor.getCurrentPosition() < target_ticks2)
                    && (right_back_motor.getCurrentPosition() < target_ticks2) && (right_front_motor.getCurrentPosition() > target_ticks))
                ;


            left_back_motor.setPower(dc_motor_power_adapter(0));
            left_front_motor.setPower(dc_motor_power_adapter(0));
            right_back_motor.setPower(dc_motor_power_adapter(0));
            right_front_motor.setPower(dc_motor_power_adapter(0));

        }else if(power < 0){

            left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() + cm_to_ticks(distance));
            left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() - cm_to_ticks(distance));
            right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() - cm_to_ticks(distance));
            right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() + cm_to_ticks(distance));


            int target_ticks = left_back_motor.getCurrentPosition() + cm_to_ticks(distance);
            int target_ticks2 = left_front_motor.getCurrentPosition() - cm_to_ticks(distance);



            left_back_motor.setPower(dc_motor_power_adapter(power));
            left_front_motor.setPower(dc_motor_power_adapter(-power));
            right_back_motor.setPower(dc_motor_power_adapter(-power));
            right_front_motor.setPower(dc_motor_power_adapter(power));


            while ((left_back_motor.getCurrentPosition() < target_ticks) && (left_front_motor.getCurrentPosition() > target_ticks2)
                    && (right_back_motor.getCurrentPosition() > target_ticks2) && (right_front_motor.getCurrentPosition() < target_ticks))
                ;


            left_back_motor.setPower(dc_motor_power_adapter(0));
            left_front_motor.setPower(dc_motor_power_adapter(0));
            right_back_motor.setPower(dc_motor_power_adapter(0));
            right_front_motor.setPower(dc_motor_power_adapter(0));

        }else if (power ==0){
            left_back_motor.setPower(dc_motor_power_adapter(0));
            left_front_motor.setPower(dc_motor_power_adapter(0));
            right_back_motor.setPower(dc_motor_power_adapter(0));
            right_front_motor.setPower(dc_motor_power_adapter(0));
        }


    }



    public void move_arm ( DcMotor arm_motor,  double power, int ticks,TouchSensor touch_sensor)
    {

        if (power>0)
        {


            int target_ticks = arm_motor.getCurrentPosition() + ticks;


            if( target_ticks <= 15000 ) {

                arm_motor.setTargetPosition(target_ticks);

                arm_motor.setPower(dc_motor_power_adapter(power));

                while ((arm_motor.getCurrentPosition() < target_ticks));

                arm_motor.setPower(dc_motor_power_adapter(0));


            } else   {


            }



        }else if(power<0){

            if(!touch_sensor.isPressed())
            {
                int target_ticks = arm_motor.getCurrentPosition() - ticks;

                arm_motor.setTargetPosition(target_ticks);


                arm_motor.setPower(dc_motor_power_adapter(power));

                while ((arm_motor.getCurrentPosition() > target_ticks));

                arm_motor.setPower(dc_motor_power_adapter(0));


            }


        }
        else if(power == 0){
            arm_motor.setPower(0);

        }

    }

    public void spin_with_encoderr(DcMotor left_back_motor, DcMotor left_front_motor,
                                  DcMotor right_back_motor, DcMotor right_front_motor,
                                  double power , int Given_Degree , char direction,boolean break_at_end)
    {
        double num_of_rotations = (robot_spin_circumference/wheel_circumference);
        double degrees_per_rotation = 360 / num_of_rotations ;
        double distance = (Given_Degree*wheel_circumference/degrees_per_rotation);

        if(direction== 'C'){
            if (power > 0 )
            {
                left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() + cm_to_ticks(distance));
                left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() + cm_to_ticks(distance));
                right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() + cm_to_ticks(distance));
                right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() + cm_to_ticks(distance));


                int target_ticks = left_back_motor.getCurrentPosition() + cm_to_ticks(distance);



                left_back_motor.setPower(dc_motor_power_adapter(power));
                left_front_motor.setPower(dc_motor_power_adapter(power));
                right_back_motor.setPower(dc_motor_power_adapter(-power));
                right_front_motor.setPower(dc_motor_power_adapter(-power));


                while ((left_back_motor.getCurrentPosition() > target_ticks) && (left_front_motor.getCurrentPosition() > target_ticks)
                        && (right_back_motor.getCurrentPosition() < target_ticks) && (right_front_motor.getCurrentPosition() < target_ticks))
                    ;


                left_back_motor.setPower(dc_motor_power_adapter(0));
                left_front_motor.setPower(dc_motor_power_adapter(0));
                right_back_motor.setPower(dc_motor_power_adapter(0));
                right_front_motor.setPower(dc_motor_power_adapter(0));

            }else if(power < 0){

                left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() - cm_to_ticks(distance));
                left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() - cm_to_ticks(distance));
                right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() - cm_to_ticks(distance));
                right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() - cm_to_ticks(distance));


                int target_ticks = left_back_motor.getCurrentPosition() - cm_to_ticks(distance);


                left_back_motor.setPower(dc_motor_power_adapter(-power));
                left_front_motor.setPower(dc_motor_power_adapter(-power));
                right_back_motor.setPower(dc_motor_power_adapter(power));
                right_front_motor.setPower(dc_motor_power_adapter(power));


                while ((left_back_motor.getCurrentPosition() < target_ticks) && (left_front_motor.getCurrentPosition() < target_ticks)
                        && (right_back_motor.getCurrentPosition() > target_ticks) && (right_front_motor.getCurrentPosition() > target_ticks))
                    ;


                left_back_motor.setPower(dc_motor_power_adapter(0));
                left_front_motor.setPower(dc_motor_power_adapter(0));
                right_back_motor.setPower(dc_motor_power_adapter(0));
                right_front_motor.setPower(dc_motor_power_adapter(0));

            }else if (power ==0){
                left_back_motor.setPower(dc_motor_power_adapter(0));
                left_front_motor.setPower(dc_motor_power_adapter(0));
                right_back_motor.setPower(dc_motor_power_adapter(0));
                right_front_motor.setPower(dc_motor_power_adapter(0));
            }



        }
        else if(direction=='A'){
            if (power > 0 )
            {
                left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() + cm_to_ticks(distance));
                left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() + cm_to_ticks(distance));
                right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() + cm_to_ticks(distance));
                right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() + cm_to_ticks(distance));


                int target_ticks = left_back_motor.getCurrentPosition() + cm_to_ticks(distance);



                left_back_motor.setPower(dc_motor_power_adapter(-power));
                left_front_motor.setPower(dc_motor_power_adapter(-power));
                right_back_motor.setPower(dc_motor_power_adapter(power));
                right_front_motor.setPower(dc_motor_power_adapter(power));


                while ((left_back_motor.getCurrentPosition() < target_ticks) && (left_front_motor.getCurrentPosition() < target_ticks)
                        && (right_back_motor.getCurrentPosition() > target_ticks) && (right_front_motor.getCurrentPosition() > target_ticks))
                    ;


                left_back_motor.setPower(dc_motor_power_adapter(0));
                left_front_motor.setPower(dc_motor_power_adapter(0));
                right_back_motor.setPower(dc_motor_power_adapter(0));
                right_front_motor.setPower(dc_motor_power_adapter(0));

            }else if(power < 0){

                left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() - cm_to_ticks(distance));
                left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() - cm_to_ticks(distance));
                right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() - cm_to_ticks(distance));
                right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() - cm_to_ticks(distance));


                int target_ticks = left_back_motor.getCurrentPosition() - cm_to_ticks(distance);


                left_back_motor.setPower(dc_motor_power_adapter(power));
                left_front_motor.setPower(dc_motor_power_adapter(power));
                right_back_motor.setPower(dc_motor_power_adapter(-power));
                right_front_motor.setPower(dc_motor_power_adapter(-power));


                while ((left_back_motor.getCurrentPosition() > target_ticks) && (left_front_motor.getCurrentPosition() > target_ticks)
                        && (right_back_motor.getCurrentPosition() < target_ticks) && (right_front_motor.getCurrentPosition() < target_ticks))
                    ;

                left_back_motor.setPower(dc_motor_power_adapter(0));
                left_front_motor.setPower(dc_motor_power_adapter(0));
                right_back_motor.setPower(dc_motor_power_adapter(0));
                right_front_motor.setPower(dc_motor_power_adapter(0));

            }else if (power ==0){
                left_back_motor.setPower(dc_motor_power_adapter(0));
                left_front_motor.setPower(dc_motor_power_adapter(0));
                right_back_motor.setPower(dc_motor_power_adapter(0));
                right_front_motor.setPower(dc_motor_power_adapter(0));
            }

        }






    }



}
