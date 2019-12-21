package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name="TahaBlueAllianceFTC" , group = "FTC")

public class TahaBlueAllianceFTC extends  LinearOpMode{

    private DcMotor left_back_motor = null;
    private DcMotor left_front_motor = null;
    private DcMotor right_back_motor = null;
    private DcMotor right_front_motor = null;
    private DcMotor arm_motor = null;
    Servo right_foundation_servo = null;
    Servo left_foundation_servo = null ;
    Servo arm_servo = null;
    Servo gripper_servo = null;
    private ModernRoboticsI2cColorSensor color_sensor = null ;

    private TouchSensor min_end_stop;
    DigitalChannel max_end_stop;

    double move_power = 70;
    double diagonal_power = 40;
    double side_power = 65;
    double spin_power = 0;
    double arm_power = 0;
    double stop_power = 0;
    int flag=0;


    double gyro_start;
    double gyro_angel;

    HelperClass helper_class_object = new HelperClass();
    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    @Override
    public void runOpMode() throws InterruptedException {
        left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
        left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");

        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
        arm_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        right_foundation_servo = hardwareMap.get(Servo.class, "right_foundation_servo");
        left_foundation_servo = hardwareMap.get(Servo.class, "left_foundation_servo");
        arm_servo = hardwareMap.get(Servo.class, "arm_servo");
        gripper_servo = hardwareMap.get(Servo.class, "gripper_servo");


        min_end_stop = hardwareMap.get(TouchSensor.class, "min_end_stop");

        max_end_stop = hardwareMap.get(DigitalChannel.class, "max_end_stop");
        max_end_stop.setMode(DigitalChannel.Mode.INPUT);


        color_sensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class,"color_sensor");


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        waitForStart();

        if(gamepad1.y){
            flag = 4 ;
            telemetry.addData("Alliance\t","1-4");
            telemetry.update();
        }else if(gamepad1.b){
            flag = 5 ;
            telemetry.addData("Alliance\t","2-5");
            telemetry.update();
        }else if(gamepad1.a){
            flag = 6 ;
            telemetry.addData("Alliance\t","3-6");
            telemetry.update();
        }



        move_with_pid(left_back_motor,left_front_motor,right_back_motor,right_front_motor,100,move_power);


        if(flag==4){
             move_side_with_pid(left_back_motor,left_front_motor,right_back_motor,right_front_motor,10,stop_power);
        }else if(flag==5){
             move_side_with_pid(left_back_motor,left_front_motor,right_back_motor,right_front_motor,10,-side_power);
        }
        else if(flag==6){
             move_side_with_pid(left_back_motor,left_front_motor,right_back_motor,right_front_motor,20,-side_power);
        }



        move_with_pid(left_back_motor,left_front_motor,right_back_motor,right_front_motor,50,-move_power);


        if(flag==4){
             move_side_with_pid(left_back_motor,left_front_motor,right_back_motor,right_front_motor,290,-side_power);
         }else if(flag==5){
             move_side_with_pid(left_back_motor,left_front_motor,right_back_motor,right_front_motor,280,-side_power);
        }else if(flag==6){
             move_side_with_pid(left_back_motor,left_front_motor,right_back_motor,right_front_motor,270,-side_power);
        }


        move_with_pid(left_back_motor,left_front_motor,right_back_motor,right_front_motor,60,move_power);


    }

    public void move_with_pid(DcMotor left_back_motor, DcMotor left_front_motor, DcMotor right_back_motor, DcMotor right_front_motor
            , double distance , double power ) {

        left_back_motor.setDirection(DcMotor.Direction.FORWARD);
        left_front_motor.setDirection(DcMotor.Direction.FORWARD);
        right_back_motor.setDirection(DcMotor.Direction.REVERSE);
        right_front_motor.setDirection(DcMotor.Direction.REVERSE);




        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        double gyro_start = gyro_angel ;
        double left_power ;
        double right_power ;

        if(power>0){

            left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));
            left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));
            right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));
            right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance));

            int left_back_motor_target_ticks = left_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
            int left_front_motor_target_ticks = left_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
            int right_back_motor_target_ticks = right_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
            int right_front_motor_target_ticks = right_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);



            double error = 0 ;
            double last_error = 0;
            double KP = 5;  //.3
            double KI = 0;  //.001
            double KD = 0;  //.2
            double probational=0 ;
            double derivative=0 ;
            double integral=0 ;

            while ((left_back_motor.getCurrentPosition() < left_back_motor_target_ticks) && (left_front_motor.getCurrentPosition() < left_front_motor_target_ticks)
                    && (right_back_motor.getCurrentPosition() < right_back_motor_target_ticks) && (right_front_motor.getCurrentPosition() < right_front_motor_target_ticks))
            {
                imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

                error= gyro_angel - gyro_start;

                probational = error ;
                integral = integral + error ;
                derivative = error - last_error;

                left_power = power + ( (probational * KP ) + (integral*KI) + (derivative*KD) ) ;
                right_power = power - ( (probational * KP ) + (integral*KI) + (derivative*KD) ) ;




                left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power),-100,100));
                left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power),-100,100));
                right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power),-100,100));
                right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power),-100,100));

                sleep(1);

                last_error = error ;

                telemetry.addData("LEFT_POWER and RIGHT_POWER","%d %d",left_back_motor.getCurrentPosition(),right_back_motor.getCurrentPosition());
                telemetry.update();
            }


            left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));

            sleep(1);



        }else if(power<0){
            left_back_motor.setTargetPosition(left_back_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance));
            left_front_motor.setTargetPosition(left_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance));
            right_back_motor.setTargetPosition(right_back_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance));
            right_front_motor.setTargetPosition(right_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance));

            int left_back_motor_target_ticks = left_back_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);
            int left_front_motor_target_ticks = left_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);
            int right_back_motor_target_ticks = right_back_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);
            int right_front_motor_target_ticks = right_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);



            double error = 0 ;
            double last_error = 0;
            double KP = 5;  //.3
            double KI = 0;  //.001
            double KD = 0;  //.2
            double probational=0 ;
            double derivative=0 ;
            double integral=0 ;

            while ((left_back_motor.getCurrentPosition() > left_back_motor_target_ticks) && (left_front_motor.getCurrentPosition() > left_front_motor_target_ticks)
                    && (right_back_motor.getCurrentPosition() > right_back_motor_target_ticks) && (right_front_motor.getCurrentPosition() > right_front_motor_target_ticks))
            {
                imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

                error = gyro_angel - gyro_start;

                probational = error ;
                integral = integral + error ;
                derivative = error - last_error;

                left_power = power + ( (probational * KP ) + (integral*KI) + (derivative*KD) ) ;
                right_power = power - ( (probational * KP ) + (integral*KI) + (derivative*KD) ) ;




                left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power),-100,100));
                left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(left_power),-100,100));
                right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power),-100,100));
                right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(right_power),-100,100));

                sleep(1);

                last_error = error ;

                telemetry.addData("LEFT_POWER and RIGHT_POWER","%d %d",left_back_motor.getCurrentPosition(),right_back_motor.getCurrentPosition());
                telemetry.update();
            }


            left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));

            sleep(1);
        }else if(power == 0){


        }



    }


    public void move_side_with_pid(DcMotor left_back_motor,DcMotor left_front_motor,DcMotor right_back_motor,DcMotor right_front_motor
            ,double distance , double power){


        left_back_motor.setDirection(DcMotor.Direction.REVERSE);
        left_front_motor.setDirection(DcMotor.Direction.FORWARD);
        right_back_motor.setDirection(DcMotor.Direction.REVERSE);
        right_front_motor.setDirection(DcMotor.Direction.FORWARD);


        if (power > 0 ) {
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


            double gyro_start = gyro_angel ;
            double increase_power ;
            double decrese_power ;
            double error = 0 ;
            double last_error = 0;
            double KP = 5;
            double KI = 0;
            double KD = 0;
            double probational=0 ;
            double derivative=0 ;
            double integral=0 ;

            int left_back_motor_target_ticks = left_back_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);
            int left_front_motor_target_ticks = left_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
            int right_back_motor_target_ticks = right_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
            int right__front_motor_target_ticks = right_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);



            left_back_motor.setTargetPosition(left_back_motor_target_ticks);
            left_front_motor.setTargetPosition(left_front_motor_target_ticks);
            right_back_motor.setTargetPosition(right_back_motor_target_ticks);
            right_front_motor.setTargetPosition(right__front_motor_target_ticks);



            while ((left_back_motor.getCurrentPosition() > left_back_motor_target_ticks) && (left_front_motor.getCurrentPosition() < left_front_motor_target_ticks)
                    && (right_back_motor.getCurrentPosition() < right_back_motor_target_ticks) && (right_front_motor.getCurrentPosition() > right__front_motor_target_ticks))
            {

                imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


                error = gyro_angel - gyro_start;

                probational = error;
                integral = integral + error;
                derivative = error - last_error;

                increase_power = power + ((probational * KP) + (integral * KI) + (derivative * KD));
                decrese_power = power - ((probational * KP) + (integral * KI) + (derivative * KD));

                left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(power), -100, 100));
                left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(increase_power), -100, 100));
                right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(decrese_power), -100, 100));
                right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(power), -100, 100));

                sleep(1);

                last_error = error;

                telemetry.addData("decrese_power and increase_power", "%f %f", decrese_power, increase_power);
                telemetry.update();

            }
            left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));

            sleep(1);

        }else if(power < 0){
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

            double gyro_start = gyro_angel ;
            double increase_power ;
            double decrese_power ;
            double error = 0 ;
            double last_error = 0;
            double KP = 5;
            double KI = 0;
            double KD = 0;
            double probational=0 ;
            double derivative=0 ;
            double integral=0 ;

            int left_back_motor_target_ticks = left_back_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);
            int left_front_motor_target_ticks = left_front_motor.getCurrentPosition() - helper_class_object.cm_to_ticks(distance);
            int right_back_motor_target_ticks = right_back_motor.getCurrentPosition() -helper_class_object.cm_to_ticks(distance);
            int right__front_motor_target_ticks = right_front_motor.getCurrentPosition() + helper_class_object.cm_to_ticks(distance);

            left_back_motor.setTargetPosition(left_back_motor_target_ticks);
            left_front_motor.setTargetPosition(left_front_motor_target_ticks);
            right_back_motor.setTargetPosition(right_back_motor_target_ticks);
            right_front_motor.setTargetPosition(right__front_motor_target_ticks);



            while ((left_back_motor.getCurrentPosition() < left_back_motor_target_ticks) && (left_front_motor.getCurrentPosition() > left_front_motor_target_ticks)
                    && (right_back_motor.getCurrentPosition() > right_back_motor_target_ticks) && (right_front_motor.getCurrentPosition() < right__front_motor_target_ticks))
            {

                imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


                error= gyro_angel - gyro_start;

                probational = error ;
                integral = integral + error ;
                derivative = error - last_error;

                decrese_power = power + ( (probational * KP ) + (integral*KI) + (derivative*KD) ) ;
                increase_power = power - ( (probational * KP ) + (integral*KI) + (derivative*KD) ) ;

                left_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(increase_power),-100,100));
                left_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(power),-100,100));
                right_back_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(power),-100,100));
                right_front_motor.setPower(Range.clip(helper_class_object.dc_motor_power_adapter(decrese_power),-100,100));

                sleep(1);

                last_error = error ;

                telemetry.addData("decrese_power and increase_power","%f %f",decrese_power,increase_power);
                telemetry.update();
            }


            left_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            left_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            right_back_motor.setPower(helper_class_object.dc_motor_power_adapter(0));
            right_front_motor.setPower(helper_class_object.dc_motor_power_adapter(0));

            sleep(1);


        }else if (power ==0){


        }


    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        gyro_angel = Double.parseDouble(helper_class_object.formatAngle(angles.angleUnit, angles.firstAngle));
                        return helper_class_object.formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return helper_class_object.formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return helper_class_object.formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
}
