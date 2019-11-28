package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp(name = "FtcManual",group = "FTC")

public class FtcManual extends OpMode {
    private DcMotor left_back_motor = null;
    private DcMotor left_front_motor = null;
    private DcMotor right_back_motor = null;
    private DcMotor right_front_motor = null;
    Servo foundation_servo= null;
   // private DcMotor arm_motor = null;
    private TouchSensor touch_sensor ;

    double move_power = 0 ;
    double diagonal_power = 0 ;
    double side_power = 0 ;
    double spin_power = 0 ;
    double arm_power = 0 ;
    double stop_power = 0 ;


    double gyro_start ;
    double gyro_angel ;



    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    HelperClass helper_class_object = new HelperClass();

    @Override
    public void init() {
        left_back_motor = hardwareMap.get(DcMotor.class, "left_back_motor");
        left_front_motor = hardwareMap.get(DcMotor.class, "left_front_motor");
        right_back_motor = hardwareMap.get(DcMotor.class, "right_back_motor");
        right_front_motor = hardwareMap.get(DcMotor.class, "right_front_motor");
       // arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
       touch_sensor = hardwareMap.get(TouchSensor.class, "touch_sensor");
        foundation_servo=hardwareMap.get(Servo.class,"foundation_servo");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();



       // arm_motor.setDirection(DcMotor.Direction.FORWARD);







    }

    @Override
    public void init_loop() {

        telemetry.addData("Status", "init_loop");
        telemetry.addData("current servo pos","%f",foundation_servo.getPosition());
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

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        telemetry.addData("hi" , "%f",gyro_angel);
        telemetry.update();




      //  while (! touch_sensor.isPressed()){
            //helper_class_object.move_arm_without_encoder(arm_motor, arm_power ,'U');

      //  }




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
            helper_class_object.move_diagonal_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,move_power,'R');
        }else if(gamepad1.dpad_up && gamepad1.dpad_left){
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            helper_class_object.move_diagonal_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,move_power,'L');
        }else if(gamepad1.dpad_down &&gamepad1.dpad_right){
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            helper_class_object.move_diagonal_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,-move_power,'L');
        }else if(gamepad1.dpad_down && gamepad1.dpad_left){
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            helper_class_object.move_diagonal_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,-move_power,'R');
        }

        else if (gamepad1.dpad_up) {
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            //move forward
            helper_class_object.move_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,
                    move_power, 'F');
        }

        else if (gamepad1.dpad_down) {
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            //move backward
            helper_class_object.move_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,
                    -move_power, 'B');
        }


        else if (gamepad1.dpad_right) {
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.FORWARD);
            right_front_motor.setDirection(DcMotor.Direction.FORWARD);
            //move right
            helper_class_object.move_side_without_encoder(left_back_motor, right_front_motor,
                    left_front_motor, right_back_motor,
                    side_power, 'R');
        }
        else if (gamepad1.dpad_left) {
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.FORWARD);
            right_front_motor.setDirection(DcMotor.Direction.FORWARD);
            //move left
            helper_class_object.move_side_without_encoder(left_back_motor, right_front_motor,
                    left_front_motor, right_back_motor,
                    side_power, 'L');

        }

        else if (gamepad1.b) {
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            //spin clockwise
            helper_class_object.spin_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor, spin_power, 'C');
        }

        else if (gamepad1.x) {
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            //spin anti-clockwise
            helper_class_object.spin_without_encoder(left_back_motor, left_front_motor,
            right_back_motor, right_front_motor, spin_power, 'A');




        }else{
            left_back_motor.setDirection(DcMotor.Direction.FORWARD);
            left_front_motor.setDirection(DcMotor.Direction.FORWARD);
            right_back_motor.setDirection(DcMotor.Direction.REVERSE);
            right_front_motor.setDirection(DcMotor.Direction.REVERSE);
            helper_class_object.move_without_encoder(left_back_motor, left_front_motor,
                    right_back_motor, right_front_motor,
                    stop_power, 'F');
        }

        if(gamepad1.y) {

            //  helper_class_object.move_arm_without_encoder(arm_motor, arm_power ,'U');
            double target_angel = 90;


            gyro_start = gyro_angel;

            while (gyro_angel < (gyro_start + target_angel)) {
                helper_class_object.spin_without_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 15, 'A');
                imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                telemetry.update();

            }
            for (int j = 0; j < 1000; ++j) {

            }
            while (gyro_angel > (gyro_start + target_angel)) {
                helper_class_object.spin_without_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 15, 'C');
                imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                telemetry.update();

            }


            if (gyro_start >= 0) {

                if (gyro_start + target_angel <= 180) {

                    while (gyro_angel < (gyro_start + target_angel)) {
                        helper_class_object.spin_without_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 15, 'A');
                        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                        telemetry.update();

                    }
                    for (int j = 0; j < 1000; ++j) {

                    }
                    while (gyro_angel > (gyro_start + target_angel)) {
                        helper_class_object.spin_without_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 15, 'C');
                        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                        telemetry.update();

                    }

                } else if (gyro_start + target_angel > 180) {

                    double moven_angels = 180 - gyro_start;
                    double remaining_angels = target_angel - moven_angels;
                    double new_target_angel = -180 + remaining_angels;


                    while (gyro_angel < 180) {
                        helper_class_object.spin_without_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 15, 'A');
                        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                        telemetry.update();

                    }
                    for (int j = 0; j < 1000; ++j) {

                    }
                    while (gyro_angel > 180) {
                        helper_class_object.spin_without_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 15, 'C');
                        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                        telemetry.update();

                    }


                    while (gyro_angel < new_target_angel) {
                        helper_class_object.spin_without_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 15, 'A');
                        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                        telemetry.update();

                    }
                    for (int j = 0; j < 1000; ++j) {

                    }
                    while (gyro_angel > new_target_angel) {
                        helper_class_object.spin_without_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 15, 'C');
                        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                        telemetry.update();


                    }
                } else if (gyro_start < 0) {

                    if (gyro_start + target_angel <= 0) {

                        while (gyro_angel < (gyro_start + target_angel)) {
                            helper_class_object.spin_without_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 15, 'A');
                            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                            telemetry.update();

                        }
                        for (int j = 0; j < 1000; ++j) {

                        }
                        while (gyro_angel > (gyro_start + target_angel)) {
                            helper_class_object.spin_without_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 15, 'C');
                            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                            telemetry.update();

                        }


                    } else if (gyro_start + target_angel > 0) {

                        double moven_angels = -gyro_start;
                        double remaining_angels = target_angel - moven_angels;
                        double new_target_angel = remaining_angels;


                        while (gyro_angel < 0) {
                            helper_class_object.spin_without_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 15, 'A');
                            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                            telemetry.update();

                        }
                        for (int j = 0; j < 1000; ++j) {

                        }
                        while (gyro_angel > 0) {
                            helper_class_object.spin_without_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 15, 'C');
                            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                            telemetry.update();

                        }


                        while (gyro_angel < new_target_angel) {
                            helper_class_object.spin_without_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 15, 'A');
                            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                            telemetry.update();

                        }
                        for (int j = 0; j < 1000; ++j) {

                        }
                        while (gyro_angel > new_target_angel) {
                            helper_class_object.spin_without_encoder(left_back_motor, left_front_motor, right_back_motor, right_front_motor, 15, 'C');
                            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                            telemetry.update();

                        }

                    }
                }


            } else if (gamepad1.a) {
                //  helper_class_object.move_arm_without_encoder(arm_motor, -arm_power ,'D');


            } else {
                //   helper_class_object.move_arm_without_encoder(arm_motor, stop_power ,'U');
            }


            if (gamepad2.dpad_up){
foundation_servo.setPosition(0);             }
            else if (gamepad2.dpad_down){
foundation_servo.setPosition(.75);
             }else {
                telemetry.addData("test servo position is ","%f",foundation_servo.getPosition());
            }


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