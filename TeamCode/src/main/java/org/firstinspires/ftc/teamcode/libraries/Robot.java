package org.firstinspires.ftc.teamcode.libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_ARM_LEFT;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_ARM_RIGHT;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_BACK_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_LEFT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_FRONT_RIGHT_WHEEL;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_LEFT_INTAKE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.MOTOR_RIGHT_INTAKE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_AUTONOMOUS_ARM;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_AUTONOMOUS_GRABBER;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION1;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_FOUNDATION2;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_GRABBER;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_INTAKE;
import static org.firstinspires.ftc.teamcode.libraries.Constants.SERVO_SCORING_ARM;

/*
 * Title: Robot
 * Date Created: 10/14/2018
 * Date Modified: 2/27/2019
 * Author: Poorvi, Sachin
 * Type: Library
 * Description: This is the base library for any main op to be based off. It will contain all the
 *              motors, servos, and sensors.
 */

public class Robot {

    private LinearOpMode opMode;

    // Motors
    private DcMotor[] dcMotors = new DcMotor[8];

    //Servos
    private Servo[] servos = new Servo[7];

    // Sensors
//    private Rev2mDistanceSensor frontDistanceSensor;
//    private RevTouchSensor[] touchSensors = new RevTouchSensor[2];

    Robot(LinearOpMode opMode) {
        this.opMode = opMode;

        initDcMotors();
        initServos();
        initSensors();

    }

    private void initDcMotors() {
        //Naming our motors
        dcMotors[MOTOR_FRONT_LEFT_WHEEL] = opMode.hardwareMap.get(DcMotor.class, "frontLeftWheel");
        dcMotors[MOTOR_FRONT_RIGHT_WHEEL] = opMode.hardwareMap.get(DcMotor.class, "frontRightWheel");
        dcMotors[MOTOR_BACK_LEFT_WHEEL] = opMode.hardwareMap.get(DcMotor.class, "backLeftWheel");
        dcMotors[MOTOR_BACK_RIGHT_WHEEL] = opMode.hardwareMap.get(DcMotor.class, "backRightWheel");
        dcMotors[MOTOR_RIGHT_INTAKE] = opMode.hardwareMap.get(DcMotor.class, "rightIntake");
        dcMotors[MOTOR_LEFT_INTAKE] = opMode.hardwareMap.get(DcMotor.class, "leftIntake");
        dcMotors[MOTOR_ARM_LEFT] = opMode.hardwareMap.get(DcMotor.class, "motorArmLeft");
        dcMotors[MOTOR_ARM_RIGHT] = opMode.hardwareMap.get(DcMotor.class, "motorArmRight");


        dcMotors[MOTOR_FRONT_RIGHT_WHEEL].setDirection(DcMotorSimple.Direction.REVERSE);
        dcMotors[MOTOR_BACK_RIGHT_WHEEL].setDirection(DcMotorSimple.Direction.REVERSE);
    }


    private void initServos() {
        servos[SERVO_FOUNDATION1] = opMode.hardwareMap.get(Servo.class, "servoFoundation1");
        servos[SERVO_FOUNDATION2] = opMode.hardwareMap.get(Servo.class, "servoFoundation2");
        servos[SERVO_AUTONOMOUS_ARM] = opMode.hardwareMap.get(Servo.class, "servoAutonomousArm");
        servos[SERVO_AUTONOMOUS_GRABBER] = opMode.hardwareMap.get(Servo.class, "servoAutonomousGrabber");
        servos[SERVO_GRABBER] = opMode.hardwareMap.get(Servo.class, "servoGrabber");
        servos[SERVO_SCORING_ARM] = opMode.hardwareMap.get(Servo.class, "servoScoringArm");
        servos[SERVO_INTAKE] = opMode.hardwareMap.get(Servo.class, "servoIntake");

    }

    private void initSensors() {
//        frontDistanceSensor = opMode.hardwareMap.get(Rev2mDistanceSensor.class, "frontDistanceSensor");
//        touchSensors[TOUCH_ARM_TOP] = opMode.hardwareMap.get(RevTouchSensor.class, "touchArmTop");
//        touchSensors[TOUCH_ARM_BOTTOM] = opMode.hardwareMap.get(RevTouchSensor.class, "touchArmBottom");
    }

    // Motor methods
    void setDcMotorPower(int index, float power) {
        dcMotors[index].setPower(power);
    }

    void setDcMotorMode(int index, DcMotor.RunMode runMode) {
        dcMotors[index].setMode(runMode);
    }

    int getDcMotorPosition(int index) {
        return dcMotors[index].getCurrentPosition();
    }

    void setDcMotorTargetPosition(int index, int targetPosition) {
        dcMotors[index].setTargetPosition(targetPosition);
    }

    boolean isMotorBusy(int index) {
        return dcMotors[index].isBusy();
    }

    // Servo methods
    void setServoPosition(int index, float position) {
        servos[index].setPosition(position);
    }


    void setDeltaServoPosition(int index, float delta) {
        servos[index].setPosition(
                // This makes sure the servo positions are between 0 and 1
                Range.clip(servos[index].getPosition() + delta, 0, 1));
    }

    float getServoPosition(int index) {
        return (float) servos[index].getPosition();
    }
//    boolean isTouchSensorPressed(int index) {
//        return touchSensors[index].isPressed();
//    }

//    double getWallDistanceCenti() {
//        return (frontDistanceSensor.getDistance(DistanceUnit.METER) * 100);
//    }
}