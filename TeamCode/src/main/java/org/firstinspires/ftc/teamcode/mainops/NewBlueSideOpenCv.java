package org.firstinspires.ftc.teamcode.mainops;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libraries.AutoLib;
import org.firstinspires.ftc.teamcode.libraries.Constants;
import org.opencv.core.Point;


/*
 * Title: AutoBlueCraterBase
 * Date Created: 11/23/2018
 * Date Modified: 2/22/2019
 * Author: Rahul, Poorvi, Varnika
 * Type: Main
 */

@Autonomous(name = "NewBlueSideOpenCv", group = "Concept")
public class NewBlueSideOpenCv extends LinearOpMode {
    private AutoLib autoLib;
    // Description: Starts on blue crater latcher

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        float fullPower = 1f;
        float fastPower = 0.7f;
        float mediumPower = 0.6f;
        float slowPower = 0.2f;
        float verySlowPower = 0.05f;
        double distance = 0;
        float armDistance = 12f;
        float latchingDistance = 3f;
        float foundationDistance = 5f;  //2
        float defaultMaxDistance = 15f;

        // Vuforia
        autoLib.autonArmDown();
        autoLib.calcMove(45, mediumPower, Constants.Direction.RIGHT);
//        Constants.Coordinates coordinates = autoLib.readCoordinates();
//        autoLib.getPipeline().getDetectedPosition();
        distance = autoLib.getDistanceCM();
        if (autoLib.getPipeline().getDetectedPosition() == 1) {
            telemetry.addData("pos", "Left");
            telemetry.update();

            Thread.sleep(200);  //400
            autoLib.autonGrab();
            Thread.sleep(200);  //500
            autoLib.autonArmUp();
//            Thread.sleep(400);
//            autoLib.calcTurn(3, slowPower);
            autoLib.calcMove(10, mediumPower, Constants.Direction.LEFT);    // move back little
//            autoLib.calcTurn(5, slowPower);    // turn, so that the robot will go straight
            autoLib.rampMove(185, fullPower, Constants.Direction.FORWARD, true);  // move forward towards foundation

            distance = autoLib.getDistanceCM();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before placing stone 1-a", distance);
            telemetry.update();
            if (distance > foundationDistance) {
                autoLib.calcMove((float) (distance - foundationDistance), slowPower, Constants.Direction.RIGHT);
            }
            telemetry.addData("Distance at foundation before placing stone 1-b", autoLib.getDistanceCM());
            telemetry.update();

            Thread.sleep(400);
            autoLib.autonArmDown();
            Thread.sleep(400);
            autoLib.autonScore();
//            autoLib.calcTurn(6, slowPower);
            autoLib.autonArmUp();
            autoLib.calcTurn(74, slowPower);    // turn, so that foundation grabbers can be used


            distance = autoLib.getFoundationDistance();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before latching 1-c", distance);
            telemetry.update();
            autoLib.calcMove((float) (distance), verySlowPower, Constants.Direction.BACKWARD);
            telemetry.addData("Distance at foundation before latching 1-d", autoLib.getFoundationDistance());
            telemetry.update();
            //Latch while moving 2 cm
            autoLib.calcMove(2, verySlowPower, Constants.Direction.BACKWARD);
            autoLib.latchServoFoundation();
            Thread.sleep(300);
//            autoLib.calcTurn(25, fastPower);
//            autoLib.calcTurn(30, 1f);
            autoLib.calcMove(85, fastPower, Constants.Direction.FORWARD);  // move closer to foundation
            autoLib.calcTurn(190, 1f);
            autoLib.autonGrab();
//            autoLib.calcMove(15, 1f, Constants.Direction.BACKWARD);
            Thread.sleep(300);
            autoLib.restServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(80, fastPower, Constants.Direction.FORWARD);

        } else if (autoLib.getPipeline().getDetectedPosition() == 0) {


            //-------------------------1st--------------------------------
//            autoLib.calcMove((float) (coordinates.yPosition / 10 - 3), mediumPower, Constants.Direction.FORWARD); //when decreased- moves to the left
            Thread.sleep(200);  //400
            autoLib.autonGrab();
            Thread.sleep(200);  //500
            autoLib.autonArmUp();
//            Thread.sleep(400);
            autoLib.calcTurn(3, fullPower);
            autoLib.calcMove(5, fullPower, Constants.Direction.LEFT);    // move back little
//            autoLib.calcTurn(5, slowPower);    // turn, so that the robot will go straight
             autoLib.rampMove(205, fullPower, Constants.Direction.FORWARD, true);  // move forward towards foundation
//            autoLib.calcMove(205, fullPower, Constants.Direction.FORWARD);  // move forward towards foundation

            distance = autoLib.getDistanceCM();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before placing stone 1-a", distance);
            telemetry.update();
            if (distance > foundationDistance) {
                autoLib.calcMove((float) (distance - foundationDistance), fullPower, Constants.Direction.RIGHT);     //distance - 9
            }
            telemetry.addData("Distance at foundation before placing stone 1-b", autoLib.getDistanceCM());
            telemetry.update();

            Thread.sleep(400);
            autoLib.autonArmDown();
            Thread.sleep(400);
            autoLib.autonScore();
            Thread.sleep(200);
//            autoLib.calcTurn(6, slowPower);
            autoLib.autonArmUp();
            autoLib.autonGrab();

            //--------------------------------------2nd ------------------------------------------------

            autoLib.calcMove(10, fullPower, Constants.Direction.LEFT);
            autoLib.calcMove(265, fullPower, Constants.Direction.BACKWARD);  // move forward towards foundation
            autoLib.calcMove(5, 1f, Constants.Direction.RIGHT);
            autoLib.autonScore();
            autoLib.autonArmDown();
            Thread.sleep(300);
            autoLib.autonGrab();
            Thread.sleep(300);
            autoLib.autonArmUp();
            autoLib.calcMove(200, 1f, Constants.Direction.FORWARD);
            Thread.sleep(400);
            autoLib.autonArmDown();
            Thread.sleep(400);
            autoLib.autonScore();
            Thread.sleep(200);
//            autoLib.calcTurn(6, slowPower);
            autoLib.autonArmUp();
            autoLib.autonGrab();

            //-----------------------------------3rd---------------------------------------------------

            autoLib.calcMove(10, fullPower, Constants.Direction.LEFT);
            autoLib.calcMove(200, fullPower, Constants.Direction.BACKWARD);  // move forward towards foundation
            autoLib.calcMove(5, 1f, Constants.Direction.RIGHT);
            autoLib.autonScore();
            autoLib.autonArmDown();
            Thread.sleep(300);
            autoLib.autonGrab();
            Thread.sleep(300);
            autoLib.autonArmUp();


            distance = autoLib.getDistanceCM();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before placing stone 1-a", distance);
            telemetry.update();
            if (distance > foundationDistance) {
                autoLib.calcMove((float) (distance - foundationDistance), fullPower, Constants.Direction.RIGHT);     //distance - 9
            }
            telemetry.addData("Distance at foundation before placing stone 1-b", autoLib.getDistanceCM());
            telemetry.update();

//            autoLib.calcTurn(72, fullPower);    // turn, so that foundation grabbers can be used
//
//
//            distance = autoLib.getFoundationDistance();
//            if (distance > defaultMaxDistance) {
//                distance = defaultMaxDistance;
//            }
//            telemetry.addData("Distance at foundation before latching 1-c", distance);
//            telemetry.update();
////            if (distance > latchingDistance) {
//            autoLib.calcMove((float) distance, verySlowPower, Constants.Direction.BACKWARD);
////            }
//            telemetry.addData("Distance at foundation before latching 1-d", autoLib.getFoundationDistance());
//            telemetry.update();
//
//            //Latch while moving 2 cm
//            autoLib.calcMove(2, verySlowPower, Constants.Direction.BACKWARD);
//            autoLib.latchServoFoundation();
//            Thread.sleep(300);
////            autoLib.calcTurn(25, fastPower);
////            autoLib.calcTurn(30, 1f);
//            autoLib.calcMove(85, fullPower, Constants.Direction.FORWARD);  // move closer to foundation-70
//            autoLib.calcTurn(190, 1f);
//            autoLib.autonGrab();
////            autoLib.calcMove(15, 1f, Constants.Direction.BACKWARD);
//            Thread.sleep(300);
//            autoLib.restServoFoundation();
//            Thread.sleep(300);
//            autoLib.calcMove(80, fullPower, Constants.Direction.FORWARD);

        } else if (autoLib.getPipeline().getDetectedPosition() == 2) {
            Thread.sleep(400);
            autoLib.autonGrab();
            Thread.sleep(500);
            autoLib.autonArmUp();
            Thread.sleep(400);
            autoLib.calcTurn(3, slowPower);
            autoLib.calcMove(10, mediumPower, Constants.Direction.LEFT);    // move back little
//            autoLib.calcTurn(5, slowPower);    // turn, so that the robot will go straight
            autoLib.calcMove(210, mediumPower, Constants.Direction.FORWARD);  // move forward towards foundation

            distance = autoLib.getDistanceCM();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before placing stone 1-a", distance);
            telemetry.update();
            if (distance > foundationDistance) {
                autoLib.calcMove((float) (distance - foundationDistance), slowPower, Constants.Direction.RIGHT);
            }
            telemetry.addData("Distance at foundation before placing stone 1-b", autoLib.getDistanceCM());
            telemetry.update();

            Thread.sleep(400);
            autoLib.autonArmDown();
            Thread.sleep(400);
            autoLib.autonScore();
//            autoLib.calcTurn(6, slowPower);
            autoLib.autonArmUp();
            autoLib.calcTurn(74, slowPower);    // turn, so that foundation grabbers can be used


            distance = autoLib.getFoundationDistance();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before latching 1-c", distance);
            telemetry.update();
//            if (distance > latchingDistance) {
            autoLib.calcMove((float) (distance), verySlowPower, Constants.Direction.BACKWARD);
//            }
            telemetry.addData("Distance at foundation before latching 1-d", autoLib.getFoundationDistance());
            telemetry.update();

            //Latch while moving 2 cm
            autoLib.calcMove(2, verySlowPower, Constants.Direction.BACKWARD);
            autoLib.latchServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(85, fastPower, Constants.Direction.FORWARD);  // move closer to foundation-70
            autoLib.calcTurn(190, 1f);
            autoLib.autonGrab();
//            autoLib.calcMove(15, 1f, Constants.Direction.BACKWARD);
            Thread.sleep(300);
            autoLib.restServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(80, fastPower, Constants.Direction.FORWARD);
        }
        telemetry.update();
    }

    private void initialize() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        autoLib = new AutoLib(this, new Point[]{new Point(190, 156), new Point(272, 191), new Point(190, 326), new Point(272, 354)});

        autoLib.restServoFoundation();

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
