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

@Autonomous(name = "RedSideOpenCv", group = "Concept")
public class RedSideOpenCv extends LinearOpMode {
    private AutoLib autoLib;
    // Description: Starts on blue crater latcher

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        float fullPower = 1f;
        float fastPower = .8f;
        float mediumPower = .7f; //.6f
        float slowPower = 0.3f;
        float verySlowPower = 0.1f;
        double distance = 0;
        float armDistance = 12f;
        float latchingDistance = 3f;
        float foundationDistance = 5f;  //2
        float defaultMaxDistance = 15f;
        int moveToStoneDistance = 60;
        int backAndForthStoneDistance = 7; //5
        int stoneLength = 20;
        int initialMoveDistance = 43;   //33
        int stoneToFoundationDistance = 195;    //190
        int secondStoneDistance = 50;
        int foundationAdjustedDistance = 5;
        int diagonalDistance = 200;

        // Vuforia
        telemetry.addData("pos", autoLib.getPipeline().getDetectedPosition());
        telemetry.update();
        autoLib.autonArmDown();
//        autoLib.calcMove(45, mediumPower, Constants.Direction.RIGHT);
//        Constants.Coordinates coordinates = autoLib.readCoordinates();
//        autoLib.getPipeline().getDetectedPosition();
//        distance = autoLib.getDistanceCM();


        if (autoLib.getPipeline().getDetectedPosition() == 2) {
            telemetry.addData("pos", "Left");
            telemetry.update();
            autoLib.calcMove(moveToStoneDistance, slowPower, Constants.Direction.RIGHT);
            autoLib.calcMove(stoneLength * 2, slowPower, Constants.Direction.FORWARD);
            Thread.sleep(300);  //400
            autoLib.autonGrab();
            Thread.sleep(500);  //500
            autoLib.autonArmUp();
//            Thread.sleep(400);
//            autoLib.calcTurn(3, slowPower);
            autoLib.calcMove(backAndForthStoneDistance, slowPower, Constants.Direction.LEFT);    // move back little
            autoLib.rampMove(stoneToFoundationDistance + (2 * stoneLength), slowPower, Constants.Direction.BACKWARD, true);  // move forward towards foundation
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
            Thread.sleep(200);
            autoLib.autonArmDown();
            Thread.sleep(200);
            autoLib.autonScore();
            autoLib.autonArmUp();
            autoLib.autonGrab();
            autoLib.calcTurn(74, fullPower);
            distance = autoLib.getFoundationDistance();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before latching 1-a", distance);
            telemetry.update();
            autoLib.calcMove((float) (distance + foundationDistance), verySlowPower, Constants.Direction.BACKWARD);
            telemetry.addData("Distance at foundation before latching 1-b", autoLib.getFoundationDistance());
            telemetry.update();
            //Latch while moving 5 cm
            autoLib.calcMove(10, verySlowPower, Constants.Direction.BACKWARD);
            autoLib.latchServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(75, fastPower, Constants.Direction.FORWARD);  // move closer to foundation
            autoLib.calcTurn(-140, fullPower);
//            autoLib.calcMove(15, 1f, Constants.Direction.BACKWARD);
            Thread.sleep(200);
            autoLib.restServoFoundation();
            Thread.sleep(100);
            autoLib.calcMove(5, fullPower, Constants.Direction.RIGHT);
            autoLib.calcMove(95, fastPower, Constants.Direction.FORWARD);

        } else if (autoLib.getPipeline().getDetectedPosition() == 1) {
            telemetry.addData("pos", "Center");
            telemetry.update();

            autoLib.calcMove(moveToStoneDistance, slowPower, Constants.Direction.RIGHT);
            autoLib.calcMove(stoneLength, slowPower, Constants.Direction.FORWARD);
            Thread.sleep(300);  //400
            autoLib.autonGrab();
            Thread.sleep(500);  //500
            autoLib.autonArmUp();
//            Thread.sleep(400);
//            autoLib.calcTurn(3, slowPower);
            autoLib.calcMove(backAndForthStoneDistance, slowPower, Constants.Direction.LEFT);    // move back little
            autoLib.calcMove(stoneToFoundationDistance + stoneLength, fullPower, Constants.Direction.BACKWARD);
//            autoLib.calcMove(backAndForthStoneDistance + foundationAdjustedDistance, slowPower, Constants.Direction.RIGHT);
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
            Thread.sleep(200);
            autoLib.autonArmDown();
            Thread.sleep(200);
            autoLib.autonScore();
            autoLib.autonArmUp();
            autoLib.autonGrab();
            autoLib.calcTurn(74, fullPower);
            distance = autoLib.getFoundationDistance();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before latching 1-c", distance);
            telemetry.update();
            autoLib.calcMove((float) (distance + foundationDistance), verySlowPower, Constants.Direction.BACKWARD);
            telemetry.addData("Distance at foundation before latching 1-d", autoLib.getFoundationDistance());
            telemetry.update();
            //Latch while moving 5 cm
            autoLib.calcMove(10, verySlowPower, Constants.Direction.BACKWARD);
            autoLib.latchServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(75, fastPower, Constants.Direction.FORWARD);  // move closer to foundation
            autoLib.calcTurn(-140, fullPower);
//            autoLib.calcMove(15, 1f, Constants.Direction.BACKWARD);
            Thread.sleep(200);
            autoLib.restServoFoundation();
            Thread.sleep(100);
//            autoLib.calcMove(5, fullPower, Constants.Direction.RIGHT);
            autoLib.calcMove(95, fastPower, Constants.Direction.FORWARD);

        } else if (autoLib.getPipeline().getDetectedPosition() == 0) {
            telemetry.addData("pos", "Right");
            telemetry.update();
            autoLib.calcMove(moveToStoneDistance, slowPower, Constants.Direction.RIGHT);
//            autoLib.calcMove(stoneLength, slowPower, Constants.Direction.BACKWARD);
            Thread.sleep(300);  //400
            autoLib.autonGrab();
            Thread.sleep(500);  //500
            autoLib.autonArmUp();
//            Thread.sleep(400);
//            autoLib.calcTurn(3, slowPower);
            autoLib.calcMove(backAndForthStoneDistance, slowPower, Constants.Direction.LEFT);    // move back little
            autoLib.calcMove(stoneToFoundationDistance, fullPower, Constants.Direction.BACKWARD);
//            autoLib.calcMove(backAndForthStoneDistance + foundationAdjustedDistance, slowPower, Constants.Direction.RIGHT);
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
            Thread.sleep(200);
            autoLib.autonArmDown();
            Thread.sleep(200);
            autoLib.autonScore();
            autoLib.autonArmUp();
            autoLib.autonGrab();
            autoLib.calcTurn(74, fullPower);
            distance = autoLib.getFoundationDistance();
            if (distance > defaultMaxDistance) {
                distance = defaultMaxDistance;
            }
            telemetry.addData("Distance at foundation before latching 1-c", distance);
            telemetry.update();
            autoLib.calcMove((float) (distance + foundationDistance), verySlowPower, Constants.Direction.BACKWARD);
            telemetry.addData("Distance at foundation before latching 1-d", autoLib.getFoundationDistance());
            telemetry.update();
            //Latch while moving 5 cm
            autoLib.calcMove(10, verySlowPower, Constants.Direction.BACKWARD);
            autoLib.latchServoFoundation();
            Thread.sleep(300);
            autoLib.calcMove(75, fastPower, Constants.Direction.FORWARD);  // move closer to foundation
            autoLib.calcTurn(-140, fullPower);
//            autoLib.calcMove(15, 1f, Constants.Direction.BACKWARD);
            Thread.sleep(200);
            autoLib.restServoFoundation();
            Thread.sleep(100);
            autoLib.calcMove(5, fullPower, Constants.Direction.RIGHT);
            autoLib.calcMove(95, fastPower, Constants.Direction.FORWARD);
        }
        telemetry.update();
    }

    private void initialize() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

//        autoLib = new AutoLib(this, new Point[]{new Point(194, 184), new Point(275, 160), new Point(194, 353), new Point(275, 331)});
        autoLib = new AutoLib(this, new Point[]{new Point(217, 426), new Point(299, 454), new Point(217, 260), new Point(299, 289)});

        autoLib.restServoFoundation();
        autoLib.autonScore();
        autoLib.grabCapstone();
        autoLib.scoringSlideRest();

        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
