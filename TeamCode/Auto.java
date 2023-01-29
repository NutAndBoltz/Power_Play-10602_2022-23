//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//
//@Disabled
//@Autonomous(name="Auto", group="Pushbot")
//public class Auto extends LinearOpMode {
//
//    public robotInit robot = new robotInit();
//    ElapsedTime runtime = new ElapsedTime();
//
//    int location; //for storing location after detection
//
//    OpenCvCamera webcam;
//    DeterminationPipeline pipeline;
//
//    @Override
//    public void runOpMode() {
//
//        robot.init(hardwareMap);
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
//
//        pipeline = new DeterminationPipeline();
//        webcam.setPipeline(pipeline);
//
//        resetEncoder();
//        startEncoderMode();
//
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//
//
//        // Wait for the game to start (driver presses PLAY)
//        telemetry.addLine("Waiting for start");
//        telemetry.update();
//        waitForStart();
//
//
//        // Telemetry for testing ring detection
//        while (opModeIsActive()) {
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.addData("Position", pipeline.position);
//            telemetry.update();
//
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(5000);
//
//            //Step 1: Detect location
//            if (pipeline.position == DeterminationPipeline.SleevePosition.ONE) {
//                telemetry.addData("Detected", "location 1!");
//                telemetry.addData("Analysis", pipeline.getAnalysis());
//                telemetry.update();
//
//                location = 1;
//
//            } else if (pipeline.position == DeterminationPipeline.SleevePosition.TWO) {
//                telemetry.addData("Detected", "location 2!");
//                telemetry.addData("Analysis", pipeline.getAnalysis());
//                telemetry.update();
//
//                location = 2;
//
//            } else if (pipeline.position == DeterminationPipeline.SleevePosition.THREE){
//                telemetry.addData("Detected", "location 3!");
//                telemetry.addData("Analysis", pipeline.getAnalysis());
//                telemetry.update();
//
//                location = 3;
//
//            } else {
//                telemetry.addData("Detected", "Unknown position");
//            }
//            sleep(5000);
//
//            // Step 2: Park
//            if (location == 1) {
//                telemetry.addData("Moving to", "level 1!");
//                telemetry.addData("Analysis", pipeline.getAnalysis());
//                telemetry.update();
//
//                //strafe left
//                moveLeft(30);
//                moveRight(2);
//                //drive forward
//                moveForward(28);
//
////                moveForward(35);
////                raise(10);
////                turnright(15);
////                moveForward(5);
////                placeCone();
////                moveBackward(25);
////                moveRight(5);
//
//
//
//
//            } else if (location == 2) {
//                telemetry.addData("Moving to", "level 2!");
//                telemetry.addData("Analysis", pipeline.getAnalysis());
//                telemetry.update();
//
//                //drive forward
//                moveLeft(30);
//                moveRight(30);
//                moveForward(28);
//
//            } else {
//                telemetry.addData("Moving to", "level 3!");
//                telemetry.addData("Analysis", pipeline.getAnalysis());
//                telemetry.update();
//
//                //strafe left
//                moveLeft(30);
//                moveRight(60);
//                //drive forward
//                moveForward(28);
//
//            }
//
//            // Stop, take a well deserved breather
//            //sleep(1000);     // pause for servos to move
//
//            telemetry.addData("Path", "Complete");
//            telemetry.update();
//            stop();
//
//        }
//    }
//
//
//    /* FUNCTIONS */
//
//
//    /* CUSTOM SLEEVE DETECTION */
//    public static class DeterminationPipeline extends OpenCvPipeline
//    {
//        /*
//         * An enum to define the sleeve position
//         */
//        public enum SleevePosition
//        {
//            ONE,
//            TWO,
//            THREE
//        }
//
//        /*
//         * Some color constants
//         */
//        static final Scalar BLUE = new Scalar(0, 0, 255);
//        static final Scalar GREEN = new Scalar(0, 255, 0);
//
//        /*
//         * The core values which define the location and size of the sample regions
//         */
//        //  static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(85,183);
//        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(180,150);
//
//        static final int REGION_WIDTH = 30;
//        static final int REGION_HEIGHT = 40;
//
//        //Position 3: ALL WHITE Analysis: 136, 105, 139, 143, 137, 130, 145, 147, 155,
//        //Position 2: HALF GREEN Analysis: 111, 115, 105, 109, 111, 108, 124, 99
//        //Position 1: ALL GREEN Analysis: 91, 80, 78, 73, 116, 106,
//
//        final int THREE_POSITION_THRESHOLD = 130;
//        final int TWO_POSITION_THRESHOLD = 107;
//
//        Point region1_pointA = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x,
//                REGION1_TOPLEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//        /*
//         * Working variables
//         */
//        Mat region1_Cb;
//        Mat YCrCb = new Mat();
//        Mat Cb = new Mat();
//        int avg1;
//
//        // Volatile since accessed by OpMode thread w/o synchronization
//        //Create position object and set default value
//        public volatile SleevePosition position = SleevePosition.ONE;
//
//        /*
//         * This function takes the RGB frame, converts to YCrCb,
//         * and extracts the Cb channel to the 'Cb' variable
//         */
//        void inputToCb(Mat input)
//        {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(YCrCb, Cb, 0);
//        }
//
//        @Override
//        public void init(Mat firstFrame)
//        {
//            inputToCb(firstFrame);
//
//            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
//        }
//
//        @Override
//        public Mat processFrame(Mat input)
//        {
//            inputToCb(input);
//
//            avg1 = (int) Core.mean(region1_Cb).val[0];
//
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region1_pointA, // First point which defines the rectangle
//                    region1_pointB, // Second point which defines the rectangle
//                    BLUE, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines
//
//            // Record our analysis
//            if(avg1 > THREE_POSITION_THRESHOLD){
//                position = SleevePosition.THREE;
//            }else if (avg1 > TWO_POSITION_THRESHOLD){
//                position = SleevePosition.TWO;
//            }else{
//                position = SleevePosition.ONE;
//            }
//
//            return input;
//        }
//
//        public int getAnalysis()
//        {
//            return avg1;
//        }
//    }
//
//    /* ENCODER FUNCTIONS */
//    public void resetEncoder()
//    {
//        robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.armLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.armLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
//    public void startEncoderMode()
//    {
//        //Set Encoder Mode
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.armLiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.armLiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    /* MOVEMENT FUNCTIONS */
//    public void moveForward(int inches) {
//        int newmotorFLTarget;
//        int newmotorFRTarget;
//        int newmotorBLTarget;
//        int newmotorBRTarget;
//
//        // Determine new target position, and pass to motor controller
//        newmotorFLTarget = robot.motorFL.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
//        newmotorFRTarget = robot.motorFR.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
//        newmotorBLTarget = robot.motorBL.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
//        newmotorBRTarget = robot.motorBR.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
//        robot.motorFL.setTargetPosition(newmotorFLTarget);
//        robot.motorFR.setTargetPosition(newmotorFRTarget);
//        robot.motorBL.setTargetPosition(newmotorBLTarget);
//        robot.motorBR.setTargetPosition(newmotorBRTarget);
//
//        // Turn On RUN_TO_POSITION
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
//        runtime.reset();
//        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget );
//            telemetry.update();
//        }
//
//        // Stop all motion;
//        stopRobot();
//
//        // Turn off RUN_TO_POSITION
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//
//    public void moveBackward(double inches) {
//        int newmotorFLTarget;
//        int newmotorFRTarget;
//        int newmotorBLTarget;
//        int newmotorBRTarget;
//
//        // Determine new target position, and pass to motor controller
//        newmotorFLTarget = robot.motorFL.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
//        newmotorFRTarget = robot.motorFR.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
//        newmotorBLTarget = robot.motorBL.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
//        newmotorBRTarget = robot.motorBR.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
//        robot.motorFL.setTargetPosition(newmotorFLTarget);
//        robot.motorFR.setTargetPosition(newmotorFRTarget);
//        robot.motorBL.setTargetPosition(newmotorBLTarget);
//        robot.motorBR.setTargetPosition(newmotorBRTarget);
//
//        // Turn On RUN_TO_POSITION
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
//
//        runtime.reset();
//        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget );
//            telemetry.update();
//        }
//
//        // Stop all motion;
//        stopRobot();
//
//        // Turn off RUN_TO_POSITION
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }
//
//
//    public void moveRight(double inches) {
//        int newmotorFLTarget;
//        int newmotorFRTarget;
//        int newmotorBLTarget;
//        int newmotorBRTarget;
//
//        // Determine new target position, and pass to motor controller
//        newmotorFLTarget = robot.motorFL.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
//        newmotorFRTarget = robot.motorFR.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
//        newmotorBLTarget = robot.motorBL.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
//        newmotorBRTarget = robot.motorBR.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
//        robot.motorFL.setTargetPosition(newmotorFLTarget);
//        robot.motorFR.setTargetPosition(newmotorFRTarget);
//        robot.motorBL.setTargetPosition(newmotorBLTarget);
//        robot.motorBR.setTargetPosition(newmotorBRTarget);
//
//        // Turn On RUN_TO_POSITION
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
//
//        runtime.reset();
//        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget );
//            telemetry.update();
//        }
//
//        // Stop all motion;
//        stopRobot();
//
//        // Turn off RUN_TO_POSITION
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }
//
//    public void moveLeft(double inches) {
//        int newmotorFLTarget;
//        int newmotorFRTarget;
//        int newmotorBLTarget;
//        int newmotorBRTarget;
//
//        // Determine new target position, and pass to motor controller
//        newmotorFLTarget = robot.motorFL.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
//        newmotorFRTarget = robot.motorFR.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
//        newmotorBLTarget = robot.motorBL.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
//        newmotorBRTarget = robot.motorBR.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
//        robot.motorFL.setTargetPosition(newmotorFLTarget);
//        robot.motorFR.setTargetPosition(newmotorFRTarget);
//        robot.motorBL.setTargetPosition(newmotorBLTarget);
//        robot.motorBR.setTargetPosition(newmotorBRTarget);
//
//        // Turn On RUN_TO_POSITION
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
//
//        runtime.reset();
//        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget );
//            telemetry.update();
//        }
//
//        // Stop all motion;
//        stopRobot();
//
//        // Turn off RUN_TO_POSITION
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//
//    public void turnleft(double inches) {
//        int newmotorFLTarget;
//        int newmotorFRTarget;
//        int newmotorBLTarget;
//        int newmotorBRTarget;
//
//        // Determine new target position, and pass to motor controller
//        newmotorFLTarget = robot.motorFL.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);      newmotorFRTarget = robot.motorFR.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
//        newmotorBLTarget = robot.motorBL.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);       newmotorBRTarget = robot.motorBR.getCurrentPosition() - (int)(inches * robot.COUNTS_PER_INCH);
//
//        robot.motorFL.setTargetPosition(newmotorFLTarget);
//        robot.motorFR.setTargetPosition(newmotorFRTarget);
//        robot.motorBL.setTargetPosition(newmotorBLTarget);
//        robot.motorBR.setTargetPosition(newmotorBRTarget);
//
//        // Turn On RUN_TO_POSITION
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
//        runtime.reset();
//        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget );
//            telemetry.update();
//        }
//
//        // Stop all motion;
//        stopRobot();
//
//        // Turn off RUN_TO_POSITION
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public void turnright(double inches) {
//        int newmotorFLTarget;
//        int newmotorFRTarget;
//        int newmotorBLTarget;
//        int newmotorBRTarget;
//
//        // Determine new target position, and pass to motor controller
//        newmotorFLTarget = robot.motorFL.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);      newmotorFRTarget = robot.motorFR.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
//        newmotorBLTarget = robot.motorBL.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);      newmotorBRTarget = robot.motorBR.getCurrentPosition() + (int)(inches * robot.COUNTS_PER_INCH);
//        robot.motorFL.setTargetPosition(newmotorFLTarget);
//        robot.motorFR.setTargetPosition(newmotorFRTarget);
//        robot.motorBL.setTargetPosition(newmotorBLTarget);
//        robot.motorBR.setTargetPosition(newmotorBRTarget);
//
//        // Turn On RUN_TO_POSITION
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.motorFL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorFR.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBL.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.motorBR.setPower(Math.abs(robot.DRIVE_SPEED));
//        runtime.reset();
//        while (opModeIsActive() && (robot.motorFL.isBusy() || robot.motorFR.isBusy() || robot.motorBL.isBusy() || robot.motorBR.isBusy())) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d :%7d", newmotorFLTarget, newmotorFRTarget );
//            telemetry.update();
//        }
//
//        // Stop all motion;
//        stopRobot();
//
//        // Turn off RUN_TO_POSITION
//        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    //RAISE ARM FUNCTION
//    public void raise(double count) {
//
//        int newArmLiftLeftTarget;
//        int newArmLiftRightTarget;
//
//        // Determine new target position, and pass to motor controller
//        newArmLiftLeftTarget = robot.armLiftLeft.getCurrentPosition() + (int) (count);
//        newArmLiftRightTarget = robot.armLiftRight.getCurrentPosition() + (int) (count);
//        robot.armLiftLeft.setTargetPosition(newArmLiftLeftTarget);
//        robot.armLiftRight.setTargetPosition(newArmLiftRightTarget);
//
//        // Turn On RUN_TO_POSITION
//        robot.armLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.armLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.armLiftLeft.setPower(Math.abs(robot.DRIVE_SPEED));
//        robot.armLiftRight.setPower(Math.abs(robot.DRIVE_SPEED));
//
//    }
//
//    //LOWER ARM FUNCTION
//        public void lower(double count) {
//
//            int newArmLiftRightTarget;
//            int newArmLiftLeftTarget;
//
//            // Determine new target position, and pass to motor controller
//            newArmLiftRightTarget = robot.armLiftRight.getCurrentPosition() - (int) (count);
//            newArmLiftLeftTarget = robot.armLiftLeft.getCurrentPosition() - (int) (count);
//            robot.armLiftRight.setTargetPosition(newArmLiftRightTarget);
//            robot.armLiftLeft.setTargetPosition(newArmLiftLeftTarget);
//
//            // Turn On RUN_TO_POSITION
//            robot.armLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.armLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            robot.armLiftRight.setPower(Math.abs(robot.DRIVE_SPEED));
//            robot.armLiftLeft.setPower(Math.abs(robot.DRIVE_SPEED));
//
//        }
//
//    public void stopRobot() {
//        robot.motorFL.setPower(0);
//        robot.motorFR.setPower(0);
//        robot.motorBL.setPower(0);
//        robot.motorBR.setPower(0);
//    }
//
//
//    public void placeCone(){
//            //move motor down
//            robot.armLiftLeft.setPower(0.2);
//            robot.armLiftRight.setPower(0.2);
//            runtime.reset();
//            while (runtime.seconds() < 0.6){
//            }
//
//            //unclamp servo
//            robot.armLiftLeft.setPower(0);
//            robot.armLiftRight.setPower(0);
//            robot.closerL.setPosition(0.5);
//            robot.closerR.setPosition(0);
//
//
//            //wait
//            runtime.reset();
//            while (runtime.seconds() < 1){
//            }
//
//            //move arm back down
//            robot.armLiftRight.setPower(-0.2);
//            robot.armLiftLeft.setPower(-0.2);
//            runtime.reset();
//            while (runtime.seconds() < 0.5){
//            }
//
//            robot.armLiftLeft.setPower(0);
//            robot.armLiftRight.setPower(0);
//        }
//
//
//    public void stopRobot(int seconds) {
//        //delay
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < seconds)) {
//            stopRobot();
//            telemetry.addData("Motor", "Stopped");    //
//            telemetry.update();
//        }
//    }
//}
