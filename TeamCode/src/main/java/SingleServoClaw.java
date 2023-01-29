import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class SingleServoClaw extends LinearOpMode {
    public int SLIDER_SPEED = 10;
    public int currentPosition = 0;
    public boolean buttonY;
    public boolean buttonA;
    public boolean buttonX;
    public boolean buttonB;//low junction
    public double armDown;
    public double armUp;
    public double defaultSpeed = 0.13;
    public robotInit robot = new robotInit();
    ElapsedTime runtime = new ElapsedTime();


        //    public class releaseLeftClaw implements Runnable {
//        public void run(){
////
//        }
//    }
//    public class clampLeftClaw implements Runnable {
//        public void run(){
//
//        }
//    }
//    public class releaseRightClaw implements Runnable {
//        public void run(){
////
//        }
//    }
//    public class startRightClaw implements Runnable {
//        public void run(){
//
//        }
//    }
        public void runOpMode() {
            robot.init(hardwareMap);
            boolean openToggle = false;

            waitForStart();

            while (opModeIsActive()) {
                //VARIABLES INITIALIZATION
                //WHEELS

                buttonY = gamepad1.y;
                buttonA = gamepad1.a;
                buttonX = gamepad1.x;
                buttonB = gamepad1.b;//low junction
                chekButtons();
                telemetry.addData("Left slider:", robot.armLiftLeft.getCurrentPosition());
                telemetry.addData("Right slider:", robot.armLiftRight.getCurrentPosition());
                telemetry.update();


                robot.armLiftRight.setPower(defaultSpeed);
                robot.armLiftLeft.setPower(defaultSpeed);

                double vertical = 0.80 * (-gamepad2.left_stick_y); //move forward, backward
                double horizontal = 0.80 * (gamepad2.left_stick_x); //move left, right
                double turn = 0.80 * (-gamepad2.right_stick_x); //turn left, right


                //SLIDER
                boolean buttonY = gamepad1.y;
                boolean buttonA = gamepad1.a;

                double armDown = (0.5) * (gamepad1.left_trigger); // brings linear slides down
                double armUp = -gamepad1.right_trigger; // brings linear slides up

                //SPINNER
                double turntable = (-gamepad1.right_stick_x); // turning on the turntable
                robot.waiter.setPower(turntable);

                //CLAW
                boolean clamp = gamepad1.right_bumper; // clamps the closer servo
                boolean release = gamepad1.left_bumper; // release the closer servo

                //turning is same, triggers raise/lower, bumpers open/close,


                //SLIDER
                robot.armLiftLeft.setPower(armDown + armUp);
                robot.armLiftRight.setPower(armDown + armUp);
//            telemetry.addData("LEFT: ",robot.armLiftLeft.getCurrentPosition());
//            telemetry.addData("RIGHT: ",robot.armLiftRight.getCurrentPosition());

                telemetry.update();

                //mecnum wheels driving
                robot.motorFL.setPower(vertical + horizontal - turn);
                robot.motorFR.setPower(vertical - horizontal + turn);
                robot.motorBL.setPower(vertical - horizontal - turn);
                robot.motorBR.setPower(vertical + horizontal + turn);


                //clamp and release cone with closer servo PROBLEM WITH SERVOS
                if (clamp) {
                    robot.closerL.setPosition(0); //Rotates clockwise
                    // robot.closerR.setPosition(0.6); //Rotates clockwise
//                robot.closerR.setPosition(0); //Rotates counterclockwise
                    telemetry.addData("CURRENT ACTION:", "clamp pressed");
                    telemetry.update();
                }
                if (release) {
                    robot.closerL.setPosition(0.5); //release cone with closer servo
                    // robot.closerR.setPosition(0); //release cone with closer servo
//                robot.closerR.setPosition(.5); //release cone with closer servo
                    telemetry.addData("CURRENT ACTION", "Release pressed");
                    telemetry.update();
                }
            }
        }


        //MINIT AUTNOMOUS

        //            if (buttonY) {
//               raiseTop();
//            }
//            if (buttonA) {
//                lowerZero();
//            }
        boolean chekButtons() {
            if (buttonY) {
                raise(3);
                return true;
            } else if (buttonA) {
                raise(0);
                return true;
            } else if (buttonB) {
                raise(1);
                return true;
            } else if (buttonX) {
                raise(2);
                return true;
            }
//    }else if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
//        double armDown = -gamepad1.left_trigger; // brings linear slides down
//        double armUp = gamepad1.right_trigger; // brings linear slides up
//        double speed = 0.5* (armDown + armUp);
//
////        robot.armLiftLeft.setPower( speed > 0 ?speed+ defaultSpeed: speed);
////        robot.armLiftRight.setPower(speed > 0 ?speed+ defaultSpeed: speed);
//        robot.armLiftLeft.setPower( speed + defaultSpeed);
//        robot.armLiftRight.setPower(speed + defaultSpeed);
//        return true;
//    }
            return false;
        }

        public void raise(int numberOfJunction) {
            double currentSpeed = currentPosition < numberOfJunction ? Math.abs(SLIDER_SPEED) : Math.abs(SLIDER_SPEED) * 0.3;
            int[] positionSet;


            int LOW_JUNCTION = 1100 + 135;
            int MEDIUM_JUNCTION = LOW_JUNCTION + 1050 - 50;
            int HIGH_JUNCTION = MEDIUM_JUNCTION + 1050 - 50;

            int GROUND_JUNCTION = 115;


            // allocates memory for 10 integers
            positionSet = new int[]{GROUND_JUNCTION, LOW_JUNCTION, MEDIUM_JUNCTION, HIGH_JUNCTION};


            int newArmLiftTargetRight;
            int newArmLiftTargetLeft;


            // Determine new target position, and pass to motor controller
            newArmLiftTargetRight = positionSet[numberOfJunction];
            newArmLiftTargetLeft = positionSet[numberOfJunction];

            robot.armLiftLeft.setTargetPosition(newArmLiftTargetLeft);
            robot.armLiftRight.setTargetPosition(newArmLiftTargetRight);

            // Turn On RUN_TO_POSITION
            robot.armLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLiftLeft.setPower(currentSpeed);
            robot.armLiftRight.setPower(currentSpeed);

            currentPosition = numberOfJunction;
            while (opModeIsActive() && robot.armLiftLeft.isBusy() && robot.armLiftRight.isBusy()) {
                // Display it for the driver.
                telemetry.addData("CURRENT ACTION: ", "RAISING TO THE TOP");
                telemetry.update();

                if (chekButtons()) break;
            }

        }

    }