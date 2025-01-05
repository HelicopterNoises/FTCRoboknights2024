package teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="2025 Test Omni Op Mode", group="Linear OpMode")

public class BasicOmniOpMode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor winch = null;
    private DcMotor sliderail = null;

    private Servo claw = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        claw = hardwareMap.get(Servo.class, "claw");  


        winch = hardwareMap.get(DcMotor.class, "singleMotor");
        sliderail = hardwareMap.get(DcMotor.class, "sliderail");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //sets up the encoders, "zeroes". it
        sliderail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //^

        winch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //needed to turn robot back on
        sliderail.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("StatusPos", pos);
        telemetry.update();
        double speed = 0.6;//EH Slot 0
        double sliderailPower = 0.0;
        double targetposition = 0.0;
        boolean slideLock = false;
        double pos = 0;
        boolean buttonApressed = false;
        int Apress = 0;


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = -gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            double winchPos = winch.getCurrentPosition();  //this gets the current position of the motor from the encoder

            double CPR = 537.7;

            // Get the current angle of the winch
            double winchAngle = -(((winchPos / CPR) * 360) % 360); //no mod so that it can go below 0

            //gets the current distance of the sliderail from initialization point
            double sliderailPos = sliderail.getCurrentPosition();
            double circumference = Math.PI * 1.5; //1.5 is the spool diameter in INCHES
            // Get the current distance from base.
            double sliderailDistance = (sliderailPos / CPR) * circumference;

            boolean encoderBlock = true;

            if (gamepad1.left_bumper) { //when bumper is held down, the encoder block doesn't work. Otherwise it works.
                encoderBlock = false;
            } else {
                encoderBlock = true;
            }

            if ((gamepad1.a) & ((winchPos >= -2000) || (encoderBlock = false))) { //UP IS NEGATIVE ON THE WINCH
                winch.setPower(-speed);
            } else if (gamepad1.b) {
                winch.setPower(speed);
            } else {
                winch.setPower(0);
            }

            //The following code is Sliderail controls with encoder blockers and SlideLock, which ensures the sliderail doesn't fall when extended, even at high angles
            //Slidelock is not good for the motor so it is minimized as much as possible
            if (gamepad1.dpad_up & ((sliderailPos >= -2100) || (encoderBlock = false))) { //Raises sliderail, but does not allow overextension IF enabled (disabled via left bumper)
                sliderail.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                sliderailPower = -0.5;
                targetposition = sliderail.getCurrentPosition(); //This creates a variable so IF the sliderail falls, there will be a mismatch, and the sliderail will move back and hold its position
                slideLock = true;
            }
            else if (gamepad1.dpad_down) {
                sliderail.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                sliderailPower = 0.3;
                targetposition = sliderail.getCurrentPosition();
            }
            else if (gamepad1.dpad_right){ //press to manually disable slidelock after each extension if not needed to preserve motor
                slideLock = false;
                targetposition = sliderail.getCurrentPosition();
                sliderailPower = 0;
            }
            else if (slideLock == true) { //WHEN SLIDELOCK ENABLED, THIS BLOCK HOLDS THE SLIDERAIL AT ITS POSITION
                sliderailPower = 0;
                sliderail.setTargetPosition((int)targetposition); //to make motor run to location, set target position
                sliderail.setMode(DcMotor.RunMode.RUN_TO_POSITION);//then tells it to move to the location it was originally extended to
                sliderailPower = 0.25; //this specifies maximum power it can run at to correct
            }
            else { //If all conditions is false, don't run. This is probably not neccesary
                sliderailPower = 0;
                slideLock = false;
            }
            sliderail.setPower(sliderailPower);

            if (sliderailPos <= -2100) { //telemetry if sliderail is overextended
                telemetry.addData("you're going too far friendo (disable with left bumper) ", sliderailPos);
            }
            else {
                telemetry.addData("EncoderBlock (disable w/ left bumper): ", encoderBlock);
            }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++            Start of Claw                               ++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
    if (gamepad1.right_bumper) {
        if (buttonApressed == false) {
            Apress = Apress + 1;
        if((Apress%2)== 0){
            pos = 0.0;
            claw.setPosition(pos);
        }
        else {
            pos = 0.038;
            claw.setPosition(pos);
        }
            buttonApressed = true;
        }
    }
    else{
        buttonApressed = false;
    }
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++           end of Claw                                  ++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Slidelock (disable w/ Right Dpad): ", slideLock);
            telemetry.addData("Sliderail distance: ", sliderailDistance);
            telemetry.update();


        }
    }
}
