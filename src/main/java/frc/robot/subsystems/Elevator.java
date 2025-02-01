package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

// TBD: Need to add current limits so we don't burn out motor if we hit top or bottom
// TBD: Need to add PIDF controller
public class Elevator extends SubsystemBase {

    private DigitalInput elevatorBottomSwitch;
    private SparkFlex elevatorMotorLeft;
    // private SparkFlex elevatorMotorRight;

    Encoder enc;
    ArrayList<Integer> elevatorPositions;

    final double ELEVATOR_UP_SPEED = 0.6;
    final double ELEVATOR_DOWN_SPEED = -0.2;
    final double ELEVATOR_BRAKE_SPEED = 0.4;
    boolean bottomState; // state true if bottom limit switch is pushed in

    // TBD: Need to update to use SparkFlex internal encoder
    double encoder;

    // Commmand variables
    // TBD:
    boolean finished = false;
    int direction;

    boolean limitSwitchState = false;
    boolean distanceReached = false;

    // Constants for the elevator stops, CURRENTLY PLACEHOLDERS -- TBD: Update positions for 2025
    int L1HATCH_POS = -10;
    int CARGO_SHIP_POS = -183;
    int L2HATCH_POS = -356;

    public Elevator() {
        // TBD: Need to update bottom switch to match electrical
        elevatorBottomSwitch = new DigitalInput(1);        
        
        
        elevatorMotorLeft = new SparkFlex(Constants.ElevatorConstants.LEFT_CANID, SparkLowLevel.MotorType.kBrushless);
        // TBD: Need to add right motor following
        // elevatorMotorRight = new SparkFlex(Constants.ElevatorConstants.RIGHT_CANID, SparkLowLevel.MotorType.kBrushless);
        // elevatorMotorRight.follow(elevatorMotorLeft);

        // Construct arraylist
        elevatorPositions = new ArrayList<Integer>();
        
        // Construct encoder
        enc = new Encoder(2, 3, false, Encoder.EncodingType.k4X); //default encoder settings, need more info on the one we're using 
        SmartDashboard.putData("encoder", enc);

        // Add elevator switches to smart dashboard
        SmartDashboard.putData("elevatorBottomSwitch", elevatorBottomSwitch);

        // Create array that holds the different stops on the elevator
        elevatorPositions.add(L1HATCH_POS);
        elevatorPositions.add(CARGO_SHIP_POS);
        elevatorPositions.add(L2HATCH_POS);
    }


    @Override
    public void periodic() {
        // TBD: Do we need a top limit switch and do we need an override?
        // if (Robot.oi.manipulatorStick.getRawButton(7) == false) {
            // This code is run every loop
            // if (elevatorMotorLeft.get() > 0 && getTopSwitchState() == true) {
            //     pause();
            //     // System.out.println("Top switch pressed. ABORT ABORT ABORT");
            // }
            if(elevatorMotorLeft.get() <= 0 && getBottomSwitchState() == true) {
                stop();
                enc.reset();
                // System.out.println("Bottom switch pressed. ABORT ABORT ABORT");
            }
        // }
    }

    // Get the current position of the elevator
    public int getCurrentPos(){
        return enc.get();
    }

    // Get any elevator position out of the elevatorPositions array
    public int getElevatorPositions(int index) {
        return elevatorPositions.get(index);
    }

    // Get the direction needed to be travelled by the elevator. 1 is up, -1 is down, 0 is not move
    public int getdirection(int targetPosIndex) {
        System.out.println("Moving from " + getCurrentPos() + " to " + getElevatorPositions(targetPosIndex));
        int direction;
        if (getElevatorPositions(targetPosIndex) < getCurrentPos()) {
            direction = 1;
        }
        else if (getElevatorPositions(targetPosIndex) > getCurrentPos()) {
            direction = -1;
        }
        else {
            direction = 0;
        }
        return direction;
    }

    // Check if limit switches are pushed in, true means pushed in INVERTED (DONT KNOW HOW THEY WILL BE WIRED)
    public boolean getBottomSwitchState(){
        return !elevatorBottomSwitch.get();
    }

    // public boolean getTopSwitchState(){
    //     return !elevatorTopSwitch.get();
    // }

    // Returns true if either switch is pushed in
    public boolean getSwitchesStates() {
        boolean state;

        // if(getTopSwitchState() == true || getBottomSwitchState() == true) {
        if(getBottomSwitchState() == true) {
            state = true;
        }
        else {
            state = false;
        }
        return state;
    }

    // Movement of the motor
    public void up(){
        System.out.println("Moving on up!");
        elevatorMotorLeft.set(ELEVATOR_UP_SPEED);
    }

    public void down(){
        System.out.println("Moving down below!");
        elevatorMotorLeft.set(ELEVATOR_DOWN_SPEED);
    }

    public void directionMove(int direction) {
        switch(direction) {
            case 1:
                up();
                break;
            case -1:
                down();
                break;
            case 0:
                stop();
                break;
        }
    }

    // Reset encoder
    public void resetEncoder() {
        enc.reset();
    }

    // Called when an elevator command is interrupted or ended
    public void stop(){
        elevatorMotorLeft.set(0);
    }

    // Called when an elevator command is interrupted or ended
    public void pause(){
        elevatorMotorLeft.set(ELEVATOR_BRAKE_SPEED);
    }
    
    // Command methods

    public void moveInit(int posIndex) {
        distanceReached = false;
        finished = false;

        System.out.println("Moving to position " + posIndex);
        direction = getdirection(posIndex); // get direction and speed motor needs to move
        directionMove(direction);
        System.out.println("Motor moving in direction: " + direction);
    }

    public boolean moveIsFinished(int posIndex) {
        switch (direction) {
            case 1:
                if (getCurrentPos() <= getElevatorPositions(posIndex)) {
                    distanceReached = true;
                }
                break;
            case -1:
                if (getCurrentPos() >= getElevatorPositions(posIndex)) {
                    distanceReached = true;
                }
                break;
            case 0:
                distanceReached = true;
        }
        
        if (distanceReached == true) {
            finished = true;
            System.out.println("Motor stopping!");
        }
        return finished;
    }
}

