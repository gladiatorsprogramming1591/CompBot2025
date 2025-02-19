package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Constants {

    
    public class ElevatorConstants {
        public static final int ELEVATOR_LEADER_CAN_ID = 1; // Right
        public static final int ELEVATOR_FOLLOWER_CAN_ID = 2; // Left
        public static final int BOTTOM_LIMIT_SWITCH_ID = 16; 
        
        public static final boolean LEADER_INVERTED = false; 
        public static final boolean FOLLOWER_INVERTED_FROM_LEADER = true;
        public static final double RAMP_RATE = 0.1; 
        public static final int CURRENT_LIMIT = 80; 

        public static final double OUTPUT_MAXIMUM = 1.0; 
        public static final double OUTPUT_MINIMUM = -1.0; 

        public static final double INCHES_PER_ROTATION = 22.0/9.0/4.0;
        public static final double INITIAL_HEIGHT_INCHES = 0;

        //Constants for going up 
        public static final double P_UP = 0.1; 
        public static final double I_UP = 0; 
        public static final double D_UP = 0; 
        public static final double FF_UP = .623; 

        public static final double MAX_VEL_UP = 2000; 
        public static final double MAX_ACCEL_UP = 4000; 
        public static final double ALLOWERD_ERR_UP = 0.08; 

        //Constants for going down
        public static final double P_DOWN = 0.1; 
        public static final double I_DOWN = 0; 
        public static final double D_DOWN = 0; 

        public static final double MAX_VEL_DOWN = 3500; 
        public static final double MAX_ACCEL_DOWN = 6000;
        public static final double ALLOWERD_ERR_DOWN = 1.0; 

        public static final SparkFlexConfig MOTOR_CONFIG = new SparkFlexConfig() {{
            idleMode(IdleMode.kBrake);
            smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
            inverted(ElevatorConstants.LEADER_INVERTED);
            openLoopRampRate(ElevatorConstants.RAMP_RATE);
            limitSwitch.reverseLimitSwitchEnabled(false);
            limitSwitch.forwardLimitSwitchEnabled(false);
            closedLoop.outputRange(-0.1, OUTPUT_MAXIMUM, ClosedLoopSlot.kSlot0) // kslot 0 is up
                .p(ElevatorConstants.P_UP, ClosedLoopSlot.kSlot0)
                .i(ElevatorConstants.I_UP, ClosedLoopSlot.kSlot0)
                .d(ElevatorConstants.D_UP, ClosedLoopSlot.kSlot0);
            closedLoop.maxMotion.maxVelocity(MAX_VEL_UP, ClosedLoopSlot.kSlot0)
                .maxAcceleration(MAX_ACCEL_UP, ClosedLoopSlot.kSlot0)
                .allowedClosedLoopError(ALLOWERD_ERR_UP, ClosedLoopSlot.kSlot0);

            closedLoop.outputRange(OUTPUT_MINIMUM, 1, ClosedLoopSlot.kSlot1) // kslot 1 is down
                .p(ElevatorConstants.P_DOWN, ClosedLoopSlot.kSlot1)
                .i(ElevatorConstants.I_DOWN, ClosedLoopSlot.kSlot1)
                .d(ElevatorConstants.D_DOWN, ClosedLoopSlot.kSlot1);
            closedLoop.maxMotion.maxVelocity(MAX_VEL_DOWN, ClosedLoopSlot.kSlot1)
                .maxAcceleration(MAX_ACCEL_DOWN, ClosedLoopSlot.kSlot1)
                .allowedClosedLoopError(ALLOWERD_ERR_DOWN, ClosedLoopSlot.kSlot1);
        }};


        //Elevator Positions
        public static final double kSTOW = 0.0; 
        public static final double kL1 = 0.2; 
        public static final double kL2 = 7; 
        public static final double kL3 = 15.5; 
        public static final double kL4 = 27; 
        public static final double kPROCESSOR = 0; 
        public static final double kNET = 27;
        public static final double STOW_ANGLE = 0; 

    }

    public class EndEffectorConstants {
        public static final int EE_MOTOR_ID = 3;
        public static final int INTAKE_CURRENT_LIMIT = 0; 
        public static final int INTAKE_RAMP_RATE = 30; 
        public static final boolean INTAKE_INVERTED = false;

        //Coral Constants
        public static final double CORAL_INTAKE_SPEED = 0.2; 
        public static final double CORAL_EJECT_SPEED = 0.5;

        //Algae Constants
        public static final double ALGAE_INTAKE_SPEED = -0.3; 
        public static final double HAS_ALGAE_CURRENT = 10;
        public static final double ALGAE_EJECT_SPEED = 0.1; 
         
        
        public static final SparkFlexConfig MOTOR_CONFIG = new SparkFlexConfig() {{
            idleMode(IdleMode.kBrake);
            smartCurrentLimit(EndEffectorConstants.INTAKE_CURRENT_LIMIT);
            inverted(EndEffectorConstants.INTAKE_INVERTED);
            openLoopRampRate(EndEffectorConstants.INTAKE_RAMP_RATE);
            limitSwitch.reverseLimitSwitchEnabled(false);
            limitSwitch.forwardLimitSwitchEnabled(false);
        }};


    }
    
    public class WristConstants {
        // Wrist Positions
        public static final double kWRIST_STOW = 0; //placeholders (obviously)
        public static final double kGROUND_INTAKE = 0; 
        public static final double kCORAL_MARK_PICKUP = 0;
        public static final int WRIST_CAN_ID = 4; 
        public static final int WRIST_CURRENT_LIMIT = 12; 
        public static final boolean MOTOR_INVERTED = false; 
        public static final double RAMP_RATE = 0.5; 
        public static final double WRIST_P = 0; 
        public static final double WRIST_I = 0; 
        public static final double WRIST_D = 0; 

        public static final SparkMaxConfig MOTOR_CONFIG = new SparkMaxConfig() {{
            idleMode(IdleMode.kCoast);
            smartCurrentLimit(WristConstants.WRIST_CURRENT_LIMIT);
            inverted(WristConstants.MOTOR_INVERTED);
            openLoopRampRate(WristConstants.RAMP_RATE);
            closedLoop.p(WristConstants.WRIST_P);
            closedLoop.i(WristConstants.WRIST_I);
            closedLoop.d(WristConstants.WRIST_D);
            closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
            absoluteEncoder.positionConversionFactor(360);
            absoluteEncoder.zeroOffset((360-30)/360.0);
        }};

    }

    public class CANdleConstants {
        
    }

}
