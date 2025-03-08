package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;

public class Constants {

    public static final class DriveConstants {
        public static final double STATIC_DEADBAND = 0.10; // 10% Deadband before robot moves
        public static final double ROTATION_DEADBAND = 0.12; // 12% Deadband before robot rotates
        public static final double KINETIC_DEADBAND = 0.01; // 1% Deadband to perpendicular axis while robot is in motion

        public static final double TIME_TO_STOP = 0.75;
        public static final double INITIAL_LIMIT = 10.0;
        public static final double LIMIT_SCALE_PER_INCH = 0.92;
    }    
    
    public class ElevatorConstants {
        public static final int ELEVATOR_LEADER_CAN_ID = 1; // Right
        public static final int ELEVATOR_FOLLOWER_CAN_ID = 2; // Left
        
        public static final boolean LEADER_INVERTED = false; 
        public static final boolean FOLLOWER_INVERTED_FROM_LEADER = true;
        public static final double RAMP_RATE = 0.01; 
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

        public static final double MAX_VEL_UP = 100; 
        public static final double MAX_ACCEL_UP = 200; 
        public static final double ALLOWERD_ERR_UP = 0.08; 

        //Constants for going down
        public static final double P_DOWN = 0.05; 
        public static final double I_DOWN = 0; 
        public static final double D_DOWN = 0; 

        public static final double MAX_VEL_DOWN = 2400; 
        public static final double MAX_ACCEL_DOWN = 4800;
        public static final double ALLOWERD_ERR_DOWN = 1.0; 

        //Elevator Positions
        public static final double kSTOW = 0.2; 
        public static final double kL1 = 0.3; 
        public static final double kL2 = 7.50; 
        public static final double kL3 = 15.50; 
        public static final double kL4 = 26.5; 
        public static final double kPROCESSOR = 0.2; 
        public static final double kNET = kL4;
        public static final double ALGAE_HIGH = 15.75;
        public static final double ALGAE_LOW = 9.8;

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
    }

    public class EndEffectorConstants {
        public static final int EE_MOTOR_ID = 3;
        public static final int INTAKE_CURRENT_LIMIT = 60; 
        public static final double INTAKE_RAMP_RATE = 0.1; 
        public static final boolean INTAKE_INVERTED = true;

        //Coral Constants
        public static final double CORAL_INTAKE_SPEED = 0.2; 
        public static final double CORAL_REVERSE_SPEED = -0.15; 
        public static final double CORAL_EJECT_SPEED = 0.5;
        public static final double ARM_CORAL_SPEED = 0;
        public static final double L1_CORAL_EJECT_SPEED = 0.75;
        
        //Algae Constants
        public static final double ALGAE_INTAKE_SPEED = -1; 
        public static final double HAS_ALGAE_CURRENT = 30;
        public static final double ALGAE_EJECT_SPEED = 1.0; 
        public static final double ALGAE_HOLD_SPEED = -0.5;

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
        // Wrist Position
        public static final double WRIST_INTAKE =       249.0;    // 254
        public static final double WRIST_STOW =         WRIST_INTAKE + 16.5;
        public static final double REEF_ACQUIRE_ANGLE = WRIST_INTAKE + 22.0;
        public static final double GROUND_INTAKE =      WRIST_INTAKE + 65.0; 
        public static final double WRIST_PROCESSOR =    WRIST_INTAKE + 25.0;
        public static final double WRIST_HOVER =        WRIST_INTAKE + 34.0;
        public static final double WRIST_HOVER_L4 =     WRIST_INTAKE + 46.0; // +39
        public static final double WRIST_ALGAE_LOW =    WRIST_INTAKE + 41.0;
        public static final double WRIST_ALGAE_HIGH =   WRIST_INTAKE;
        public static final double WRIST_DUNK_CORAL =   WRIST_INTAKE; 
        public static final double WRIST_L1 =           WRIST_INTAKE; 

        public static final int WRIST_CAN_ID = 4; 
        public static final int WRIST_CURRENT_LIMIT = 30; 
        public static final boolean MOTOR_INVERTED = true; 
        public static final double RAMP_RATE = 0.1; 
        public static final double WRIST_P = 0.025; 
        public static final double WRIST_I = 0; 
        public static final double WRIST_D = 0.04; 

        public static final SparkMaxConfig MOTOR_CONFIG = new SparkMaxConfig() {{
            idleMode(IdleMode.kBrake
            );
            smartCurrentLimit(WristConstants.WRIST_CURRENT_LIMIT);
            inverted(WristConstants.MOTOR_INVERTED);
            openLoopRampRate(WristConstants.RAMP_RATE);
            closedLoop.p(WristConstants.WRIST_P);
            closedLoop.i(WristConstants.WRIST_I);
            closedLoop.d(WristConstants.WRIST_D);
            // closedLoop.maxMotion.allowedClosedLoopError(3);
            // closedLoop.maxMotion.maxAcceleration(100000);
            // closedLoop.maxMotion.maxVelocity(120000);
            closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
            absoluteEncoder.positionConversionFactor(360);
            // absoluteEncoder.zeroOffset(252/360);
        }};
        
    }
    public class VisionConstants {
    }

    public class robotInitConstants {
        public static final DigitalInput dIO_port = new DigitalInput(0);
        // .get() returns true if DIO port is unused, and false if DIO port is jumped (with a resistor). Chaz will always be jumped on this channel.
        public static final boolean isCompBot = dIO_port.get();
    }

}
