package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class Constants {

    
    public class ElevatorConstants {

        public static final int LEFT_CANID = 1;
        public static final int RIGHT_CANID = 2;
        public static final int ELEVATOR_LEADER_CAN_ID = 0;
        public static final int ELEVATOR_FOLLOWER_CAN_ID = 0;

        
        public static final boolean LEADER_INVERTED = false; 
        public static final double RAMP_RATE = 9.0; 
        public static final int CURRENT_LIMIT = 12; 
        
        //Constants for going up 

        public static final double P_UP = 0; 
        public static final double I_UP = 0; 
        public static final double D_UP = 0; 

        public static final double MAX_VEL_UP = 0; 
        public static final double MAX_ACCEL_UP = 0; 
        public static final double ALLOWERD_ERR_UP = 0; 

        //Constants for going up 

        public static final double P_DOWN = 0; 
        public static final double I_DOWN = 0; 
        public static final double D_DOWN = 0; 

        public static final double MAX_VEL_DOWN = 0; 
        public static final double MAX_ACCEL_DOWN = 0; 
        public static final double ALLOWERD_ERR_DOWN = 0; 

        public static final SparkFlexConfig MOTOR_CONFIG = new SparkFlexConfig() {{
            smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
        }};

        //Elevator Positions
        public static final double kSTOW = 0; 
        public static final double kL1 = 0; 
        public static final double kL2 = 0; 
        public static final double kL3 = 0; 
        public static final double kL4 = 0; 
        public static final double kPROCESSOR = 0; 
        public static final double kNET = 0;
        public static final int kCurrentLimitDefault = 0; 

    }

    public class EndEffectorConstants {
        public static final double CORAL_INTAKE_SPEED = 0.2; 
        public static final double ALGAE_INTAKE_SPEED = -0.3; 
        public static final double HAS_ALGAE_CURRENT = 100;
        public static final double ALGAE_EJECT_SPEED = 0.4; // Not very confident in this
        public static final double CORAL_EJECT_SPEED = 0.5; 

    }
    
    public class WristConstants {
        // Wrist Positions
        public static final double kWRIST_STOW = 0; //placeholders (obviously)
        public static final double kGROUND_INTAKE = 0; 
        public static final double kCORAL_MARK_PICKUP = 0; 
    }

}
