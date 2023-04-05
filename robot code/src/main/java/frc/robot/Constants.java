// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
/**
 * The left-to-right distance between the drivetrain wheels
 *
 * Should be measured from center to center.
 */

    public static class Drive{
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.57;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.57;
        public static final double MAX_SPEED_MULTIPLIER = 1.0; 

        public static final int DRIVETRAIN_PIGEON_ID = 31; 
        public static final String kDriveCANivore = "Drivetrain";

            
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 15; 
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5; 
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 25; 

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 14; 
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; 
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 24;  

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 16; 
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; 
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 26; 

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 17; 
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7; 
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 27; 

        public static final double kP = 5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
        public static double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
        public static double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);
        public static double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

        public static double autoBalanceP = 0.035;
    }

    public static class Auto{
        public static final double holonomicXkP = 2.0;
        public static final double holonomicXkI = 0;
        public static final double holonomicXkD = 0;
        public static final double holonomicYkP = 2.4;
        public static final double holonomicYkI = 0;
        public static final double holonomicYkD = 0;
        public static final double holonomicOkP = 2.0;
        public static final double holonomicOkI = 0.0;
        public static final double holonomicOkD = 0.0;
        public static final double holonomicOMaxVelocity = 2;
        public static final double holonomicOMaxAcceleration = 5;
    }

    public static class Controller{
        public static final double kTriggerThreshold = 0.6;
    }

    public static class Elevator{
        public static double forwardLimitInches = 40;
        public static double reverseLimitInches = 0;

        public static double kP = 0.04;
        public static double kI = 0.0;
        public static double kD = 0.0;

        public static int masterMotorID = 8;
        public static int followerMotorID = 9;
        public static int limitSwitchID = 9;

        public static double peakOutForward = 1;
        public static double peakOutReverse = -1;

        public static double intakeHighPos = 15;
        public static double intakePos = 8.5;
        public static double intakeCloseCubePos = 8.5;
        public static double bottomPos = 0.1;
        public static double scoreHighPos = 35;
        public static double scoreMidPos = 13.3;
        public static double scoreLowPos = 0.1;
        public static double loadDoublePos = 17;
        public static double stowPos = 1;
        public static double loadPos = 0;
    }

    public static class Wrist{
        public static int motorID = 12;
        public static double kP = 0.07;
        public static double kI = 0.0;
        public static double kD = 0.0;

        public static double forwardLimit = 10.75;
        public static double reverseLimit = -10.75;

        public static double intakeBackPos = 3.3;
        public static double intakeUpPos = 3.8;
        public static double intakeCubePos = 5.5;
        public static double intakeCloseCubePos = 4.0;
        public static double loadDoublePos = -5.3;
        public static double scoreHighPos = -4.9;
        public static double scoreMidPos = -5.0;
        public static double scoreLowPos = -2.63;
        public static double carryPos = -1.0;
        public static double shootPos = -2.9;
        public static double shootHighPos = -3.3;
        public static double loadPos = -1.5;
        public static double stowPos = 1.7;
        
    }

    public static class Pivot{
        public static int motorID = 11;
        public static double kP = 1.3;
        public static double kI = 0.0;
        public static double kD = 0.0;

        public static double forwardLimit = 1000;
        public static double reverseLimit = -1000;
        public static double zero = 0.958;

        public static double intakeBackPos = 2.2;
        public static double intakeCloseCubePos = 2.0;
        public static double scoreHighPos = -1.2;
        public static double scoreMidPos = -1.0;
        public static double scoreLowPos = -1.0;
        public static double cubePos = 1.60;
        public static double intakeHighPos = 2.13;
        public static double loadPos = -1;
        public static double loadDoublePos = -0.5;

        public static double shootHighPos = -0.5;
        public static double stowPos = -0.75;
        public static double shootMidPos = -0.4;
        
    }

    public static class Grasper{
        public static int motorID = 13;

        public static double kP = 0.02;
        public static double kF = 0.17;
        public static double kI = 0.0;
        public static double kD = 0.0;
    }

    public static class Manipulator{
        
    }
}
