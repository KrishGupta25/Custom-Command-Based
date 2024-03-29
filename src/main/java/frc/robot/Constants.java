// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  //PID Constants
    public static final double kDt = 0.02;
    //Drivetrain
      public static final double kP = 0.0046211;
      public static final double kD = 0.0033563;
      public static final double kF = 0.02380952380952381;
      public static final double kWheelDiameter = 0.1524;
      public static final double kTurnP = 0.0024972;//0.000005; //max output/max error
      public static final double kTurnD = 0;//0.0033563;//
      public static final double kAutoP = 3.6/13;
    //Arm
      public static final double kArmP = 0.05; //0.052272
      public static final double kArmI = 0;
      public static final double kArmD = 0.02;
      public static final double kArmFF = 0;
      public static final double kArmIZone = 0;
      //PID Max Constraints
        public static final double kMaxVelocityDegPerSecond = 80;
        public static final double kMaxAccelerationDegPerSecSquared = 100;
        public static final double mGoal = 0;
        public static final double msetpoint = 0;

  //DriveTain.java
    //CANIDS
      public static final int LeftMasterCANID = 11;
      public static final int LeftSlaveCANID = 13;
      public static final int RightMasterCANID = 12;
      public static final int RightSlaveCANID = 14;
      public static final int mainControllerPort = 0; 
    //Others
      public static double turnAngle = 0.0;
  //Arm.java 
    //CANDIS
    public static final int LeftArmCANID = 31;
    public static final int RightArmCANID = 32;
    public static double presetPosition = 0.0;

    //Constants for feedforward
      public static final double kArmS = 0.12855;
      public static final double kArmG = 0.14945;
      public static final double kArmV = 0.050359;

      public static final double  kArmGearRatio = 228.5;
      public static final double  kLowGearRatio = 18.75;
    
  //Intake.java
      //CANIDS
      public static final int LeftIntakeCANID = 21;
      public static final int RightIntakeCANID = 22;

}
