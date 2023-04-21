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
public final class Constants
{
  //DriveTain.java
    //CANIDS
      public static final int LeftMasterCANID = 11;
      public static final int LeftSlaveCANID = 13;
      public static final int RightMasterCANID = 12;
      public static final int RightSlaveCANID = 14;
      public static final int mainControllerPort = 0;    

  //Arm.java 
    //CANDIS
    public static final int LeftArmCANID = 31;
    public static final int RightArmCANID = 32;

    //Constants for feedforward
      public static final double kArmS = 0.12855;
      public static final double kArmG = 0.14945;
      public static final double kArmV = 0.050359;

      public static final double  kArmGearRatio = 228.5;
      public static final double  kLowGearRatio = 10.75;
    //PID Constants
      public static final double kArmP = 0.05; //0.052272
      public static final double kArmI = 0;
      public static final double kArmD = 0;
      public static final double kArmFF = 0;
      public static final double kArmIZone = 0;
    //PID Max Constraints
      public static final double kMaxVelocityDegPerSecond = 80;
      public static final double kMaxAccelerationDegPerSecSquared = 100;
      public static final double mGoal = 0;
      public static final double msetpoint = 0;
  //Intake.java
      //CANIDS
      public static final int LeftIntakeCANID = 21;
      public static final int RightIntakeCANID = 22;

}
