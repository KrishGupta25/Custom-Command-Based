// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.lang.Math;

public class DriveTrain extends SubsystemBase
{
    //Initializes all motor controllers
    CANSparkMax mLeftMaster = new CANSparkMax(Constants.LeftMasterCANID, MotorType.kBrushless);
    CANSparkMax mLeftSlave = new CANSparkMax(Constants.LeftSlaveCANID, MotorType.kBrushless);
    CANSparkMax mRightMaster = new CANSparkMax(Constants.RightMasterCANID, MotorType.kBrushless);
    CANSparkMax mRightSlave = new CANSparkMax(Constants.RightSlaveCANID, MotorType.kBrushless);

    //Creating Motor Controller Groups
    MotorControllerGroup mLeftMotorControllerGroup = new MotorControllerGroup(mLeftMaster, mLeftSlave);
    MotorControllerGroup mRightMotorControllerGroup = new MotorControllerGroup(mRightMaster, mRightSlave);

    //Creating Encoders for The Motors
    RelativeEncoder mLeftEncoder = mLeftMaster.getEncoder();
    RelativeEncoder mRightEncoder = mRightMaster.getEncoder();

    //Making a Differential DriveTrain
    DifferentialDrive mDrivetrain = new DifferentialDrive(mLeftMotorControllerGroup, mRightMotorControllerGroup);

  

  public DriveTrain() 
  {
    //Restarting Everything to Factory Defualt
    mLeftMaster.restoreFactoryDefaults();
    mLeftSlave.restoreFactoryDefaults();
    mRightMaster.restoreFactoryDefaults();
    mRightSlave.restoreFactoryDefaults();

    //Resetting Encoders Position
    mLeftEncoder.setPosition(0);
    mRightEncoder.setPosition(0);

    //Sets all of them to follow respective ones
    mLeftSlave.follow(mLeftMaster);
    mRightSlave.follow(mRightMaster);

    //Sets left side to be inverted
    mLeftMotorControllerGroup.setInverted(true);
    mRightMotorControllerGroup.setInverted(false);
    
    //Current Limits Motors
    mLeftMaster.setSmartCurrentLimit(80);
    mLeftSlave.setSmartCurrentLimit(80);
    mRightMaster.setSmartCurrentLimit(80);
    mRightSlave.setSmartCurrentLimit(80);


  }


  public void arcadeDrive(double left, double right)
  {
    mDrivetrain.arcadeDrive(left, right);
  }


  public void drive(double left, double right)
  {
    mDrivetrain.tankDrive(left, right);
  }


  public void cheesyDrive(double throttle, double wheel, Boolean isQuickTurn)
  {

    double scaledThrottle = (throttle + (throttle < 0 ? 0.075 : -0.075)) / (1 - 0.075);
    throttle = (Math.abs(throttle) > 0.075) ? scaledThrottle : 0;

    double scaledWheel = (wheel + (wheel < 0 ? 0.075 : -0.075)) / (1 - 0.075);
    wheel = (Math.abs(wheel) > 0.075) ? scaledWheel : 0;

    double wheelNonLinearity = 0.5;
    double denominator = Math.sin(Math.PI/ 2.0 * wheelNonLinearity);

    if (!isQuickTurn)
    {
      wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel);
      wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel);
      wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }

    wheel *= 1;

     double rightOutput = throttle - wheel, leftOutput = throttle + wheel;
     double scalingFactor = Math.max(1.0, Math.abs(throttle));

     drive(leftOutput/scalingFactor, rightOutput/scalingFactor);

    
  }

  @Override
  public void periodic(){}

  @Override
  public void simulationPeriodic(){}
}
