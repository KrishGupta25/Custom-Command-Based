// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.Sendable; 
import frc.robot.Constants;

import java.lang.Math;

public class driveTrainSubSystem extends SubsystemBase
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
    public RelativeEncoder mLeftEncoder = mLeftMaster.getEncoder();
    RelativeEncoder mRightEncoder = mRightMaster.getEncoder();

    //Making a Differential DriveTrain
    DifferentialDrive mDrivetrain = new DifferentialDrive(mLeftMotorControllerGroup, mRightMotorControllerGroup);

    SparkMaxPIDController mLeftController = mLeftMaster.getPIDController();
    SparkMaxPIDController mRightController = mRightMaster.getPIDController();

      
    private TrapezoidProfile.Constraints m_turnConstraints = new TrapezoidProfile.Constraints(60, 60);
    private TrapezoidProfile.State m_turnGoal = new TrapezoidProfile.State(); //Making Goal
    private TrapezoidProfile.State m_turnSetpoint = new TrapezoidProfile.State(); //Making Setpoint

    //Making Arm PID Controller 
    ProfiledPIDController mTurnController = new ProfiledPIDController(Constants.kTurnP, 0, Constants.kTurnD, m_turnConstraints);
  
    
    public AHRS gyro = new AHRS(SPI.Port.kMXP);
    

  
  public driveTrainSubSystem() 
  {
    //Current Limits Motors
    mLeftMaster.setSmartCurrentLimit(15);

    mLeftSlave.setSmartCurrentLimit(15);
    mRightMaster.setSmartCurrentLimit(15);
    mRightSlave.setSmartCurrentLimit(15);

    //Restarting Everything to Factory Defualt
    mLeftMaster.restoreFactoryDefaults();
    mLeftSlave.restoreFactoryDefaults();
    mRightMaster.restoreFactoryDefaults();
    mRightSlave.restoreFactoryDefaults();

    
    //Setting IdleMode
    mLeftMaster.setIdleMode(IdleMode.kBrake);
    mLeftSlave.setIdleMode(IdleMode.kBrake);
    mRightMaster.setIdleMode(IdleMode.kBrake);
    mRightSlave.setIdleMode(IdleMode.kBrake);

    //Setting Encoder Position
    mLeftEncoder.setPosition(0);
    mLeftEncoder.setPositionConversionFactor((Constants.kWheelDiameter*Math.PI)/Constants.kLowGearRatio);
    mLeftEncoder.setVelocityConversionFactor((Constants.kWheelDiameter*Math.PI)/Constants.kLowGearRatio/60);

    mRightEncoder.setPosition(0);
    mRightEncoder.setPositionConversionFactor((Constants.kWheelDiameter*Math.PI)/Constants.kLowGearRatio);
    mRightEncoder.setVelocityConversionFactor((Constants.kWheelDiameter*Math.PI)/Constants.kLowGearRatio/60);


    //Sets all of them to follow respective ones
    mLeftSlave.follow(mLeftMaster);
    mRightSlave.follow(mRightMaster);

    //Sets left side to be inverted
    mLeftMotorControllerGroup.setInverted(true);
    mRightMotorControllerGroup.setInverted(false);

    gyro.zeroYaw();


    mTurnController.setP(Constants.kTurnP);
    mTurnController.setD(Constants.kTurnD);

 
  
  }


  public void arcadeDrive(double left, double right)
  {
    mDrivetrain.arcadeDrive(left, right);
  }

  public void drive(double left, double right)
  {
    mDrivetrain.tankDrive(left, right);
  }

  public void RightSlavesetSpeed(double speed)
  {
    mRightSlave.set(speed);
  }

  public void RightMastersetSpeed(double speed)
  {
    mRightMaster.set(speed);
  }

  public void cheesyDrive(double throttle, double wheel, Boolean isQuickTurn)
  {
    //gyro.getYaw();
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

     drive(leftOutput/scalingFactor*0.5, rightOutput/scalingFactor*0.5);

    
  }

  public void turnToAngle(double angle)
  {

    Constants.turnAngle = angle;
    System.out.println("Goal Angle " + angle);
    System.out.println("Gyro Angle YAW (TurntoAngle) " + gyro.getYaw());


    //System.out.println("TURN SETPOINT: " + m_turnSetpoint.velocity);

    System.out.println("Encoder Val: " + mRightEncoder.getPosition());
    //double angleSpeed = mTurnController.calculate(gyro.getYaw(), m_turnGoal);
    //System.out.println("Angle Speed " + angleSpeed);

    
   
    mLeftController.setReference(1, CANSparkMax.ControlType.kVelocity);
    mRightController.setReference(1, CANSparkMax.ControlType.kVelocity);

  }

  public double getAngle()
  {
    return Constants.turnAngle;
  }

  public void driveDistance(double distance)
  {
    
    mLeftController.setP(1);
    mRightController.setP(1);
    mLeftController.setReference(-distance, ControlType.kVelocity);
    mRightController.setReference(distance, ControlType.kVelocity);
  }

  @Override
  public void periodic(){}

  @Override
  public void simulationPeriodic(){}
}
