// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
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

    //Making Trapezoidal Motion Profiling Parts
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(60, 120);
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(); //Making Goal
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(); //Making Setpoint

    ProfiledPIDController turnPIDController = new ProfiledPIDController(Constants.kTurnP, 0, Constants.kTurnD, m_constraints);
    
    public AHRS gyro = new AHRS(SPI.Port.kMXP);
    

  
  public driveTrainSubSystem() 
  {
    //Current Limits Motors
    mLeftMaster.setSmartCurrentLimit(80);
    mLeftSlave.setSmartCurrentLimit(80);
    mRightMaster.setSmartCurrentLimit(80);
    mRightSlave.setSmartCurrentLimit(80);

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


    mLeftController.setP(Constants.kTurnP);
    mLeftController.setD(Constants.kTurnD);
    //mLeftController.setFF(Constants.kF);

    //mLeftController.setOutputRange(-180, 180);
    //mLeftController.setSmartMotionAllowedClosedLoopError(1, 0);
    turnPIDController.enableContinuousInput(-180, 180);
    turnPIDController.setTolerance(1, 1);


    mRightController.setP(Constants.kTurnP);
    mRightController.setD(Constants.kTurnD);
    //mRightController.setFF(0.0);//Constants.kF);

    //mRightController.setOutputRange(-180, 180);
    //mRightController.setSmartMotionAllowedClosedLoopError(1, 0);
  



    gyro.zeroYaw();
  
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

     drive(leftOutput/scalingFactor, rightOutput/scalingFactor);

    
  }

  public void turnToAngle(double angle)
  {

    Constants.turnAngle = angle;
    System.out.println("Goal Angle " + angle);
    System.out.println("Gyro Angle YAW (TurntoAngle) " + gyro.getYaw());

    m_goal = new TrapezoidProfile.State(angle, 0);
  
    //var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    //m_setpoint = profile.calculate(gyro.getYaw());
    //double turnSpeed = m_setpoint.velocity;
    double turnSpeed = turnPIDController.calculate(gyro.getYaw(), angle);
        
    System.out.println("Turn Speed " + turnSpeed);
 

    mLeftController.setReference(turnSpeed, CANSparkMax.ControlType.kDutyCycle, 0);
    mRightController.setReference(turnSpeed, CANSparkMax.ControlType.kDutyCycle, 0);
    /*
    m_goal = new TrapezoidProfile.State(angle, 0);
    var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
    m_setpoint = profile.calculate(Constants.kDt);
    double angleSpeed = m_setpoint.velocity;

    mLeftController.setReference(angleSpeed, CANSparkMax.ControlType.kDutyCycle, 0);
    mRightController.setReference(angleSpeed, CANSparkMax.ControlType.kDutyCycle, 0);
    */
  }

  public double getAngle()
  {
    return Constants.turnAngle;
  }

  @Override
  public void periodic(){}

  @Override
  public void simulationPeriodic(){}
}
