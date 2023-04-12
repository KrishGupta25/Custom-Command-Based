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
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

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
    DifferentialDrive differentialDrive = new DifferentialDrive(mLeftMotorControllerGroup, mRightMotorControllerGroup);


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
    mLeftMotorControllerGroup.setInverted(false);


  }

  public void arcadeDrive(double fwd, double rot)
  {
    differentialDrive.arcadeDrive(fwd, rot);
  }
  /* (non-Javadoc)
   * @see edu.wpi.first.wpilibj2.command.Subsystem#periodic()
   */
  @Override
  public void periodic(){}

  @Override
  public void simulationPeriodic(){}
}
