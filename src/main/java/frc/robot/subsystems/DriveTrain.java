// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  private CANSparkMax mLeftSlave;
  private CANSparkMax mLeftMaster;
  private CANSparkMax mRightMaster;
  private CANSparkMax mRightSlave;

  /** Creates a new DriveTrain. */
  public DriveTrain() 
  {
    //Initializes all motor controllers
    CANSparkMax mLeftMaster = new CANSparkMax(Constants.LeftMasterCANID, MotorType.kBrushless);
    CANSparkMax mLeftSlave = new CANSparkMax(Constants.LeftSlaveCANID, MotorType.kBrushless);
    CANSparkMax mRightMaster = new CANSparkMax(Constants.RightMasterCANID, MotorType.kBrushless);
    CANSparkMax mRightSlave = new CANSparkMax(Constants.RightSlaveCANID, MotorType.kBrushless);

    //Sets all of them to follow respective ones
    mLeftSlave.follow(mLeftMaster);
    mRightSlave.follow(mRightMaster);

    //Sets left side to be inverted
    mLeftMaster.setInverted(true);
    mLeftSlave.setInverted(true);
    mRightMaster.setInverted(false);
    mRightSlave.setInverted(false);

  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }

  public void setLeftMotors(double speed)
  {
    mLeftMaster.set(speed);
    mLeftSlave.set(speed);
  }

  public void setRightMotos(double speed)
  {
    mRightMaster.set(speed);
    mRightSlave.set(speed);
  }
}
