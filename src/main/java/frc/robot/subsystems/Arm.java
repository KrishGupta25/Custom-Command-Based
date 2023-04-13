// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants;

/** Add your docs here. */
public class Arm extends SubsystemBase
{
    //Making The Motor Controllers
    CANSparkMax mLeftArm = new CANSparkMax(Constants.LeftArmCANID, MotorType.kBrushless);
    CANSparkMax mRightArm = new CANSparkMax(Constants.RightArmCANID, MotorType.kBrushless);
    
    //Making The Encoders
    RelativeEncoder mLeftEncoder = mLeftArm.getEncoder();
    RelativeEncoder mRightEncoder = mRightArm.getEncoder();
    
    ArmFeedforward mArmFF = new ArmFeedforward(Constants.kArmS, Constants.kArmG, Constants.kArmV);

public Arm()
{
    //Restarting Motors to Factory Default
    mLeftArm.restoreFactoryDefaults();
    mRightArm.restoreFactoryDefaults();

    //Resetting Encoders
    mLeftEncoder.setPosition(0);
    mRightEncoder.setPosition(0);

    //Make Arms Follow Eachother
    mRightArm.follow(mLeftArm, true);

    //Current Limits Motors
    mLeftArm.setSmartCurrentLimit(30);
    mRightArm.setSmartCurrentLimit(30);

}

public void brakeMode(Boolean input)
{
    if (input) {
        mLeftArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mLeftArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    } else {
        mLeftArm.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mLeftArm.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

}

public void setSpeed(double speed)
{
    //deadzone handling -- ensures inputs are still within the range [0, 1] even after discarding the inputs up to 0.05
        double scaledSpeed = (speed + (speed < 0 ? 0.17 : -0.17)) / (1 - 0.17);
        speed = (Math.abs(speed) > 0.17) ? scaledSpeed : 0;

        if (speed < 0)
        {
            mLeftArm.set(speed*0.5);
        }
        else
        {
            mLeftArm.set(speed);
        }

}
}

