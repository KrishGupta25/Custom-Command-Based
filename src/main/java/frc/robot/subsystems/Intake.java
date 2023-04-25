// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class Intake extends SubsystemBase
{
    //Making Motor Controllers
    CANSparkMax mLeftIntake = new CANSparkMax(Constants.LeftIntakeCANID, MotorType.kBrushless);
    CANSparkMax mRightIntake = new CANSparkMax(Constants.RightIntakeCANID, MotorType.kBrushless);

    public Intake()
    {
        //Setting it to Follow
        mRightIntake.setInverted(false);
        mLeftIntake.setInverted(false);

        //Resetting Factory Defaults
        mLeftIntake.restoreFactoryDefaults();
        mRightIntake.restoreFactoryDefaults();
    

    
        mRightIntake.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mLeftIntake.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void setSpeed(double speed)
    {
        mLeftIntake.set(-1*speed);
        mRightIntake.set(speed);
    }

}
