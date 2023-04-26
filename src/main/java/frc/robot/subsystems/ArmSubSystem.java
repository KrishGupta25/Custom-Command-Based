// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

/** Add your docs here. */
public class ArmSubSystem extends SubsystemBase
{
    //Making The Motor Controllers
    public CANSparkMax mLeftArm = new CANSparkMax(Constants.LeftArmCANID, MotorType.kBrushless);
    public CANSparkMax mRightArm = new CANSparkMax(Constants.RightArmCANID, MotorType.kBrushless);
    
    //Making The Encoders
    public RelativeEncoder mArmEncoder = mLeftArm.getEncoder();
    
    //Making a FF
    ArmFeedforward mArmFF = new ArmFeedforward(Constants.kArmS, Constants.kArmG, Constants.kArmV);

    //Making Arm PID Controller 
    SparkMaxPIDController mArmController = mLeftArm.getPIDController();

    //Making Trapezoidal Motion Profiling Parts
    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(80, 120);
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State(); //Making Goal
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(); //Making Setpoint

         
public ArmSubSystem()
{
    
    //Restarting Motors to Factory Default
    mLeftArm.restoreFactoryDefaults();
    mRightArm.restoreFactoryDefaults();

    //Resetting Encoders
    mArmEncoder.setPosition(0);

    //Make Arms Follow Eachother
    mRightArm.follow(mLeftArm, true);

    //Current Limits Motors
    mLeftArm.setSmartCurrentLimit(20);
    mRightArm.setSmartCurrentLimit(20);

    //Setting PID Values
    mArmController.setP(Constants.kArmP);
    mArmController.setI(Constants.kArmI);
    mArmController.setD(Constants.kArmD);  
    
    //Setting to Brake Mode
    mLeftArm.setIdleMode(IdleMode.kBrake);
    mRightArm.setIdleMode(IdleMode.kBrake);

}


public void setPosition(String preset)
{
    //Taking input and picking the case
    if (!preset.equals(""))
    {
        double position = 0;

        switch (preset)
        {
            case "stowed": //stowed
                position = 0;
                break;
            case "mid": //mid goal
                position = -38;
                break;
            case "high": //high goal
                position = -120;
                break;

            case "human": //human player
                position = -31.8;
                break;
            case "low": //low goal
                position = -26.3;
                break;
            case "ground": //ground cube intake
            {
                position = - 98.47;
                break;
            }
        }

        System.out.println("Position from Subsystem " + position);
        System.out.println("Encoder Position from Subsystem " + mLeftArm.getEncoder().getPosition());

        Constants.presetPosition = position;
        m_goal = new TrapezoidProfile.State(position, 0);
        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(Constants.kDt);
        
   
        mArmController.setReference(position, CANSparkMax.ControlType.kPosition, 0);

    }
}

public double getPresetPosition()
{
    return Constants.presetPosition;
}


public void brakeMode(Boolean input)
{
    if (input) 
    {
        mLeftArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mLeftArm.setIdleMode(CANSparkMax.IdleMode.kBrake);
    } else
    {
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
            System.out.print(mLeftArm.getEncoder().getPosition());
        }
        else
        {
            mLeftArm.set(speed);
            System.out.print(mLeftArm.getEncoder().getPosition());
        }
}
}

