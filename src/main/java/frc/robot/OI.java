// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class OI 
{
    XboxController mainController = new XboxController(Constants.mainControllerPort);

    public double GetDriveRaxAxis(int axis)
    {
        return mainController.getRawAxis(axis);
    }
}