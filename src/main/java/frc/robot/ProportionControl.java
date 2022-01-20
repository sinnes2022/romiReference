// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class ProportionControl
{
    private double lastVal;
    private double steps = 5;

    public ProportionControl()
    {
        lastVal = 0;
    }

    public double prop(double val)
    {
        if (val < lastVal)
        {
            lastVal = lastVal - ((lastVal - val) / steps);
        }
        else if (val > lastVal)
        {
            lastVal = lastVal + ((val - lastVal) / steps);
        }
        else lastVal = val;

        return lastVal;
    }
}
