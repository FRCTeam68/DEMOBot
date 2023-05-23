/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * Add your docs here.
 */
public class SmartPID {
/*
    NetworkTableEntry entry_p = Shuffleboard.getTab("SmartPID").add("kProportion", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
    NetworkTableEntry entry_i = Shuffleboard.getTab("SmartPID").add("kIntegral", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
    NetworkTableEntry entry_d = Shuffleboard.getTab("SmartPID").add("kDerivitive", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
    NetworkTableEntry entry_f = Shuffleboard.getTab("SmartPID").add("kFeedForward", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
            */

    GenericEntry entry_setpoint = Shuffleboard.getTab("SmartPID").add("kSetPoint", 0)
            .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0, "max", 10000)).getEntry();
/*
    NetworkTableEntry entry_iZone = Shuffleboard.getTab("SmartPID").add("kiZone", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();

    NetworkTableEntry entry_p_2 = Shuffleboard.getTab("SmartPID").add("kProportion_2", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
    NetworkTableEntry entry_i_2 = Shuffleboard.getTab("SmartPID").add("kIntegral_2", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
    NetworkTableEntry entry_d_2 = Shuffleboard.getTab("SmartPID").add("kDerivitive_2", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
    NetworkTableEntry entry_f_2 = Shuffleboard.getTab("SmartPID").add("kFeedForward_2", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
*/
    GenericEntry entry_setpoint_2 = Shuffleboard.getTab("SmartPID").add("kSetPoint_2", 0)
            .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0, "max", 1000)).getEntry();
/*
    NetworkTableEntry entry_iZone_2 = Shuffleboard.getTab("SmartPID").add("kiZone_2", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();

    NetworkTableEntry entry_p_3 = Shuffleboard.getTab("SmartPID").add("kProportion_3", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
    NetworkTableEntry entry_i_3 = Shuffleboard.getTab("SmartPID").add("kIntegral_3", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
    NetworkTableEntry entry_d_3 = Shuffleboard.getTab("SmartPID").add("kDerivitive_3", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
    NetworkTableEntry entry_f_e = Shuffleboard.getTab("SmartPID").add("kFeedForward_3", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
*/
    GenericEntry entry_setpoint_3 = Shuffleboard.getTab("SmartPID").add("kSetPoint_3", 0)
            .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0, "max", 1000)).getEntry();
/*
    NetworkTableEntry entry_iZone_3 = Shuffleboard.getTab("SmartPID").add("kiZone_3", 0)
            .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 10)).getEntry();
*/
    /**
     * @param These are prebuilt widgets that are already integrated into the
     *              Shuffleboard app
     */
    public double getEntrySetPoint() {
        double rpm = entry_setpoint.getDouble(0);
        return rpm;
    }
/*
    public double getEntryP() {
        double p = entry_p.getDouble(0);
        return p;
    }

    public double getEntryI() {
        double i = entry_i.getDouble(0);
        return i;
    }

    public double getEntryD() {
        double d = entry_d.getDouble(0);
        return d;
    }

    public double getEntryF() {
        double f = entry_f.getDouble(0.00015);
        return f;
    }

    public double getEntryiZone() {
        double iZone = entry_iZone.getDouble(0);
        return iZone;
    }
    */
    public double getEntrySetPoint_2() {
        double rpm_2 = entry_setpoint_2.getDouble(0);
        return rpm_2;
    }
    /*
    public double getEntryP_2() {
        double p_2 = entry_p_2.getDouble(0);
        return p_2;
    }

    public double getEntryI_2() {
        double i_2 = entry_i_2.getDouble(0);
        return i_2;
    }

    public double getEntryD_2() {
        double d_2 = entry_d_2.getDouble(0);
        return d_2;
    }

    public double getEntryF_2() {
        double f_2 = entry_f_2.getDouble(0.00015);
        return f_2;
    }

    public double getEntryiZone_2() {
        double iZone_2 = entry_iZone_2.getDouble(0);
        return iZone_2;
    }
    */
    public double getEntrySetPoint_3() {
        double rpm_3 = entry_setpoint_3.getDouble(0);
        return rpm_3;
    }
    /**
     * 
     * THIS CUSTOM ENTRY CURRENTLY IS IN A NON WORKiNG STATE This will create a
     * network table that will return the value that has been set in the shuffle
     * board app, this way you can dynamically tune PID without having to restart
     * the robot and redeploy code
     * 
     * @param entry   The network table entry that will create the dashboard widget
     * @param VarName This string wll be the name of the dashboard widget
     * @param min     The minimum input of the dashboard widget
     * @param max     The maximum input of the dashboard widget
     */
    public double createCustomEntry(GenericEntry entry, String VarName, double min, double max) {
        entry = Shuffleboard.getTab("SmartPID").add(VarName, 0).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", min, "max", max)).getEntry();
        double entryValue = entry.getDouble(0);
        return entryValue;
    }
}
