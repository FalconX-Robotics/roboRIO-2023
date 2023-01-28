// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Led extends SubsystemBase {
    private SendableChooser<Pattern> m_ledChooser = new SendableChooser<Pattern>();
    private PWMSparkMax m_ledController = new PWMSparkMax(Constants.LED_PORT);
    private Pattern m_lastSelected = null;

    public Led() {
        for (Pattern pattern : Pattern.values()) {
            m_ledChooser.addOption(pattern.name(), pattern);
        }
        m_ledChooser.setDefaultOption("kNone", Pattern.kNone);

        SmartDashboard.putData("Drivetrain/Led Chooser", m_ledChooser);
    }

    public enum Pattern {
        kNone(0.0), kViolet(0.91), kRed(0.61),  kBlack(0.99), 
        kYellow(0.69), kWhite(0.93), kDarkGray(0.97), kHotPink(0.57), 
        kGold(0.67);

        private double speed;

        private Pattern(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return this.speed;
        }
    }

    /**
     * Sets the led pattern according to values from:
     * <p>
     * www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
     * 
     * @param pattern the pattern to set the led to
     */
    public void setLed(Pattern pattern) {
        m_ledController.set(pattern.getSpeed());
    }

    public void periodic() {
        Pattern selected = m_ledChooser.getSelected();
        if (m_lastSelected != selected) {
            setLed(selected);
            m_lastSelected = selected;
        }
    }
}