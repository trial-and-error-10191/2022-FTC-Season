package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorPowerProfile {

    private final double TRAPEZOID_START_POWER = 0.05;
    private final double TRAPEZOID_MAX_POWER = 0.75;
    // percentage of travel distance for ramping up to max power (trapezoid power profile)
    private final double RAMP_UP_LENGTH = 0.25;
    // percentage of travel distance for ramping down to stationary (trapezoid power profile)
    private final double RAMP_DOWN_LENGTH = 0.75;

    private final double LINEAR_MAX_POWER = 0.75;
    private final double LIFT_POWER_INCREMENT = 0.05;

    private final double TRIANGLE_START_POWER = 0.05;
    private final double TRIANGLE_MAX_POWER = 0.5;

    enum MotionProfile {
        LINEAR,
        TRAPEZOIDAL,
        TRIANGULAR
    }

    private final MotionProfile profile;

    public MotorPowerProfile(int profileChoice) {

        if (profileChoice == 1) {
            profile = MotionProfile.TRIANGULAR;
        } else if (profileChoice == 2) {
            profile = MotionProfile.TRAPEZOIDAL;
        } else {
            profile = MotionProfile.LINEAR;
        }
    }

    public double getPower(double currentPower, double percentage, ElapsedTime rampUpTime) {

        double power = 0.0;
        switch (profile) {
            case LINEAR:
                power = LinearPowerProfile(currentPower);
                break;
            case TRAPEZOIDAL:
                power = TrapezoidalPowerProfile(percentage, rampUpTime);
                break;
            case TRIANGULAR:
                power = TriangularPowerProfile(percentage);
                break;
        }
        return power;
    }

    private double TriangularPowerProfile(double progress) {

        double outputPower, b, m;

        if (progress < 0.5)
        {
            b = TRIANGLE_START_POWER;
            m = 2.0 * (TRIANGLE_MAX_POWER - TRIANGLE_START_POWER);
        } else
        {
            b = 2.0 * TRIANGLE_MAX_POWER - TRIANGLE_START_POWER;
            m = 2.0 * (TRIANGLE_START_POWER - TRIANGLE_MAX_POWER);
        }

        outputPower = m * progress + b;
        return outputPower;
    }

    private double LinearPowerProfile(double currentPower) {

        double outputPower = Math.abs(currentPower);
        if (outputPower < LINEAR_MAX_POWER)
        {
            outputPower += LIFT_POWER_INCREMENT;
        }
        return outputPower;
    }

    // Use for 'long distance' travel
    private double TrapezoidalPowerProfile(double progress, ElapsedTime rampUpTime) {

        double outputPower = 0.0;
        if (rampUpTime.time() < 1.0)
        {
            outputPower = rampUpTime.time() * TRAPEZOID_MAX_POWER;
        } else if (progress > RAMP_DOWN_LENGTH)
        {
            double b = 4.0 * TRAPEZOID_MAX_POWER - 3.0 * TRAPEZOID_START_POWER;
            double m = 4.0 * (TRAPEZOID_START_POWER - TRAPEZOID_MAX_POWER);
            outputPower = m * progress + b;
        } else
        {
            outputPower = TRAPEZOID_MAX_POWER;
        }

        return outputPower;
    }
}
