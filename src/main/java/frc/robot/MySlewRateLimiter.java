package frc.robot;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MySlewRateLimiter implements Sendable {
    private double positiveRateLimit;
    private double negativeRateLimit;
    private double prevVal;
    private double prevTime;

    public MySlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
        this.prevVal = initialValue;
        this.prevTime = MathSharedStore.getTimestamp();
    }

    public MySlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, (double)0.0F);
    }

    public double calculate(double input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - this.prevTime;
        this.prevVal += MathUtil.clamp(input - this.prevVal, this.negativeRateLimit * elapsedTime, this.positiveRateLimit * elapsedTime);
        this.prevTime = currentTime;
        return this.prevVal;
    }

    public double lastValue() {
        return this.prevVal;
    }

    public void reset(double value) {
        this.prevVal = value;
        this.prevTime = MathSharedStore.getTimestamp();
    }

    public void updateValues(double positiveRateLimit, double negativeRateLimit) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("MySlewRateLimiter");

        builder.addDoubleProperty("Positive Rate Limit", () -> positiveRateLimit, null);
    }
}
