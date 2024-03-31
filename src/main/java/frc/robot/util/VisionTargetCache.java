package frc.robot.util;

public class VisionTargetCache<T> {
    private final int expireAfterAmount;

    private int cycleAmtSinceLastUpdate;
    private T lastTarget;

    public VisionTargetCache(int expireAfterAmount) {
        this.expireAfterAmount = expireAfterAmount;
    }

    public void addTarget(T target) {
        lastTarget = target;
        cycleAmtSinceLastUpdate = 0;
    }

    public boolean targetNotExpired() {
        return cycleAmtSinceLastUpdate < expireAfterAmount;
    }

    public void reset() {
        cycleAmtSinceLastUpdate = 0;
        lastTarget = null;
    }

    public T getAndIncrement() {
        cycleAmtSinceLastUpdate++;

        return lastTarget;
    }
}