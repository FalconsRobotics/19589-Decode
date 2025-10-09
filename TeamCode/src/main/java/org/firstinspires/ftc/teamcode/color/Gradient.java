package org.firstinspires.ftc.teamcode.color;

public class Gradient {
    public static class CyclicGradient {
        public double currentPosition;
        private boolean movingUp;
        private double step = 0.001;

        public CyclicGradient(double startPos) {
            this.currentPosition = startPos;
            this.movingUp = true;
        }

        public double getNextPosition(double low, double high) {
            if (movingUp) {
                if (currentPosition >= high) {movingUp = false;}
            }
            else {
                if (currentPosition <= low) {movingUp = true;}
            }

            currentPosition = currentPosition + (movingUp ? step : -step);
            currentPosition = Math.max(low, Math.min(high, currentPosition));

            return currentPosition;
        }

    }
}
