package org.firstinspires.ftc.teamcode.color;

public class Pattern {
    public static class CyclicGradient {
        public double currentPosition;
        private boolean movingUp;
        private double step = 0.0005;

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
        public double getNextPosition(double low, double high, double stepI) {
            if (movingUp) {
                if (currentPosition >= high) {movingUp = false;}
            }
            else {
                if (currentPosition <= low) {movingUp = true;}
            }

            currentPosition = currentPosition + (movingUp ? stepI : -stepI);
            currentPosition = Math.max(low, Math.min(high, currentPosition));

            return currentPosition;
        }

    }

    public static class StraightGradient {
        public double currentPosition;
        private double step = 0.0008;

        public StraightGradient(double startPos) {
            this.currentPosition = startPos;
        }

        public double getNextPosition(double min, double max) {
            if (currentPosition >= max) {
                currentPosition = min;
            }
            currentPosition += step;
            currentPosition = Math.max(min, Math.min(max, currentPosition));
            return currentPosition;
        }
        public double getNextPosition(double min, double max, double stepI) {
            if (currentPosition >= max) {
                currentPosition = min;
            }
            currentPosition += stepI;
            currentPosition = Math.max(min, Math.min(max, currentPosition));
            return currentPosition;
        }
    }

    public static class Flash {
        private double ColorOn, ColorOff;
        private double intervalTarget = 8000;
        private double intervalCount = 0;
        public double currentPosition = 0;

        public Flash(double offColor) {
            this.ColorOff = offColor;
            this.currentPosition = offColor;
        }

        public double getFlashPosition(double colorOn, double colorOff) {
            if (intervalCount < intervalTarget) {
                currentPosition = colorOff;
                intervalCount ++ ;
            }
            else if (intervalCount >= intervalTarget && intervalCount <= intervalTarget+intervalTarget) {
                currentPosition = colorOn;
                intervalCount ++ ;
            }
            else if (intervalCount > intervalTarget+intervalTarget) {
                intervalCount = 0;
            }

            return currentPosition;
        }
    }
}
