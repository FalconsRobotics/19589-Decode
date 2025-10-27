package org.firstinspires.ftc.teamcode.Subsystems.Color;

import com.acmerobotics.dashboard.config.Config;

public class Color {
    /// Spectrum: 0.277 - 0.722 ( > 0.722 is White)
    public static double RED = 0.28;
    public static double ORANGE = 0.333;
    public static double YELLOW = 0.380; //0.388
    public static double SAGE = 0.444;
    public static double GREEN = 0.5;
    public static double AZURE = 0.555;
    public static double BLUE = 0.611;
    public static double INDIGO = 0.666;
    public static double VIOLET = 0.722;
    public static double WHITE = 1.0;

    /// Light That Can be Produced by the LED
    public static double MIN = 0.28;
    public static double MAX = 0.723;

    /// Ball Colors
    public static double BALL_GREEN = 0.5;
    public static double BALL_PURPLE = 0.71;

    /// Patterns
    public static class Pattern {
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

        public static class SmoothTo {
            public double currentPosition;
            public double startPosition, targetPosition;
            public double step = 0.03;
            public SmoothTo(double currentPos) {
                this.startPosition = currentPos;
                this.currentPosition = currentPos;
            }

            public double getNextPosition(double target) {
                this.targetPosition = target;
                double error = targetPosition - currentPosition;

                if (Math.abs(error) < 1e-9) {return currentPosition;}

                double direction = Math.signum(error);
                double moveAmount = Math.min(step, Math.abs(error));

                currentPosition += direction * moveAmount;

                if (direction > 0 && currentPosition > targetPosition || direction < 0 && currentPosition < targetPosition) {
                    currentPosition = targetPosition;
                }

                return currentPosition;
            }
        }
    }

    /// ColorSensor
    public enum BallColor {
        GREEN, PURPLE, NULL
    }
    public static boolean withinSetRange(int in, int min, int max) {
        if (in >= min && in <= max) {
            return true;
        }
        else {
            return false;
        }
    }
    public static class RGB {
        public int R, G, B;

        public RGB(int Ri, int Gi, int Bi) {
            this.R = Ri; this.G = Gi; this.B = Bi;
        }
        public void setRGB(int r, int g, int b) {
            this.R = r; this.G = g; this.B = b;
        }
        public void setR(int r) {
            this.R = r;
        }
        public void setG(int g) {
            this.G = g;
        }
        public void setB(int b) {
            this.B = b;
        }
        public int getR() {
            return this.R;
        }
        public int getG() {
            return this.G;
        }
        public int getB() {
            return this.B;
        }


    }
    @Config
    public static class P {
        public static int RMIN = 60;//70
        public static int RMAX = 130;//102
        public static int GMIN = 70;//100
        public static int GMAX = 180;//133
        public static int BMIN = 80;//105
        public static int BMAX = 200;//153
    }
    @Config
    public static class G {
        public static int RMIN = 32;//47
        public static int RMAX = 75;//63
        public static int GMIN = 100;//123
        public static int GMAX = 240;//183
        public static int BMIN = 78;//88
        public static int BMAX = 160;//140
    }

    public static BallColor detectColor(RGB input) {
        if (withinSetRange(input.R, P.RMIN, P.RMAX) && withinSetRange(input.G, P.GMIN, P.GMAX) && withinSetRange(input.B, P.BMIN, P.BMAX)) {
            return BallColor.PURPLE;
        }
        else if (withinSetRange(input.R, G.RMIN, G.RMAX) && withinSetRange(input.G, G.GMIN, G.GMAX) && withinSetRange(input.B, G.BMIN, G.BMAX)) {
            return BallColor.GREEN;
        }
        else {
            return BallColor.NULL;
        }
    }
    public static BallColor detectColor(int r, int g, int b) {
        if (withinSetRange(r, P.RMIN, P.RMAX) && withinSetRange(g, P.GMIN, P.GMAX) && withinSetRange(b, P.BMIN, P.BMAX)) {
            return BallColor.PURPLE;
        }
        else if (withinSetRange(r, G.RMIN, G.RMAX) && withinSetRange(g, G.GMIN, G.GMAX) && withinSetRange(b, G.BMIN, G.BMAX)) {
            return BallColor.GREEN;
        }
        else {
            return BallColor.NULL;
        }
    }
}