package org.firstinspires.ftc.teamcode.Util;

import android.text.PrecomputedText;

public class Enums {
    public static enum Artifacts {
        GREEN,
        PURPLE,
        NONE
    }
    public static enum GoalColor{
        RED(24),
        BLUE(20);

        private final int id;

        GoalColor(int id){
            this.id = id;
        }
        public int getValue(){
            return id;
        }

    }
    public static enum Angle{
        LOW_DIS(0),
        MID_DIS(0.225),
        HIGH_DIS(0.5);
        private double value;


        Angle(double value) {
            this.value = value;
        }

        public double getValue(){
            return this.value;
        }

        public Angle next() {
            Angle[] values = Angle.values();
            return values[(this.ordinal() + 1) % values.length];
        }
    }

}
