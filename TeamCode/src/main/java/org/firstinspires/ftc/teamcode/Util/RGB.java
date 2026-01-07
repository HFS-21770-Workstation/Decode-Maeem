package org.firstinspires.ftc.teamcode.Util;

public class RGB{
    double red;
    double green;
    double blue;


    public RGB(double red,double green,double blue){
        double s = red + green + blue;
        this.red = red/s * 1000;
        this.green = green/s * 1000;
        this.blue = blue/s * 1000;
    }

    public double dis(RGB other){
        double diffRed = this.red - other.red;
        double diffGreen = this.green - other.green;
        double diffBlue = this.blue - other.blue;

        double distanceSquared = (diffRed * diffRed) + (diffGreen * diffGreen) + (diffBlue * diffBlue);

        return (double) Math.sqrt(distanceSquared);
    }
}