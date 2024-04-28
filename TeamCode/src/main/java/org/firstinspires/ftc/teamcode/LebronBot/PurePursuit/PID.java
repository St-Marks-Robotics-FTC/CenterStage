package org.firstinspires.ftc.teamcode.LebronBot.PurePursuit;

import java.util.ArrayList;

public class PID {
    private double p;
    private double i;
    private double d;
    private ArrayList<Double> lastError;

    public PID(double p, double i, double d) {
        this.p=p;
        this.i=i;
        this.d=d;
        lastError = new ArrayList<>();
    }

    public double update(double error) {
        return P(error)+I(error)-D(error);
    }

    private double P(double error) {
        return p*Math.abs(error);
    }
    private double I(double error) {
        if (lastError.size()>50) lastError.remove(0);
        lastError.add(Math.abs(error));
        double average = 0;
        for (double x : lastError) average+=x;
        return i*average;
    }

    private double D(double error) {
        return d*Math.abs(Math.abs(error)-lastError.get(lastError.size()-1));
    }
}
