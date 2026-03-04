package org.firstinspires.ftc.teamcode.Util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class calculateShootPower {

    double[] a;

    private calculateShootPower(double[][] matrix) {
        matrix = diagonalizationMatrix(matrix);
        this.a = new double[matrix.length];
        for (int i = 0; i < matrix.length; i++) {
            a[i] = matrix[i][matrix[i].length - 1];
        }
    }

    public static double[] multiplyRow(double[] row, double p) {
        double[] outPut = new double[row.length];
        for (int i = 0; i < row.length; i++) {
            outPut[i] = row[i] * p;

        }
        return outPut;
    }

    public static double[] rowAddition(double[] row, double[] row2, double p) {
        double[] outPut = new double[row.length];
        double[] temp = multiplyRow(row2, p);
        for (int i = 0; i < row.length; i++) {
            outPut[i] = row[i] + temp[i];
        }
        return outPut;
    }

    public static void swapRows(double[][] mat, int i, int j) {
        double[] temp = mat[i];
        mat[i] = mat[j];
        mat[j] = temp;
    }

    public static double[][] diagonalizationMatrix(double[][] mat) {
        int rows = mat.length;
        int cols = mat[0].length;
        for (int i = 0; i < rows; i++) {
            int maxRow = i;
            for (int k = i + 1; k < rows; k++) {
                if (Math.abs(mat[k][i]) > Math.abs(mat[maxRow][i])) {
                    maxRow = k;
                }
            }
            swapRows(mat, i, maxRow);
            double pivot = mat[i][i];
            if (Math.abs(pivot) > 1e-9) {
                mat[i] = multiplyRow(mat[i], 1 / pivot);
                for (int k = 0; k < rows; k++) {
                    if (k == i) continue;
                    double factor = mat[k][i];
                    mat[k] = rowAddition(mat[k], mat[i], -factor);
                }
            }
        }
        return mat;
    }

    public double calculateVelocity(double d) {
        double velocity = 0;
        for (int i = 0; i < a.length; i++) {
            velocity += a[i] * Math.pow(d, i);
        }
        return velocity;
    }

    public double[] getPolynomial() {
        return a;
    }

    public static class Builder {
        private List<double[]> samplesList = new ArrayList<>();

        public Builder() {}

        public Builder addSample(double d, double p) {
            samplesList.add(new double[]{d, p});
            return this;
        }

        public calculateShootPower build() {
            int n = samplesList.size();
            if (n < 2) return null;

            double[][] mat = new double[n][n + 1];
            for (int i = 0; i < n; i++) {
                double d = samplesList.get(i)[0];
                double p = samplesList.get(i)[1];
                for (int j = 0; j < n; j++) {
                    mat[i][j] = Math.pow(d, j);
                }
                mat[i][n] = p;
            }
            return new calculateShootPower(mat);
        }
    }
}