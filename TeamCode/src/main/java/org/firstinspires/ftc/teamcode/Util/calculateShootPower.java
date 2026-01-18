package org.firstinspires.ftc.teamcode.Util;

public class calculateShootPower {
    double[] a;

    private calculateShootPower(double[][] matrix) {
        matrix = diagonalizationMatrix(matrix);
        this.a = new double[6];

        for (int i = 0; i < matrix.length; i++) {
            a[i] = matrix[i][matrix[i].length - 1];
        }
    }

    public static double[] multiplyRow(double[] row, double p){
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

    public static double[][] diagonalizationMatrix(double[][] mat){
        int rows = mat.length;
        for (int i = 0; i < rows; i++) {
            int maxRow = i;
            for (int k = i + 1; k < rows; k++) {
                if (Math.abs(mat[k][i]) > Math.abs(mat[maxRow][i])) {
                    maxRow = k;
                }
            }
            swapRows(mat, i, maxRow);
            double first = mat[i][i];
            if (Math.abs(first) > 1e-9) {
                mat[i] = multiplyRow(mat[i], 1/first);
                for (int k = 0; k < rows; k++) {
                    if (k == i) continue;
                    double factor = mat[k][i];
                    mat[k] = rowAddition(mat[k], mat[i], -factor);
                }
            }
        }
        return mat;
    }

    public double calculate(double d, double v){
        double result = a[0]*v*v + a[1]*d*v + a[2]*d*d + a[3]*v + a[4]*d + a[5];
        return Math.max(0, Math.min(1, result));
    }

    public static class Builder{
        public double[][] mat = new double[6][7];
        int samples;

        public Builder(){
            samples = 0;
        }

        public Builder addSample(double d, double v, double p){
            if (samples == 6) return this;
            double[] sampleRow = {v*v, d*v, d*d, v, d, 1, p};
            mat[samples] = sampleRow;
            samples++;
            return this;
        }

        public calculateShootPower build(){
            if (samples == 6) return new calculateShootPower(mat);
            return null;
        }
    }
}
