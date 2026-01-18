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

    public static double[] rowAddition (double[] row, double[] row2 , double p) {
        double [] outPut = new double[row.length];
        double [] temp = multiplyRow(row2,p);
        for (int i = 0; i < row.length; i++) {
            outPut[i] = row[i] + temp[i];
        }
        return outPut;
    }
    public static double[][] diagonalizationMatrix (double [][] mat){
        int rows = mat.length;
        for (int i = 0; i  < rows; i++) {
            double first = mat[i][i];
            if (first != 0) {

                mat[i] = multiplyRow(mat[i], 1/first);
                for (int k = i + 1; k < rows; k++) {
                    double factor = mat[k][i];
                    mat[k] = rowAddition(mat[k], mat[i], -factor);


                }
            }
        }
        for (int i = rows - 1; i >= 0; i--) {
            double first = mat[i][i];
            if (first != 0) {
                for (int k = i - 1; k >= 0; k--) {
                    double factor = mat[k][i];
                    mat[k] = rowAddition(mat[k], mat[i], -factor);

                }
            }

        }
        return mat;
    }

    public double calculate(double d, double v){
        double result = a[5] + a[4] * d + a[3] * v + a[2] * Math.pow(d, 2) + a[1] * d * v + a[0] * Math.pow(v, 2);
        return Math.max(0, Math.min(1, result));
    }

//    public String getPolynomialString() {
//        if (this.a == null || this.a.length < 6) {
//            return "Polynomial not initialized";
//        }
//
//        return String.format(
//                "P(d, v) = %.4f*v² + %.4f*d*v + %.4f*d² + %.4f*v + %.4f*d + %.4f",
//                a[0], a[1], a[2], a[3], a[4], a[5]
//        );
//    }


    public static class Builder{
        public double[][] mat = new double[6][7];
        int samples;

        public Builder(){
            samples = 0;
        }

        public Builder addSample(double d, double v, double p){
            if (samples == 6) {
                return this;
            }
            //v^2,d*v,d^2,v,d,1,p
            double[] sapmpleRow = {Math.pow(v, 2), d * v, Math.pow(d, 2), v, d, 1, p};
            mat[samples] = sapmpleRow;
            samples++;
            return this;
        }

        public calculateShootPower build(){
            if (samples == 6) {
                return new calculateShootPower(mat);
            }
            return null;
        }


    }
}

