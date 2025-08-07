package org.firstinspires.ftc.teamcode.robot;

public class Vector {
    private final double x;
    private final double y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector add(Vector other) {
        return new Vector(this.x + other.x, this.y + other.y);
    }

    public Vector subtract(Vector other) {
        return new Vector(this.x - other.x, this.y - other.y);
    }

    public Vector multiply(double scalar) {
        return new Vector(this.x * scalar, this.y * scalar);
    }

    public Vector divide(double scalar) {
        return new Vector(this.x / scalar, this.y / scalar);
    }

    public double getArgument() {
        return Math.atan2(y, x);
    }

    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Rotates the vector by the specified angle in radians.
     *
     * @param angle The angle in radians to rotate the vector.
     * @return A new Vector that is the result of the rotation.
     */
    public Vector rotate(double angle) {
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        return new Vector(x * cos - y * sin, x * sin + y * cos);
    }
}
