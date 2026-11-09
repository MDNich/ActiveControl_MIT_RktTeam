package info.openrocket.core.aerodynamics.barrowman;

/** Linear interpolation on the original Digital DATCOM grid, with the first axis contiguous. */
final class DatcomTable {
	private final double[] values;
	private final double[][] axes;
	private final int[] strides;

	DatcomTable(double[] values, double[]... axes) {
		this.values = values;
		this.axes = axes;
		strides = new int[axes.length];
		int size = 1;
		for (int i = 0; i < axes.length; i++) {
			strides[i] = size;
			size *= axes[i].length;
		}
		if (size != values.length) throw new IllegalArgumentException("DATCOM chart dimensions");
	}

	double at(double... point) {
		if (point.length != axes.length) throw new IllegalArgumentException("DATCOM chart coordinates");
		return interpolate(axes.length - 1, 0, point);
	}

	private double interpolate(int dimension, int offset, double[] point) {
		if (dimension < 0) return values[offset];
		double[] axis = axes[dimension];
		double value = Math.max(axis[0], Math.min(point[dimension], axis[axis.length - 1]));
		int lower = 0;
		while (lower < axis.length - 2 && axis[lower + 1] < value) lower++;
		double fraction = (value - axis[lower]) / (axis[lower + 1] - axis[lower]);
		int first = offset + lower * strides[dimension];
		double a = interpolate(dimension - 1, first, point);
		double b = interpolate(dimension - 1, first + strides[dimension], point);
		return (1 - fraction) * a + fraction * b;
	}
}
