/*
 * Copyright (c) 2017-2020 DarkCompet. All rights reserved.
 */

package tool.compet.sensor;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.view.Display;
import android.view.Surface;
import android.view.WindowManager;

import java.util.ArrayList;
import java.util.List;

/**
 * To get orientation of the device with earth north pole, we just need 2 sensors
 * TYPE_ACCELEROMETER and TYPE_MAGNETIC_FIELD.
 * If the device has not TYPE_ACCELEROMETER present, we use TYPE_GRAVITY instead.
 *
 * There are 2 kind of coordinate systems, device and earth. They are quiet same to
 * remember if you consider user are looking map in device and walking on the earth surface.
 *
 * In device coordinate system, Ox is from device center -> right,
 * Oy is from device cener -> top, Oz is from device center -> user face.
 *
 * In earth coordinate system, Ox is from user -> east pole, Oy is from user -> north pole,
 * and Oz is from user -> sky (same direction from earth's center -> user).
 *
 * For orientation occurs while receive events, don't worry, we did it for you !
 * Actually, device's coordinate-system which sensor use is based on default orientation
 * of device (query with Display.getRotation() to know it is portrait or landscape), so we
 * checked and used SensorManager.remapCoordinateSystem() to handle in #onSensorChanged().
 *
 * We let you know 4 components for orientation (in radian):
 * <ul>
 *    <li>
 *       Azimuth: the direction (north/east/south/west) the device is pointing.
 *       In other words, Rotation around Oz, rotZ = 0 means Magnetic north.
 *    </li>
 *    <li>
 *       Pitch: the top-to-bottom tilt of the device. In other words,
 *       It is Rotation around earth's Ox, rotX = 0 means flat.
 *    </li>
 *    <li>
 *       Roll: the left-to-right tilt of the device. In other words,
 *       It is Rotation around earth's Oy, rotY = 0 means flat.
 *    </li>
 *    <li>
 *       Inclination: the angle between Oy of device and Oxy of earth coordinate system.
 *       Note that, if you wanna get Declination (diff of magnetic north and true north),
 *       see class android.hardware.GeomagneticField.
 *    </li>
 * </ul>
 */
//todo how to use SensorManager.getAltitude()
public class DkOrientationSensorManager implements SensorEventListener {
	public interface Listener {
		/**
		 * @param accuracy value in SensorManager.SENSOR_STATUS_*
		 */
		void onSensorAccuracyChanged(int accuracy);

		/**
		 * @param azimuth (in radian) rotation around z-axis, diff angle with earth's north pole.
		 * @param pitch (in radian) rotation around x-axis (east pole).
		 * @param roll (in radian) rotation around y-axis (north pole).
		 * @param inclination diff with true north, see #GeomagneticField
		 */
		void onSensorOrientationChanged(double azimuth, double pitch, double roll, double inclination);
	}

	private final List<Listener> listeners = new ArrayList<>();
	private final Display defaultDisplay;
	private final SensorManager sensorManager;
	private int sensorDelay = SensorManager.SENSOR_DELAY_UI;
	private boolean hasAcc;
	private boolean hasMag;

	// For filter raw values from sensors
	private boolean applyLowFilter = true;
	private double smoothAlpha = 0.1;

	private final float[] accVals = new float[3];
	private final float[] magVals = new float[3];

	// Rotation matrix
	private final float[] rotMatrix = new float[9];
	// Remap the matrix based on current device/activity rotation.
	private final float[] rotMatrixAdjusted = new float[9];
	// Inclination matrix
	private final float[] incMatrix = new float[9];
	// Remap the matrix based on current device/activity rotation.
	private final float[] incMatrixAdjusted = new float[9];
	// Orientation result
	private final float[] orientations = new float[3];

	public DkOrientationSensorManager(Context context) {
		this(context, null);
	}

	public DkOrientationSensorManager(Context context, Listener listener) {
		sensorManager = (SensorManager) context.getSystemService(Context.SENSOR_SERVICE);

		// Get the display from the window manager (for rotation).
		WindowManager wm = (WindowManager) context.getSystemService(Context.WINDOW_SERVICE);
		defaultDisplay = wm.getDefaultDisplay();

		register(listener);
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		switch (event.sensor.getType()) {
			case Sensor.TYPE_GRAVITY:
			case Sensor.TYPE_ACCELEROMETER: {
				if (applyLowFilter) {
					applyLowFilter(event.values, accVals);
				}
				hasAcc = true;
				break;
			}
			case Sensor.TYPE_MAGNETIC_FIELD: {
				if (applyLowFilter) {
					applyLowFilter(event.values, magVals);
				}
				hasMag = true;
				break;
			}
		}

		// Handle when we got both of acc and mag values
		if (hasAcc && hasMag) {
			hasAcc = hasMag = false;

			// Get rotation matrix and inclination matrix from sensor values (accVals and magVals)
			if (SensorManager.getRotationMatrix(rotMatrix, incMatrix, accVals, magVals)) {
				// Re-map coordinate system since maybe device rotation occurs
				remapCoordinateSystem(rotMatrix, rotMatrixAdjusted);
				remapCoordinateSystem(incMatrix, incMatrixAdjusted);

				float[] orientations = SensorManager.getOrientation(rotMatrixAdjusted, this.orientations);
				float inclination = SensorManager.getInclination(incMatrixAdjusted);

				double azimuth = orientations[0];
				double pitch = orientations[1];
				double roll = orientations[2];

				for (Listener listener : listeners) {
					listener.onSensorOrientationChanged(azimuth, pitch, roll, inclination);
				}
			}
		}
	}

	private void remapCoordinateSystem(float[] input, float[] output) {
		switch (defaultDisplay.getRotation()) {
			case Surface.ROTATION_0: {
				System.arraycopy(input, 0, output, 0, input.length);
				break;
			}
			case Surface.ROTATION_90: {
				SensorManager.remapCoordinateSystem(
					input,
					SensorManager.AXIS_Y,
					SensorManager.AXIS_MINUS_X,
					output);
				break;
			}
			case Surface.ROTATION_180: {
				SensorManager.remapCoordinateSystem(
					input,
					SensorManager.AXIS_MINUS_X,
					SensorManager.AXIS_MINUS_Y,
					output);
				break;
			}
			case Surface.ROTATION_270: {
				SensorManager.remapCoordinateSystem(
					input,
					SensorManager.AXIS_MINUS_Y,
					SensorManager.AXIS_X,
					output);
				break;
			}
		}
	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		for (Listener listener : listeners) {
			listener.onSensorAccuracyChanged(accuracy);
		}
	}

	public DkOrientationSensorManager register(Listener listener) {
		if (listener != null && ! listeners.contains(listener)) {
			listeners.add(listener);
		}
		return this;
	}

	public DkOrientationSensorManager unregister(Listener listener) {
		listeners.remove(listener);
		return this;
	}

	/**
	 * @return status of succeed, fail of Acc and Mag, arr[0] is for Acc, arr[1] is for Mag.
	 */
	public boolean[] start() {
		Sensor graSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
		Sensor accSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		Sensor magSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

		boolean okAcc = sensorManager.registerListener(this, accSensor, sensorDelay);
		boolean okMag = sensorManager.registerListener(this, magSensor, sensorDelay);

		// Use accelerometer if gravity is not presented
		if (!okAcc) {
			okAcc = sensorManager.registerListener(this, graSensor, sensorDelay);
		}

		return new boolean[]{okAcc, okMag};
	}

	public void stop() {
		sensorManager.unregisterListener(this);
	}

	/**
	 * See https://en.wikipedia.org/wiki/Low-pass_filter#Discrete-time_realization
	 */
	private void applyLowFilter(float[] nextVals, float[] curVals) {
		for (int i = Math.min(nextVals.length, curVals.length) - 1; i >= 0; --i) {
			curVals[i] += smoothAlpha * (nextVals[i] - curVals[i]);
		}
	}

	public double getSmoothAlpha() {
		return smoothAlpha;
	}

	/**
	 * Specify frequency of event you wanna get from Sensor manager.
	 * Note that, SENSOR_DELAY_FASTEST will consume large battery ;(,
	 * beside that SENSOR_DELAY_NORMAL will save battery better ;).
	 * See #SensorManager.getDelay() to convert delayType to value of hertz.
	 *
	 * @param delayType is type of SensorManager.SENSOR_DELAY_*
	 */
	public DkOrientationSensorManager setSensorDelay(int delayType) {
		sensorDelay = delayType;
		return this;
	}

	/**
	 * Specify percent-amount of raw values from Sensors to add to current values.
	 * Eg, we have current and next value: curVals[], nextVals[],
	 * SmoothAlpha f value means curVals will increase amount: f * nextVals[].
	 *
	 * @param smoothAlpha should be in range [0.0, 1.0].
	 */
	public DkOrientationSensorManager setSmoothAlpha(double smoothAlpha) {
		this.smoothAlpha = smoothAlpha;
		return this;
	}

	public void setApplyLowFilter(boolean applyLowFilter) {
		this.applyLowFilter = applyLowFilter;
	}

	public boolean isApplyLowFilter() {
		return applyLowFilter;
	}
}
