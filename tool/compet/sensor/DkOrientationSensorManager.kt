/*
 * Copyright (c) 2017-2020 DarkCompet. All rights reserved.
 */
package tool.compet.sensor

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.view.Display
import android.view.Surface
import android.view.WindowManager
import kotlin.math.min
import kotlin.math.sqrt

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
 *
 * We let you know 4 components for orientation (in radian):
 *
 * - Azimuth: the direction (north/east/south/west) the device is pointing.
 * In other words, Rotation around Oz, rotZ = 0 means Magnetic north.
 *
 * - Pitch: the top-to-bottom tilt of the device. In other words,
 * It is Rotation around earth's Ox, rotX = 0 means flat.
 *
 * - Roll: the left-to-right tilt of the device. In other words,
 * It is Rotation around earth's Oy, rotY = 0 means flat.
 *
 * - Inclination: the angle between Oy of device and Oxy of earth coordinate system.
 * Note that, if you wanna get Declination (diff of magnetic north and true north),
 * see class android.hardware.GeomagneticField.
 */
//todo how to use SensorManager.getAltitude()
class DkOrientationSensorManager @JvmOverloads constructor(context: Context, listener: Listener? = null) :
	SensorEventListener {

	interface Listener {
		/**
		 * @param accuracy value in SensorManager.SENSOR_STATUS_*
		 */
		fun onSensorAccuracyChanged(accuracy: Int)

		/**
		 * @param azimuth     (in radian, range [-pi, pi]) rotation around z-axis, diff angle with earth's north pole.
		 * @param pitch       (in radian, range [-pi/2, pi/2]) rotation around x-axis (east pole).
		 * @param roll        (in radian, range [-pi, pi]) rotation around y-axis (north pole).
		 * @param inclination (in radian, range [-pi/2, pi/2]) diff between earth's surface and magnetic fields.
		 * For declination (diff between true-north with compass's magnetic north) see #GeomagneticField.
		 */
		fun onSensorOrientationChanged(azimuth: Float, pitch: Float, roll: Float, inclination: Float)
	}

	// Low filter will make data get from sensor smoothly
	var applyLowFilter = true

	/**
	 * Specify percent-amount of raw values from Sensors to add to current values.
	 * Eg, we have current and next value: curVals[], nextVals[],
	 * SmoothAlpha f value means curVals will increase amount: f * nextVals[].
	 *
	 * Value should be in range [0.0, 1.0].
	 */
	var smoothAlpha = 0.1f

	/**
	 * Specify frequency of event you wanna get from Sensor manager.
	 * Note that, SENSOR_DELAY_FASTEST will consume large battery ;(,
	 * beside that SENSOR_DELAY_NORMAL will save battery better ;).
	 * See #SensorManager.getDelay() to convert delayType to value of hertz.
	 *
	 * Value is one of SensorManager.SENSOR_DELAY_*
	 */
	var sensorDelay = SensorManager.SENSOR_DELAY_UI

	private val sensorManager: SensorManager
	private val listeners = mutableListOf<Listener>()
	private val defaultDisplay: Display

	private var hasAcc = false
	private var hasMag = false

	private val accVals = FloatArray(3)
	private val magVals = FloatArray(3)

	// Rotation matrix
	private val rotMatrix = FloatArray(9)

	// Remap the matrix based on current device/activity rotation.
	private val rotMatrixAdjusted = FloatArray(9)

	// Inclination matrix
	private val incMatrix = FloatArray(9)

	// Remap the matrix based on current device/activity rotation.
	private val incMatrixAdjusted = FloatArray(9)

	// Orientation result
	private val orientations = FloatArray(3)

	init {
		sensorManager = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager

		// Get the display from the window manager (for rotation).
		val wm = context.getSystemService(Context.WINDOW_SERVICE) as WindowManager
		defaultDisplay = wm.defaultDisplay
		register(listener)
	}

	override fun onSensorChanged(event: SensorEvent) {
		when (event.sensor.type) {
			Sensor.TYPE_GRAVITY, Sensor.TYPE_ACCELEROMETER -> {
				if (applyLowFilter) {
					applyLowFilter(event.values, accVals)
				}
				hasAcc = true
			}
			Sensor.TYPE_MAGNETIC_FIELD -> {
				if (applyLowFilter) {
					applyLowFilter(event.values, magVals)
				}
				hasMag = true
			}
		}

		// Handle when we got both of acc and mag values
		if (hasAcc && hasMag) {
			hasMag = false
			hasAcc = hasMag

			// Get rotation matrix and inclination matrix from sensor values (accVals and magVals)
			if (SensorManager.getRotationMatrix(rotMatrix, incMatrix, accVals, magVals)) {
				// Re-map coordinate system since maybe device rotation occurs
				remapCoordinateSystem(rotMatrix, rotMatrixAdjusted)
				remapCoordinateSystem(incMatrix, incMatrixAdjusted)
				val orientations = SensorManager.getOrientation(rotMatrixAdjusted, orientations)
				val inclination = SensorManager.getInclination(incMatrixAdjusted)
				val azimuth = orientations[0]
				val pitch = orientations[1]
				val roll = orientations[2]

				for (listener in listeners) {
					listener.onSensorOrientationChanged(azimuth, pitch, roll, inclination)
				}
			}
		}
	}

	private fun remapCoordinateSystem(input: FloatArray, output: FloatArray) {
		when (defaultDisplay.rotation) {
			Surface.ROTATION_0 -> {
				System.arraycopy(input, 0, output, 0, input.size)
			}
			Surface.ROTATION_90 -> {
				SensorManager.remapCoordinateSystem(
					input,
					SensorManager.AXIS_Y,
					SensorManager.AXIS_MINUS_X,
					output
				)
			}
			Surface.ROTATION_180 -> {
				SensorManager.remapCoordinateSystem(
					input,
					SensorManager.AXIS_MINUS_X,
					SensorManager.AXIS_MINUS_Y,
					output
				)
			}
			Surface.ROTATION_270 -> {
				SensorManager.remapCoordinateSystem(
					input,
					SensorManager.AXIS_MINUS_Y,
					SensorManager.AXIS_X,
					output
				)
			}
		}
	}

	override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {
		for (listener in listeners) {
			listener.onSensorAccuracyChanged(accuracy)
		}
	}

	fun register(listener: Listener?): DkOrientationSensorManager {
		if (listener != null && !listeners.contains(listener)) {
			listeners.add(listener)
		}
		return this
	}

	fun unregister(listener: Listener): DkOrientationSensorManager {
		listeners.remove(listener)
		return this
	}

	/**
	 * @return status of succeed, fail of Acc and Mag, arr[0] is for Acc, arr[1] is for Mag.
	 */
	fun start(): BooleanArray {
		val sensorManager = sensorManager
		val graSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY)
		val accSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
		val magSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
		var okAcc = sensorManager.registerListener(this, accSensor, sensorDelay)
		val okMag = sensorManager.registerListener(this, magSensor, sensorDelay)

		// Use accelerometer if gravity is not presented
		if (!okAcc) {
			okAcc = sensorManager.registerListener(this, graSensor, sensorDelay)
		}
		return booleanArrayOf(okAcc, okMag)
	}

	fun stop() {
		sensorManager.unregisterListener(this)
	}

	/**
	 * See https://en.wikipedia.org/wiki/Low-pass_filter#Discrete-time_realization
	 */
	private fun applyLowFilter(nextVals: FloatArray, curVals: FloatArray) {
		for (i in min(nextVals.size, curVals.size) - 1 downTo 0) {
			curVals[i] += smoothAlpha * (nextVals[i] - curVals[i])
		}
	}

	fun norm(x: Float, y: Float, z: Float): Float {
		return sqrt(x * x + y * y + z * z)
	}

	fun clamp(v: Float, min: Float, max: Float): Float {
		return if (v > max) max else if (v < min) min else v
	}
}