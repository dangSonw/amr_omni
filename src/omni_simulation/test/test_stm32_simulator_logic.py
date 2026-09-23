import math
import unittest

from omni_simulation.stm32_simulator import (
    EncoderPll,
    SimplePID,
    ScalarKalman,
    compute_crc16_ccitt,
    serialize_serial_frame,
    deserialize_serial_frame,
    compute_imu_covariances,
    CMD_CALIB_TRIGGER_IMU,
    RESP_ACK_NACK,
    STANDARD_GRAVITY_MPS2,
    DEFAULT_INFLATION_ALPHA,
)


class TestEncoderPll(unittest.TestCase):
    def setUp(self):
        self.pll = EncoderPll(bandwidth_rad_s=20.0, counts_per_revolution=2048)

    def test_initialization_and_bandwidth(self):
        self.assertEqual(self.pll.counts_per_rev, 2048)
        self.assertAlmostEqual(self.pll.pll_bandwidth, 20.0)
        self.assertAlmostEqual(self.pll.kp, 40.0)
        self.assertAlmostEqual(self.pll.ki, 400.0)
        self.assertEqual(self.pll.vel_estimate_rad_s, 0.0)
        self.assertEqual(self.pll.pos_estimate_rad, 0.0)

    def test_constant_velocity_tracking(self):
        # 1 revolution per second = 2048 counts/sec = 20.48 counts per 10ms (approx 20 counts/dt)
        # Expected velocity = 2 * pi = 6.283185 rad/s
        dt = 0.01
        expected_omega = 2.0 * math.pi
        counts_per_step = 20.48
        accumulated_counts = 0.0

        for _ in range(300):  # 3 seconds to converge
            accumulated_counts += counts_per_step
            delta = int(round(accumulated_counts))
            accumulated_counts -= delta
            vel = self.pll.update(delta, dt)

        self.assertAlmostEqual(vel, expected_omega, delta=0.15)

    def test_zero_speed_watchdog(self):
        dt = 0.01
        # Spin up first
        for _ in range(50):
            self.pll.update(10, dt)
        self.assertGreater(self.pll.vel_estimate_rad_s, 1.0)

        # Stop pulses for > 50 ms
        for _ in range(6):  # 6 * 10ms = 60ms
            vel = self.pll.update(0, dt)

        # Must snap to exactly 0.0 after watchdog timeout
        self.assertEqual(vel, 0.0)
        self.assertEqual(self.pll.vel_estimate_rad_s, 0.0)
        self.assertEqual(self.pll.pos_error_rad, 0.0)

    def test_raw_timer_rollover(self):
        dt = 0.01
        # Initialize at 65530
        self.pll.update_raw(65530, dt)
        # Rollover to 10 (delta = 16 ticks)
        vel = self.pll.update_raw(10, dt)
        self.assertGreater(vel, 0.0)
        self.assertTrue(math.isfinite(vel))


class TestImuCovariances(unittest.TestCase):
    def test_covariance_inflation_and_bounds(self):
        ang_cov, lin_cov = compute_imu_covariances(dt_sec=0.02)
        # 3x3 diagonal check
        self.assertEqual(ang_cov[0], 1.0e6)  # Unmeasured roll
        self.assertEqual(ang_cov[4], 1.0e6)  # Unmeasured pitch
        self.assertGreaterEqual(ang_cov[8], 1.0e-4)  # Measured yaw rate variance
        self.assertGreaterEqual(lin_cov[0], 1.0e-2)  # Accel variance
        self.assertGreaterEqual(lin_cov[4], 1.0e-2)
        self.assertGreaterEqual(lin_cov[8], 1.0e-2)


class TestSerialProtocol(unittest.TestCase):
    def test_crc16_standard_vector(self):
        # "123456789" ASCII has CCITT CRC 0x29B1
        data = b'123456789'
        crc = compute_crc16_ccitt(data)
        self.assertEqual(crc, 0x29B1)

    def test_frame_serialization_roundtrip(self):
        payload = b'HELLO_IMU'
        frame_bytes = serialize_serial_frame(seq=42, msg_id=CMD_CALIB_TRIGGER_IMU, payload=payload)
        self.assertGreater(len(frame_bytes), 7)
        self.assertEqual(frame_bytes[0], 0xAA)
        self.assertEqual(frame_bytes[1], 0x55)
        self.assertEqual(frame_bytes[-1], 0x7D)

        parsed, consumed = deserialize_serial_frame(frame_bytes)
        self.assertIsNotNone(parsed)
        self.assertEqual(consumed, len(frame_bytes))
        self.assertEqual(parsed['seq'], 42)
        self.assertEqual(parsed['msg_id'], CMD_CALIB_TRIGGER_IMU)
        self.assertEqual(parsed['payload'], payload)

    def test_corrupted_frame_rejection(self):
        frame_bytes = bytearray(serialize_serial_frame(seq=1, msg_id=RESP_ACK_NACK, payload=b'TEST'))
        # Corrupt CRC byte
        frame_bytes[-2] ^= 0xFF
        parsed, consumed = deserialize_serial_frame(bytes(frame_bytes))
        self.assertIsNone(parsed)
        self.assertEqual(consumed, 0)



class TestPIDAndKalmanConfig(unittest.TestCase):
    def test_pid_passthrough_default(self):
        pid = SimplePID()
        self.assertEqual(pid.kp, 1.0)
        self.assertEqual(pid.ki, 0.0)
        self.assertEqual(pid.kd, 0.0)
        self.assertTrue(pid.is_passthrough())

        # In passthrough, update() returns 0.0 (feedback is 0.0, feedforward handles command directly)
        output = pid.update(setpoint=10.0, measurement=5.0, dt=0.01)
        self.assertEqual(output, 0.0)

    def test_pid_active_after_tuning(self):
        pid = SimplePID()
        pid.set_tunings(kp=0.5, ki=0.1, kd=0.01)
        self.assertFalse(pid.is_passthrough())

        # Error = 10 - 5 = 5.0 -> output > 0
        output = pid.update(setpoint=10.0, measurement=5.0, dt=0.01)
        self.assertGreater(output, 0.0)

    def test_scalar_kalman_set_noise(self):
        kalman = ScalarKalman(process_noise=0.5, measurement_noise=0.04)
        self.assertEqual(kalman.process_noise, 0.5)
        self.assertEqual(kalman.measurement_noise, 0.04)

        kalman.set_noise(process_noise=0.1, measurement_noise=0.01)
        self.assertEqual(kalman.process_noise, 0.1)
        self.assertEqual(kalman.measurement_noise, 0.01)

        with self.assertRaises(ValueError):
            kalman.set_noise(-1.0, 0.01)
        with self.assertRaises(ValueError):
            kalman.set_noise(0.1, 0.0)


if __name__ == '__main__':
    unittest.main()
