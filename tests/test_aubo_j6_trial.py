"""Diagnostic boundary and raw-result retention; no hardware or transport simulation."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop.aubo.j6_trial import checked_snapshot, main, require_snapshot, sequence_offsets, SEQUENCE_PLAN, REVERSAL_PLAN, SEQUENCE_10HZ_PLAN, SEQUENCE_20HZ_PLAN


class TrialTests(unittest.TestCase):
    def test_20hz_sequence_is_bounded_and_does_not_increase_target_speed(self):
        offsets = sequence_offsets(20)
        self.assertEqual(len(offsets), 201)
        self.assertEqual((offsets[0], max(offsets), offsets[-1]), (0., 2., 0.))
        self.assertTrue(all(0. <= x <= 2. for x in offsets))
        self.assertLess(max(abs(b-a) for a,b in zip(offsets,offsets[1:])) /
                        SEQUENCE_20HZ_PLAN["target_period_s"], .5)

    def test_20hz_preview_never_loads_sdk_and_cannot_combine_modes(self):
        with patch("teleop.aubo.j6_trial.load_trial_native") as load, patch("builtins.print"):
            self.assertEqual(main(["--sequence-20hz"]), 0)
            load.assert_not_called()
        with patch("sys.stderr"), self.assertRaises(SystemExit):
            main(["--sequence-10hz", "--sequence-20hz"])

    def test_10hz_sequence_respects_position_and_step_speed_limits(self):
        offsets = sequence_offsets(10)
        self.assertEqual(len(offsets), SEQUENCE_10HZ_PLAN["target_count"])
        self.assertEqual((offsets[0], max(offsets), offsets[-1]), (0., 2., 0.))
        self.assertTrue(all(0. <= x <= 2. for x in offsets))
        self.assertLess(max(abs(b-a) for a,b in zip(offsets,offsets[1:])) /
                        SEQUENCE_10HZ_PLAN["target_period_s"], .5)

    def test_10hz_preview_does_not_load_sdk_and_modes_are_exclusive(self):
        with patch("teleop.aubo.j6_trial.load_trial_native") as load, patch("builtins.print"):
            self.assertEqual(main(["--sequence-10hz"]), 0)
            load.assert_not_called()
        with patch("sys.stderr"), self.assertRaises(SystemExit):
            main(["--sequence-10hz", "--sequence"])

    def test_reversal_preview_never_loads_sdk_and_has_only_two_fixed_targets(self):
        with patch("teleop.aubo.j6_trial.load_trial_native") as load, patch("builtins.print"):
            self.assertEqual(main(["--reversal-10deg"]), 0)
            load.assert_not_called()
        self.assertEqual(REVERSAL_PLAN["target_offsets_deg"], [10., 0.])
        self.assertEqual(REVERSAL_PLAN["offset_range_deg"], [0., 10.])
        self.assertEqual(REVERSAL_PLAN["velocity_limit_deg_s"], .5)
        self.assertLess(REVERSAL_PLAN["return_trigger_measured_deg"], 10.)

    def test_reversal_cannot_be_combined_with_small_step_sequence(self):
        with patch("sys.stderr"), self.assertRaises(SystemExit):
            main(["--sequence", "--reversal-10deg"])

    def test_sequence_is_bounded_and_reverses_without_crossing_start(self):
        offsets = sequence_offsets()
        self.assertEqual(len(offsets), 21)
        self.assertEqual(offsets[0], 0.)
        self.assertEqual(offsets[-1], 0.)
        self.assertEqual(max(offsets), .8)
        self.assertTrue(all(0. <= x <= .8 for x in offsets))
        self.assertTrue(all(b > a for a, b in zip(offsets[:10], offsets[1:11])))
        self.assertTrue(all(b < a for a, b in zip(offsets[10:-1], offsets[11:])))
        self.assertLess(max(abs(b-a) for a,b in zip(offsets,offsets[1:])) /
                        SEQUENCE_PLAN["target_period_s"], .5)

    def test_sequence_preview_does_not_load_sdk(self):
        with patch("teleop.aubo.j6_trial.load_trial_native") as load, patch("builtins.print"):
            self.assertEqual(main(["--sequence"]), 0)
            load.assert_not_called()

    def test_default_preview_does_not_load_sdk(self):
        with patch("teleop.aubo.j6_trial.load_trial_native") as load, patch("builtins.print"):
            self.assertEqual(main([]), 0)
            load.assert_not_called()

    def test_late_snapshot_keeps_original_results_and_does_not_expand_budget(self):
        raw = {"joints": {"return_code": 0, "data": [0.] * 6},
               "state": {"return_code": 0, "data": 0}}
        client = Mock(snapshot=Mock(return_value=raw))
        with patch("teleop.aubo.j6_trial.time.monotonic", side_effect=[10., 10.23]):
            snap = checked_snapshot(client)
        self.assertIs(snap, raw)
        self.assertTrue(snap["observation_budget_exceeded"])
        self.assertEqual(snap["joints"]["data"], [0.] * 6)
        with self.assertRaisesRegex(RuntimeError, "200 ms"):
            require_snapshot(snap, enforce_budget=True)
        require_snapshot(snap, enforce_budget=False)

    def test_failed_sdk_result_is_preserved(self):
        raw = {"joints": {"return_code": 10007, "data": None},
               "state": {"return_code": 0, "data": 0}}
        snap = checked_snapshot(Mock(snapshot=Mock(return_value=raw)))
        self.assertEqual(snap["joints"]["return_code"], 10007)
        self.assertIn("validation_error", snap)
        with self.assertRaisesRegex(RuntimeError, "query failed"):
            require_snapshot(snap, enforce_budget=False)


if __name__ == "__main__":
    unittest.main(verbosity=2)
