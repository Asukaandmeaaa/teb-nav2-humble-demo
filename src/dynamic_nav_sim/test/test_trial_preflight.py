"""Regression checks for an idle scene that communicates but cannot start a trial."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'run_trial', Path(__file__).resolve().parents[1] / 'scripts/run_trial.py')
trial = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trial)


class PreflightTest(unittest.TestCase):
    def test_reported_idle_drift_is_explained(self):
        result = trial.scenario_freshness(
            {'start': [-18.5, 0.]}, {'phase': 'ARMED'},
            [[3327.98, -18.16, 0.]])
        self.assertFalse(result['fresh'])
        self.assertAlmostEqual(result['start_offset_m'], .34, places=5)
        self.assertIn('物理漂移', result['reasons'][0])
        self.assertEqual(result['max_start_offset_m'], .15)

    def test_fresh_scene_can_start(self):
        result = trial.scenario_freshness(
            {'start': [-18.5, 0.]}, {'phase': 'ARMED'}, [[10., -18.499, 0.]])
        self.assertTrue(result['fresh'])

    def test_returning_to_start_does_not_reset_scenario(self):
        result = trial.scenario_freshness(
            {'start': [-18.5, 0.]}, {'phase': 'STOPPED_CENTER'}, [[90., -18.5, 0.]])
        self.assertFalse(result['fresh'])

    def test_missing_data_cannot_be_fresh(self):
        self.assertFalse(trial.scenario_freshness({}, {}, [])['fresh'])


if __name__ == '__main__':
    unittest.main()
