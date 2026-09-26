"""Checks for metric definitions; no assertions that one algorithm must win."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.benchmark_offline import configuration_header, finite_metrics, settling_time, summarize


class ReportTest(unittest.TestCase):
    def row(self, i, error=0.0, valid=True, **extra):
        return {"family": "yaw", "scenario": "steady", "method": "fixture", "seed": "17",
                "time_s": str(i * 0.01), "phase": "before", "observed": "1", "accepted": "1",
                "valid": str(int(valid)), "error": str(error), "rate_error": str(error),
                "forecast_error": str(error), "equivalent_error": str(error),
                "equivalent_forecast_error": str(error), **extra}

    def test_rmse_does_not_confuse_bias_with_standard_deviation(self):
        result = finite_metrics([3, -4, float("nan")])
        self.assertEqual(result["count"], 2)
        self.assertAlmostEqual(result["rmse"], math.sqrt(12.5))
        self.assertAlmostEqual(result["p95_absolute"], 3.95)
        self.assertEqual(finite_metrics([float("nan")])["rmse"], None)

    def test_validity_does_not_hide_bad_untrusted_outputs(self):
        rows = [self.row(99, error=100), self.row(100, error=0), self.row(101, error=4, valid=False)]
        report = summarize(rows)[0]
        self.assertEqual(report["rows"], 2)
        self.assertEqual(report["valid_outputs"], 1)
        self.assertAlmostEqual(report["error"]["rmse"], math.sqrt(8))
        self.assertEqual(report["valid_error"]["rmse"], 0)

    def test_response_requires_whole_contiguous_window(self):
        rows = [self.row(i) for i in range(300, 340)]
        self.assertEqual(settling_time(rows, 3, "error", 0.1), 0)
        rows[10]["valid"] = "0"
        self.assertAlmostEqual(settling_time(rows, 3, "error", 0.1), 0.11)
        del rows[25]
        self.assertIsNone(settling_time(rows, 3, "error", 0.1))

    def test_canonical_configuration_generates_finite_values(self):
        header, values = configuration_header()
        self.assertIn("benchmark_yaw_config", header)
        self.assertIn("benchmark_range_config", header)
        self.assertTrue(all(math.isfinite(value) for value in values.values()))

    def test_valid_nonfinite_output_is_an_error(self):
        with self.assertRaises(ValueError):
            summarize([self.row(100, error=float("nan"))])


if __name__ == "__main__":
    unittest.main()
