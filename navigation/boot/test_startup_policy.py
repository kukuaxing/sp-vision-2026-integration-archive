import unittest
from startup_policy import StartupBudget

class StartupPolicyTests(unittest.TestCase):
    def test_recorded_47_second_carrier_delay_does_not_fail(self):
        budget=StartupBudget(100.)
        for elapsed in (0.,35.,47.):self.assertFalse(budget.network_wait_long(100.+elapsed))
        budget.network_ready(147.)
        self.assertFalse(budget.localization_expired(162.))
    def test_slow_network_does_not_consume_localization_budget(self):
        budget=StartupBudget(0.)
        self.assertFalse(budget.network_wait_long(119.))
        budget.network_ready(119.)
        self.assertFalse(budget.localization_expired(193.))
        self.assertTrue(budget.localization_expired(194.))
    def test_missing_hardware_reminder_does_not_start_localization_deadline(self):
        budget=StartupBudget(0.)
        self.assertFalse(budget.network_wait_long(119.9))
        for now in (120.,999.,3600.):
            self.assertTrue(budget.network_wait_long(now))
            self.assertFalse(budget.localization_expired(now))
    def test_ready_network_does_not_add_a_fixed_delay(self):
        budget=StartupBudget(10.)
        budget.network_ready(10.2)
        self.assertFalse(budget.localization_expired(11.))
        self.assertTrue(budget.localization_expired(85.2))
if __name__=='__main__':unittest.main()
