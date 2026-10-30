"""Hardware waiting reminder and a separate bounded localization deadline."""
import math

class StartupBudget:
    def __init__(self, started_s, network_timeout_s=120., localization_timeout_s=75.):
        if not all(math.isfinite(v) for v in (started_s,network_timeout_s,localization_timeout_s)) or min(network_timeout_s,localization_timeout_s)<=0:
            raise ValueError('Startup budgets must be finite and positive')
        self.network_deadline=started_s+network_timeout_s
        self.localization_timeout_s=localization_timeout_s
        self.localization_deadline=None
    def network_wait_long(self,now_s):
        return now_s>=self.network_deadline
    def network_ready(self,now_s):
        self.localization_deadline=now_s+self.localization_timeout_s
    def localization_expired(self,now_s):
        return self.localization_deadline is not None and now_s>=self.localization_deadline
