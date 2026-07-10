from types import SimpleNamespace

import pytest

import openpilot.cereal.messaging as messaging
from openpilot.selfdrive.modeld import modelrouterd


class Params:
  def __init__(self, **values):
    self.values = values
    self.writes = []

  def get_bool(self, key):
    return self.values.get(key, False)

  def put_bool(self, key, value, block=False):
    self.values[key] = value
    self.writes.append((key, value))


class SubMaster:
  def __init__(self, steps):
    self.steps = iter(steps)
    self.now = 0.
    self.updated = dict.fromkeys(("smallModelV2", "bigModelV2", "carControl", "managerState"), False)
    self.seen = self.updated.copy()
    self.alive = self.updated.copy()
    self.valid = self.updated.copy()
    self.freq_ok = self.updated.copy()
    self.logMonoTime = dict.fromkeys(self.updated, 0)
    self.data = {
      "carControl": SimpleNamespace(enabled=False),
      "managerState": SimpleNamespace(processes=[SimpleNamespace(name="bigmodeld", running=True, shouldBeRunning=True)]),
    }

  def update(self):
    step = next(self.steps)
    self.updated = dict.fromkeys(self.updated, False)
    self.now = step.get("time", self.now)
    if "enabled" in step:
      self.updated["carControl"] = True
      self.seen["carControl"] = self.alive["carControl"] = self.valid["carControl"] = self.freq_ok["carControl"] = True
      self.data["carControl"].enabled = step["enabled"]
    if "big" in step:
      self.seen["bigModelV2"] = True
      self.alive["bigModelV2"] = self.valid["bigModelV2"] = self.freq_ok["bigModelV2"] = step["big"]
    if "message" in step:
      service, self.data[service] = step["message"]
      self.updated[service] = True
      self.valid[service] = True
    if "running" in step:
      self.updated["managerState"] = True
      self.data["managerState"].processes[0].running = step["running"]
    if "should_run" in step:
      self.updated["managerState"] = True
      self.data["managerState"].processes[0].shouldBeRunning = step["should_run"]

  def all_checks(self, services):
    return all(self.seen[s] and self.alive[s] and self.valid[s] and self.freq_ok[s] for s in services)

  def __getitem__(self, service):
    return self.data[service]


class PubMaster:
  def __init__(self):
    self.sent = []

  def send(self, service, message):
    self.sent.append((service, message))


def run_router(monkeypatch, params, steps):
  sm = SubMaster(steps)
  pm = PubMaster()
  monkeypatch.setattr(modelrouterd, "Params", lambda: params)
  monkeypatch.setattr(modelrouterd, "SubMaster", lambda *_: sm)
  monkeypatch.setattr(modelrouterd, "PubMaster", lambda *_: pm)
  monkeypatch.setattr(modelrouterd.time, "monotonic", lambda: sm.now)
  monkeypatch.setattr(modelrouterd, "fill_driving_model_data", lambda *_: None)
  with pytest.raises(StopIteration):
    modelrouterd.main()
  return pm


def test_switches_only_while_disabled_and_latches_failure(monkeypatch):
  params = Params()
  steps = [
    {"time": 1., "big": True, "enabled": True},
    {"time": 2., "big": True, "enabled": False},
    {"time": 3., "big": False, "enabled": True},
    {"time": 5.1, "big": False},
    {"time": 6., "big": False, "enabled": False},
  ]
  run_router(monkeypatch, params, steps)
  assert params.writes == [("UsbGpuActive", True), ("UsbGpuFailed", True), ("UsbGpuActive", False)]


def test_transient_failure_recovers(monkeypatch):
  params = Params()
  steps = [{"time": 1., "big": True}, {"time": 2., "big": False}, {"time": 2.1, "big": True}]
  run_router(monkeypatch, params, steps)
  assert not params.get_bool("UsbGpuFailed")


def test_restart_forwards_big_model_and_odometry(monkeypatch):
  params = Params(UsbGpuActive=True)
  message = messaging.new_message("bigModelV2").bigModelV2
  message.modelV2.frameId = 42
  message.cameraOdometry.frameId = 42
  message.cameraOdometryValid = True
  mismatched = messaging.new_message("bigModelV2").bigModelV2
  mismatched.modelV2.frameId = 42
  mismatched.cameraOdometry.frameId = 41
  steps = [
    {"big": True, "message": ("bigModelV2", mismatched)},
    {"big": True, "message": ("bigModelV2", message)},
    {"big": True, "message": ("bigModelV2", message)},
  ]
  pm = run_router(monkeypatch, params, steps)
  assert [service for service, _ in pm.sent] == ["modelV2", "drivingModelData", "cameraOdometry"]
  assert pm.sent[0][1].modelV2.frameId == pm.sent[2][1].cameraOdometry.frameId == 42
  assert pm.sent[2][1].valid


def test_worker_failure_blocks_promotion(monkeypatch):
  params = Params(UsbGpuFailed=True)
  run_router(monkeypatch, params, [{"big": True, "enabled": False}])
  assert not params.get_bool("UsbGpuActive")


def test_crash_during_load_latches_failure(monkeypatch):
  params = Params()
  run_router(monkeypatch, params, [{"running": False}, {"running": False}])
  assert params.get_bool("UsbGpuFailed")


def test_transient_process_state_does_not_latch(monkeypatch):
  params = Params()
  steps = [
    {"running": False, "should_run": False},
    {"running": True, "should_run": True},
    {"running": False},
    {"running": True},
  ]
  run_router(monkeypatch, params, steps)
  assert not params.get_bool("UsbGpuFailed")
