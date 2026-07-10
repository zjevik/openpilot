#!/usr/bin/env python3
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal.messaging import PubMaster, SubMaster
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.fill_model_msg import fill_driving_model_data

BIG_MODEL_TIMEOUT = 0.5


def main() -> None:
  params = Params()
  sm = SubMaster(["smallModelV2", "bigModelV2", "carControl", "managerState"])
  pm = PubMaster(["modelV2", "drivingModelData", "cameraOdometry"])
  selected_model = "bigModelV2" if params.get_bool("UsbGpuActive") else "smallModelV2"
  big_model_failed = params.get_bool("UsbGpuFailed")
  big_model_seen = selected_model == "bigModelV2"
  big_model_unavailable_since = None
  bigmodeld_stopped_count = 0
  last_frame_id = -1

  while True:
    sm.update()
    big_model_available = sm.all_checks(["bigModelV2"])
    if big_model_available:
      big_model_seen = True
      big_model_unavailable_since = None
    elif big_model_seen and big_model_unavailable_since is None:
      big_model_unavailable_since = time.monotonic()

    if sm.updated["managerState"]:
      bigmodeld = next(p for p in sm["managerState"].processes if p.name == "bigmodeld")
      bigmodeld_stopped_count = bigmodeld_stopped_count + 1 if bigmodeld.shouldBeRunning and not bigmodeld.running else 0

    big_model_timed_out = big_model_unavailable_since is not None and time.monotonic() - big_model_unavailable_since > BIG_MODEL_TIMEOUT
    if (big_model_timed_out or bigmodeld_stopped_count >= 2) and not big_model_failed:
      big_model_failed = True
      params.put_bool("UsbGpuFailed", True)
      cloudlog.event("big_model_unavailable", error=True)

    if sm.updated["carControl"] and sm.all_checks(["carControl"]) and not sm["carControl"].enabled:
      big_model_failed |= params.get_bool("UsbGpuFailed")
      next_model = "bigModelV2" if big_model_available and not big_model_failed else "smallModelV2"
      if selected_model != next_model:
        selected_model = next_model
        params.put_bool("UsbGpuActive", selected_model == "bigModelV2", block=True)

    if not sm.updated[selected_model]:
      continue

    output = sm[selected_model]
    frame_id = output.modelV2.frameId
    if output.cameraOdometry.frameId != frame_id or frame_id <= last_frame_id:
      continue

    model_send = messaging.new_message("modelV2", valid=sm.valid[selected_model], logMonoTime=sm.logMonoTime[selected_model])
    model_send.modelV2 = output.modelV2
    drivingdata_send = messaging.new_message("drivingModelData", logMonoTime=sm.logMonoTime[selected_model])
    fill_driving_model_data(drivingdata_send, model_send)
    posenet_send = messaging.new_message("cameraOdometry", valid=output.cameraOdometryValid, logMonoTime=sm.logMonoTime[selected_model])
    posenet_send.cameraOdometry = output.cameraOdometry
    pm.send("modelV2", model_send)
    pm.send("drivingModelData", drivingdata_send)
    pm.send("cameraOdometry", posenet_send)
    last_frame_id = frame_id


if __name__ == "__main__":
  main()
