from random import randint
from common.log import Loger
from cereal import car, log, messaging
from common.realtime import DT_CTRL
from common.numpy_fast import clip, interp
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, \
  create_scc11, create_scc12, create_scc13, create_scc14, \
  create_mdps12, create_lfahda_mfc, create_hda_mfc
from selfdrive.car.hyundai.scc_smoother import SccSmoother
from selfdrive.car.hyundai.values import Buttons, CAR, FEATURES, CarControllerParams
from opendbc.can.packer import CANPacker
from common.conversions import Conversions as CV
from common.params import Params
from selfdrive.controls.lib.longcontrol import LongCtrlState
from selfdrive.road_speed_limiter import road_speed_limiter_get_active
from selfdrive.ntune import ntune_scc_get, ntune_scc_enabled

LongitudinalPlanSource = log.LongitudinalPlan.LongitudinalPlanSource
VisualAlert = car.CarControl.HUDControl.VisualAlert
min_set_speed = 30 * CV.KPH_TO_MS

def process_hud_alert(enabled, fingerprint, hud_control):

  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1
  if hud_control.rightLaneDepart:
    right_lane_warning = 1

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.car_fingerprint = CP.carFingerprint
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_name)
    self.frame = 0

    self.apply_steer_last = 0
    self.accel = 0
    self.lkas11_cnt = 0
    self.scc12_cnt = -1

    self.resume_cnt = 0
    self.last_lead_distance = 0
    self.resume_wait_timer = 0

    self.turning_signal_timer = 0
    self.longcontrol = CP.openpilotLongitudinalControl
    self.scc_live = not CP.radarOffCan

    self.turning_indicator_alert = False

    param = Params()

    self.stopping_dist_adj_enabled = False
    self.mad_mode_enabled = param.get_bool('MadModeEnabled')
    self.ldws_opt = param.get_bool('IsLdwsCar')
    self.stock_navi_decel_enabled = param.get_bool('StockNaviDecelEnabled')
    self.keep_steering_turn_signals = param.get_bool('KeepSteeringTurnSignals')
    self.haptic_feedback_speed_camera = param.get_bool('HapticFeedbackWhenSpeedCamera')

    # npilot_manager
    #if param.get_bool("UseNpilotManager"):
    #  self.stopsign_enabled = ntune_scc_get('StopAtStopSign')
    #else:
    #  self.stopsign_enabled = param.get_bool("StopAtStopSign")

    self.stopsign_enabled = ntune_scc_enabled('StopAtStopSign')

    #opkr
    self.prev_dist = 150
    self.decel_zone1 = False
    self.decel_zone2 = False
    self.decel_zone3 = False
    self.decel_zone4 = False

    self.lo_timer = 0
    self.stopped = False
    self.smooth_start = False
    self.change_accel_fast = False
    self.sm = messaging.SubMaster(['controlsState', 'radarState', 'longitudinalPlan'])
    self.log = Loger()

    self.e2e_standstill_enable = param.get_bool("DepartChimeAtResume")
    self.e2e_standstill = False
    self.e2e_standstill_stat = False
    self.e2e_standstill_timer = 0

    self.scc_smoother = SccSmoother()
    self.last_blinker_frame = 0
    self.prev_active_cam = False
    self.active_cam_timer = 0
    self.last_active_cam_frame = 0

    self.angle_limit_counter = 0
    self.cut_steer_frames = 0
    self.cut_steer = False

    self.steer_fault_max_angle = CP.steerFaultMaxAngle
    self.steer_fault_max_frames = CP.steerFaultMaxFrames

  def update(self, CC, CS, controls):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel

    # Steering Torque
    new_steer = int(round(actuators.steer * self.params.STEER_MAX))
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.params)

    # disable when temp fault is active, or below LKA minimum speed
    lkas_active = CC.latActive

    # Disable steering while turning blinker on and speed below 60 kph
    if CS.out.leftBlinker or CS.out.rightBlinker:
      self.turning_signal_timer = 0.5 / DT_CTRL  # Disable for 0.5 Seconds after blinker turned off
    if self.turning_indicator_alert: # set and clear by interface
      lkas_active = 0
    if self.turning_signal_timer > 0:
      self.turning_signal_timer -= 1

    if not lkas_active:
      apply_steer = 0

    self.apply_steer_last = apply_steer

    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint, hud_control)

    if self.haptic_feedback_speed_camera:
      if self.prev_active_cam != self.scc_smoother.active_cam:
        self.prev_active_cam = self.scc_smoother.active_cam
        if self.scc_smoother.active_cam:
          if (self.frame - self.last_active_cam_frame) * DT_CTRL > 10.0:
            self.active_cam_timer = int(1.5 / DT_CTRL)
            self.last_active_cam_frame = self.frame

      if self.active_cam_timer > 0:
        self.active_cam_timer -= 1
        left_lane_warning = right_lane_warning = 1

    clu11_speed = CS.clu11["CF_Clu_Vanz"]
    enabled_speed = 38 if CS.is_set_speed_in_mph else 60
    if clu11_speed > enabled_speed or not lkas_active:
      enabled_speed = clu11_speed

    if self.frame == 0:  # initialize counts from last received count signals
      self.lkas11_cnt = CS.lkas11["CF_Lkas_MsgCount"]

    self.lkas11_cnt = (self.lkas11_cnt + 1) % 0x10

    cut_steer_temp = False

    if self.steer_fault_max_angle > 0:
      if lkas_active and abs(CS.out.steeringAngleDeg) > self.steer_fault_max_angle:
        self.angle_limit_counter += 1
      else:
        self.angle_limit_counter = 0

      # stop requesting torque to avoid 90 degree fault and hold torque with induced temporary fault
      # two cycles avoids race conditions every few minutes
      if self.angle_limit_counter > self.steer_fault_max_frames:
        self.cut_steer = True
      elif self.cut_steer_frames > 1:
        self.cut_steer_frames = 0
        self.cut_steer = False

      if self.cut_steer:
        cut_steer_temp = True
        self.angle_limit_counter = 0
        self.cut_steer_frames += 1

    can_sends = []
    can_sends.append(create_lkas11(self.packer, self.frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, CC.enabled, hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                   left_lane_warning, right_lane_warning, 0, self.ldws_opt, cut_steer_temp))

    if CS.mdps_bus or CS.scc_bus == 1:  # send lkas11 bus 1 if mdps or scc is on bus 1
      can_sends.append(create_lkas11(self.packer, self.frame, self.car_fingerprint, apply_steer, lkas_active,
                                     CS.lkas11, sys_warning, sys_state, CC.enabled, hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                     left_lane_warning, right_lane_warning, 1, self.ldws_opt, cut_steer_temp))

    if self.frame % 2 and CS.mdps_bus: # send clu11 to mdps if it is not on bus 0
      can_sends.append(create_clu11(self.packer, CS.mdps_bus, CS.clu11, Buttons.NONE, enabled_speed))

    if pcm_cancel_cmd and (self.longcontrol and not self.mad_mode_enabled):
      can_sends.append(create_clu11(self.packer, CS.scc_bus, CS.clu11, Buttons.CANCEL, clu11_speed))

    if CS.mdps_bus or self.car_fingerprint in FEATURES["send_mdps12"]:  # send mdps12 to LKAS to prevent LKAS error
      can_sends.append(create_mdps12(self.packer, self.frame, CS.mdps12))

    self.update_auto_resume(CC, CS, clu11_speed, can_sends)
    self.update_scc(CC, CS, actuators, controls, hud_control, can_sends)

    #opkr
    if self.e2e_standstill_enable:
      try:
        self.sm.update(0)

        if self.e2e_standstill:
          self.e2e_standstill_timer += 1
          if self.e2e_standstill_timer > 500:
            self.e2e_standstill = False
            self.e2e_standstill_timer = 0
        elif CS.clu_Vanz > 0:
          self.e2e_standstill = False
          self.e2e_standstill_stat = False
          self.e2e_standstill_timer = 0
        elif self.e2e_standstill_stat and self.sm['longitudinalPlan'].trafficState != 1 and CS.clu_Vanz == 0:
          self.e2e_standstill = True
          self.e2e_standstill_stat = False
          self.e2e_standstill_timer = 0
        elif self.sm['longitudinalPlan'].trafficState == 1 and self.sm['longitudinalPlan'].stopLine[12] < 10 and CS.clu_Vanz == 0:
          self.e2e_standstill_timer += 1
          if self.e2e_standstill_timer > 300:
            self.e2e_standstill_timer = 101
            self.e2e_standstill_stat = True
        else:
          self.e2e_standstill_timer = 0
      except:
        pass

    # 20 Hz LFA MFA message
    if self.frame % 5 == 0:
      activated_hda = road_speed_limiter_get_active()
      # activated_hda: 0 - off, 1 - main road, 2 - highway
      if self.car_fingerprint in FEATURES["send_lfa_mfa"]:
        can_sends.append(create_lfahda_mfc(self.packer, CC.enabled, activated_hda))
      elif CS.has_lfa_hda:
        can_sends.append(create_hda_mfc(self.packer, activated_hda, CS, hud_control.leftLaneVisible, hud_control.rightLaneVisible))

    new_actuators = actuators.copy()
    new_actuators.steer = apply_steer / self.params.STEER_MAX
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends

  def update_auto_resume(self, CC, CS, clu11_speed, can_sends):
    # fix auto resume - by neokii
    if CS.out.cruiseState.standstill and not CS.out.gasPressed:
      if self.last_lead_distance == 0:
        self.last_lead_distance = CS.lead_distance
        self.resume_cnt = 0
        self.resume_wait_timer = 0

      elif self.scc_smoother.is_active(self.frame):
        pass

      elif self.resume_wait_timer > 0:
        self.resume_wait_timer -= 1

      elif abs(CS.lead_distance - self.last_lead_distance) > 0.1:
        can_sends.append(create_clu11(self.packer, CS.scc_bus, CS.clu11, Buttons.RES_ACCEL, clu11_speed))
        self.resume_cnt += 1

        if self.resume_cnt >= randint(6, 8):
          self.resume_cnt = 0
          self.resume_wait_timer = randint(30, 36)

    elif self.last_lead_distance != 0:
      self.last_lead_distance = 0

  def update_scc(self, CC, CS, actuators, controls, hud_control, can_sends):

    # scc smoother
    self.scc_smoother.update(CC.enabled, can_sends, self.packer, CC, CS, self.frame, controls)

    # send scc to car if longcontrol enabled and SCC not on bus 0 or ont live
    if self.longcontrol and CS.cruiseState_enabled and (CS.scc_bus or not self.scc_live):

      # self.lo_timer += 1
      # if self.lo_timer > 200:
      #   self.lo_timer = 0

      if self.frame % 2 == 0:
        set_speed = hud_control.setSpeed

        if not (min_set_speed < set_speed < 255 * CV.KPH_TO_MS):
          set_speed = min_set_speed
        set_speed *= CV.MS_TO_MPH if CS.is_set_speed_in_mph else CV.MS_TO_KPH

        #opkr
        setSpeed = round(set_speed)

        stopping = controls.LoC.long_control_state == LongCtrlState.stopping
        radar_recog = (0 < CS.lead_distance <= 149)

        #opkr
        aReqValue = CS.scc12["aReqValue"]

        #my
        #apply_accel = actuators.accel if CC.longActive and not CS.out.gasPressed else 0

        #neokii
        apply_accel = self.scc_smoother.get_apply_accel(CS, controls.sm, actuators.accel, stopping)

        if 0 < CS.lead_distance <= 149:
          # neokii's logic, opkr mod
          stock_weight = 0.0
          if aReqValue > 0.0:
            stock_weight = interp(CS.lead_distance, [4.0, 8.0, 13.0, 25.0], [0.5, 1.0, 1.0, 0.0])
          elif aReqValue < 0.0:
            stock_weight = interp(CS.lead_distance, [4.0, 25.0], [1.0, 0.0])
          else:
            stock_weight = 0.0         

          if 5.5 < CS.lead_distance <= 6.5 and aReqValue < 0.0 and not CS.out.cruiseState.standstill:
            stock_weight = interp(CS.lead_distance, [5.5, 6.5], [0.2, 1.0])

          if stopping:
            self.stopped = True
          else:
            self.stopped = False

          apply_accel = apply_accel * (1.0 - stock_weight) + aReqValue * stock_weight

        else:
          self.stopped = False
          accel = 0.0

          if self.stopsign_enabled:
            self.sm.update(0)

            if self.sm['longitudinalPlan'].onStop:
              stop_distance = self.sm['longitudinalPlan'].stopLine[12]

              if 0 <= stop_distance <= 100.0 and not CS.out.cruiseState.standstill:

                if stop_distance <= 40 and CS.out.vEgo*CV.MS_TO_MPH >= 25.0 and not self.decel_zone1 and not self.decel_zone2 and not self.decel_zone3:
                  self.decel_zone1 = False
                  self.decel_zone2 = False
                  self.decel_zone3 = True                              
                elif stop_distance <= 30 and CS.out.vEgo*CV.MS_TO_MPH >= 21.0 and not self.decel_zone1 and not self.decel_zone2 and not self.decel_zone3:
                  self.decel_zone1 = False
                  self.decel_zone2 = False
                  self.decel_zone3 = True                  
                elif stop_distance <= 20 and CS.out.vEgo*CV.MS_TO_MPH >= 18.0 and not self.decel_zone1 and not self.decel_zone2 and not self.decel_zone3:
                  self.decel_zone1 = False
                  self.decel_zone2 = False
                  self.decel_zone3 = True                  
                elif stop_distance <= 20 and CS.out.vEgo*CV.MS_TO_MPH > 14.0 and not self.decel_zone1 and not self.decel_zone2 and not self.decel_zone3:
                  self.decel_zone1 = True
                  self.decel_zone2 = False
                  self.decel_zone3 = False                  
                elif stop_distance <= 15 and CS.out.vEgo*CV.MS_TO_MPH <= 10.0 and not self.decel_zone1 and not self.decel_zone2 and not self.decel_zone3:
                  self.decel_zone1 = False
                  self.decel_zone2 = True  
                  self.decel_zone3 = False
                  
                if 0 < stop_distance <= 8.0 and not self.decel_zone3: #force to stop
                  #accel = apply_accel * interp(CS.out.vEgo*CV.MS_TO_MPH, [0.0, 4.0], [1.0, 1.5]) #ok
                  accel = apply_accel * interp(CS.out.vEgo*CV.MS_TO_MPH, [0.0, 4.0, 10.0], [1.0, 1.5, 3.0]) #test
                  apply_accel = min(apply_accel, accel)
                elif 0 < stop_distance <= 8.0 and self.decel_zone3: #force to stop
                  #apply_accel = self.accel - (DT_CTRL * interp(CS.out.vEgo, [0.5, 2.0], [1.0, 5.0]))
                  apply_accel = self.accel - (DT_CTRL * interp(CS.out.vEgo, [0.5, 3.0], [1.0, 5.0]))
                elif self.decel_zone1:
                  accel = apply_accel * interp(CS.out.vEgo*CV.MS_TO_MPH, [5.0, 10.0, 15.0, 20.0, 25.0], [1.0, 1.1, 1.2, 2.5, 3.0]) #ok
                  apply_accel = min(apply_accel, accel)
                elif self.decel_zone2:
                  accel = apply_accel * interp(CS.out.vEgo*CV.MS_TO_MPH, [5.0, 10.0], [0.92, 1.0]) #ok
                  apply_accel = min(apply_accel, accel)
                elif self.decel_zone3:
                  if (apply_accel < 0.):
                    accel = apply_accel * interp(CS.out.vEgo*CV.MS_TO_MPH, [5.0, 10.0], [1.5, 3.0]) #ok
                    apply_accel = min(apply_accel, accel)   
                  else:
                    apply_accel = min(apply_accel, self.accel)
                # elif self.decel_zone4:
                #   if (apply_accel < 0.):
                #     accel = apply_accel * interp(CS.out.vEgo*CV.MS_TO_MPH, [5.0, 10.0], [1.5, 4.5]) #test
                #     apply_accel = min(apply_accel, accel)   
                #   else:
                #     apply_accel = min(apply_accel, self.accel)  
                elif 50 <= stop_distance:
                  apply_accel = min(apply_accel, self.accel)
                # else:
                #   apply_accel = min(apply_accel, self.accel)

              # str_log = ', {:03.0f}, {:02.0f}, {:.03f}, {:}, {:}, {:}, {:}'.format(
              #           stop_distance, CS.out.vEgo*CV.MS_TO_MPH, apply_accel, self.decel_zone1, self.decel_zone2, self.decel_zone3, self.decel_zone4)
              # self.log.add( '{}'.format( str_log ) )
            else:
              self.decel_zone1 = False
              self.decel_zone2 = False
              self.decel_zone3 = False
              self.decel_zone4 = False

          if stopping:
            self.stopped = True
          else:
            self.stopped = False

        apply_accel = clip(apply_accel if CC.longActive else 0, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)

        self.accel = apply_accel

        controls.apply_accel = apply_accel
        aReqValue = CS.scc12["aReqValue"]
        controls.aReqValue = aReqValue

        if aReqValue < controls.aReqValueMin:
          controls.aReqValueMin = controls.aReqValue

        if aReqValue > controls.aReqValueMax:
          controls.aReqValueMax = controls.aReqValue

        if self.stock_navi_decel_enabled:
          controls.sccStockCamAct = CS.scc11["Navi_SCC_Camera_Act"]
          controls.sccStockCamStatus = CS.scc11["Navi_SCC_Camera_Status"]
          apply_accel, stock_cam = self.scc_smoother.get_stock_cam_accel(apply_accel, aReqValue, CS.scc11)
        else:
          controls.sccStockCamAct = 0
          controls.sccStockCamStatus = 0
          stock_cam = False

        if self.scc12_cnt < 0:
          self.scc12_cnt = CS.scc12["CR_VSM_Alive"] if not CS.no_radar else 0

        self.scc12_cnt += 1
        self.scc12_cnt %= 0xF

        can_sends.append(create_scc12(self.packer, apply_accel, CC.enabled, self.scc12_cnt, self.scc_live, CS.scc12,
                                      CS.out.gasPressed, CS.out.brakePressed, CS.out.cruiseState.standstill,
                                      self.car_fingerprint, self.stopped, radar_recog, CS.out.stockAeb))

        can_sends.append(create_scc11(self.packer, self.frame, CC.enabled, set_speed, hud_control.leadVisible, self.scc_live, CS.scc11,
                       self.scc_smoother.active_cam, stock_cam, self.stopped))

        if self.frame % 20 == 0 and CS.has_scc13:
          can_sends.append(create_scc13(self.packer, CS.scc13))

        if CS.has_scc14:
          acc_standstill = stopping if CS.out.vEgo < 2. else False

          lead = self.scc_smoother.get_lead(controls.sm)

          if lead is not None:
            d = lead.dRel
            #obj_gap = 1 if d < 25 else 2 if d < 40 else 3 if d < 60 else 4 if d < 80 else 5
            obj_gap = 2 if d < 40 else 3 if d < 60 else 4 if d < 80 else 5
          else:
            obj_gap = 0

          can_sends.append(
            create_scc14(self.packer, CC.enabled, CS.out.vEgo, acc_standstill, apply_accel, CS.out.gasPressed,
                         obj_gap, CS.scc14))
    else:
      self.scc12_cnt = -1
