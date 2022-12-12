from selfdrive.car.isotp_parallel_query import IsoTpParallelQuery
from selfdrive.swaglog import cloudlog
from common.log import Loger
from common.params import Params

EXT_DIAG_REQUEST = b'\x10\x03'
EXT_DIAG_RESPONSE = b'\x50\x03'

COM_CONT_RESPONSE = b''

def enable_radar_tracks(CP, logcan, sendcan):
  print("Try to enable radar tracks")  
  if True: #CP.openpilotLongitudinalControl: 
    rdr_fw = None
    rdr_fw_address = 0x7d0 
    if True:
      for i in range(10):
        print("O yes")
      try:
        for i in range(40):
          try:
            query = IsoTpParallelQuery(sendcan, logcan, 2, [rdr_fw_address], [b'\x10\x07'], [b'\x50\x07'], debug=True)
            for addr, dat in query.get_data(0.1).items(): # pylint: disable=unused-variable
              print("ecu write data by id ...")
              new_config = b"\x00\x00\x00\x01\x00\x01"
              #new_config = b"\x00\x00\x00\x00\x00\x01"
              dataId = b'\x01\x42'
              WRITE_DAT_REQUEST = b'\x2e'
              WRITE_DAT_RESPONSE = b'\x68'
              query = IsoTpParallelQuery(sendcan, logcan, 2, [rdr_fw_address], [WRITE_DAT_REQUEST+dataId+new_config], [WRITE_DAT_RESPONSE], debug=True)
              query.get_data(0)
              print(f"Try {i+1}")
              break
            break
          except Exception as e:
            print(f"Failed {i}: {e}") 
            Loger().add(f"Failed {i}: {e}") 
      except Exception as e:
        print("Failed to enable tracks" + str(e))
        Loger().add("Failed to enable tracks" + str(e))
  print("END Try to enable radar tracks")

def disable_ecu(logcan, sendcan, bus=0, addr=0x7d0, com_cont_req=b'\x28\x83\x01', timeout=0.1, retry=10, debug=False):
  """Silence an ECU by disabling sending and receiving messages using UDS 0x28.
  The ECU will stay silent as long as openpilot keeps sending Tester Present.

  This is used to disable the radar in some cars. Openpilot will emulate the radar.
  WARNING: THIS DISABLES AEB!"""
  cloudlog.warning(f"ecu disable {hex(addr)} ...")

  for i in range(retry):
    try:
      query = IsoTpParallelQuery(sendcan, logcan, bus, [addr], [EXT_DIAG_REQUEST], [EXT_DIAG_RESPONSE], debug=debug)

      for _, _ in query.get_data(timeout).items():
        cloudlog.warning("communication control disable tx/rx ...")

        query = IsoTpParallelQuery(sendcan, logcan, bus, [addr], [com_cont_req], [COM_CONT_RESPONSE], debug=debug)
        query.get_data(0)

        cloudlog.warning("ecu disabled")
        return True

    except Exception:
      cloudlog.exception("ecu disable exception")

    print(f"ecu disable retry ({i+1}) ...")
  cloudlog.warning("ecu disable failed")
  return False

if __name__ == "__main__":
  import time
  import cereal.messaging as messaging
  sendcan = messaging.pub_sock('sendcan')
  logcan = messaging.sub_sock('can')
  time.sleep(1)

  # honda bosch radar disable
  disabled = disable_ecu(logcan, sendcan, bus=1, addr=0x18DAB0F1, com_cont_req=b'\x28\x83\x03', timeout=0.5, debug=False)
  print(f"disabled: {disabled}")
