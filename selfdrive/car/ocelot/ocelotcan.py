# def create_steer_command(packer, steer, mode, raw_cnt):
#   """Creates a CAN message for the Seb Smith EPAS Steer Command."""

#   values = {
#     "STEER_MODE": mode,
#     "REQUESTED_STEER_TORQUE": steer,
#     "COUNTER": raw_cnt,
#   }
#   return packer.make_can_msg("OCELOT_STEERING_COMMAND", 0, values)

MAX_TORQUE = 300. # cannot be over 1000

def create_steer_command(packer, torque, enable, idx):

  values = {
    "ENABLE": enable,
    "COUNTER": idx & 0xF,
  }

  if enable:
    values["TORQUE_COMMAND1"] = 1510 + (torque * MAX_TORQUE)
    values["TORQUE_COMMAND2"] = 1510 - (torque * MAX_TORQUE) 

  return packer.make_can_msg("INTERCEPTOR_STEERING_COMMAND", 2, values)

def create_gas_command(packer, gas_amount, idx):
  # Common gas pedal msg generator
  enable = gas_amount > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER": idx & 0xF,
  }

  if enable:
    values["GAS_COMMAND"] = gas_amount * 255.
    values["GAS_COMMAND2"] = gas_amount * 255.

  return packer.make_can_msg("PEDAL_GAS_COMMAND", 2, values)

def create_brake_cmd(packer, enabled, brake, raw_cnt):
  values = {
    "BRAKE_POSITION_COMMAND" : brake * 7,
    "BRAKE_RELATIVE_COMMAND": 0, #brake,
    "BRAKE_MODE": enabled * 2.,
    "COUNTER" : raw_cnt,
  }
  return packer.make_can_msg("OCELOT_BRAKE_COMMAND", 2, values)