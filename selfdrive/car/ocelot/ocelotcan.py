def create_steer_command(packer, steer, mode, raw_cnt):
  # Creates a steer command for Ocelot Steer controller and RP Steer Actuator

  values = {
    "STEER_MODE": mode,
    "REQUESTED_STEER_TORQUE": steer,
    "COUNTER": raw_cnt,
  }
  return packer.make_can_msg("OCELOT_STEERING_COMMAND", 2, values)

def create_gas_command(packer, gas_amount, idx):
  # Common gas pedal msg generator
  enable = gas_amount > 0.001

  values = {
    "ENABLE": enable,
    "COUNTER": idx & 0xF,
  }

  if enable:
    values["GAS_COMMAND"] = gas_amount * 2500
    values["GAS_COMMAND2"] = gas_amount * 1250

  return packer.make_can_msg("PEDAL_GAS_COMMAND", 2, values)

def create_brake_cmd(packer, enabled, brake, raw_cnt):
  values = {
    "BRAKE_POSITION_COMMAND" : 0, #brake * 27,
    "BRAKE_RELATIVE_COMMAND": brake if brake > 0 else -250,
    "BRAKE_MODE": 1 if enabled else 0, # 2 is pedal position control
    "COUNTER" : raw_cnt,
  }
  return packer.make_can_msg("OCELOT_BRAKE_COMMAND", 2, values)
