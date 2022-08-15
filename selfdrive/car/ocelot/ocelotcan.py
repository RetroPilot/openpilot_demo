# def create_steer_command(packer, steer, mode, raw_cnt):
#   """Creates a CAN message for the Seb Smith EPAS Steer Command."""

#   values = {
#     "STEER_MODE": mode,
#     "REQUESTED_STEER_TORQUE": steer,
#     "COUNTER": raw_cnt,
#   }
#   return packer.make_can_msg("OCELOT_STEERING_COMMAND", 2, values)

def create_steer_command(packer, steer, enabled, raw_cnt):
  values = {
    "STEER_TORQUE_CMD": (-steer * 18) if enabled else 0.
  }
  return packer.make_can_msg("STEER_COMMAND", 2, values)

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
    "BRAKE_POSITION_COMMAND" : brake * 25,
    "BRAKE_RELATIVE_COMMAND": 0,
    "BRAKE_MODE": 2 if enabled else 0,
    "COUNTER" : raw_cnt,
  }
  return packer.make_can_msg("OCELOT_BRAKE_COMMAND", 2, values)

def xor_pa(data, length):
  csum = 0
  for i in range(length):
    csum ^= data[i+1] # first byte is the checksum
  return csum

def pla_ctrl(packer, bus, enabled, steer_angle, idx):

  # Braking_state:
  # 4 = activateable
  # 6 = active
  # 8 = ready
  # 9 = reboot
  # 10 = abort

  values = { 
    "PLA_Zaehler": idx,
    "PLA_Status": 13 if enabled else 11, # set this based off drive
    "Steering_angle": abs(steer_angle)*1400 if enabled else 0,
    "Steering_direction": 0 if (steer_angle < 0) else 1,
    "Braking_state": 8,
    "Braking_torque": 0, # Nm
  }
  dat = packer.make_can_msg("ParkAssist", bus, values)[2]
  csum = xor_pa(dat, 3)

  values ["PLA_Checksumme"] = csum

  return packer.make_can_msg("ParkAssist", bus, values)
