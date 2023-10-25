def create_steer_command(packer, steer, mode, raw_cnt):
  """Creates a CAN message for the Seb Smith EPAS Steer Command."""

  values = {
    "STEER_MODE": mode,
    "REQUESTED_STEER_TORQUE": steer,
    "COUNTER": raw_cnt,
  }
  return packer.make_can_msg("OCELOT_STEERING_COMMAND", 0, values)

def toyota_checksum(addr, dat, len):
  cksum = 0
  for i in dat:
    cksum += i
  addr1 = (addr & 0xFF)
  addr2 = (addr >> 8)
  cksum += addr1 + addr2 + len
  cksum = cksum & 0xFF
  print(''.join('{:02x}'.format(cksum)))
  return cksum

def create_toyota_steer_command(packer, steer, steer_req, raw_cnt):
  """Creates a CAN message for the Toyota Steer Command."""

  values = {
    "STEER_REQUEST": steer_req,
    "STEER_TORQUE_CMD": steer,
    "TOYOTA_COUNTER": raw_cnt,
    "SET_ME_1": 1,
  }

  # must manually create the can structure again for checksum
  dat1 = (values["STEER_REQUEST"] & 1) | ((values["TOYOTA_COUNTER"] << 1) & 0x3F) | (1 << 7)
  dat2 = (values["STEER_TORQUE_CMD"] >> 8) & 0xFF
  dat3 = (values["STEER_TORQUE_CMD"]) & 0xFF
  dat = [dat1,dat2,dat3]
  for i in dat:
    print(''.join('{:02x}'.format(i)))

  values["TOYOTA_CHECKSUM"] = toyota_checksum(0x2E4, dat, 5)

  return packer.make_can_msg("TOYOTA_STEERING_LKA", 0, values)

def create_steer_command(packer, steer, enabled, raw_cnt):
  values = {
    "REQUESTED_STEER_TORQUE": steer if enabled else 0.,
    "STEER_MODE": 1 if enabled else 0,
    "COUNTER": raw_cnt,
  }
  return packer.make_can_msg("OCELOT_STEERING_COMMAND", 0, values)

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

  return packer.make_can_msg("PEDAL_GAS_COMMAND", 0, values)

def create_brake_cmd(packer, enabled, brake, raw_cnt):
  values = {
    "BRAKE_POSITION_COMMAND" : brake * 25,
    "BRAKE_RELATIVE_COMMAND": 0,
    "BRAKE_MODE": 2 if enabled else 0,
    "COUNTER" : raw_cnt,
  }
  return packer.make_can_msg("OCELOT_BRAKE_COMMAND", 0, values)
