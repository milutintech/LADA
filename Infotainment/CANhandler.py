import can

# Define CAN bus using MCP2515 on Raspberry Pi
can_interface = 'socketcan'
channel = 'can0'
bus = can.interface.Bus(channel=channel, bustype=can_interface, bitrate=500000)

VCU_ID = 0x554
INFO_ID = 0x553

def read_vcu_data():
    """Read data from the VCU (ID 0x554)."""
    message = bus.recv()
    if message.arbitration_id == VCU_ID:
        bms_soc = message.data[0]
        bms_u_bat = (message.data[1] << 8) | message.data[2]
        bms_i_bat = (message.data[3] << 8) | message.data[4]
        state = message.data[5]
        return bms_soc, bms_u_bat, bms_i_bat, state
    return None

def write_info_data(dmc_trq_rq, nlg_ac_curr_lim_max, max_chg_soc, offroad_mode):
    """Write data to the INFO device (ID 0x553)."""
    data = [0x00] * 8
    data[0] = (dmc_trq_rq >> 8) & 0xFF
    data[1] = dmc_trq_rq & 0xFF
    data[2] = nlg_ac_curr_lim_max
    data[3] = max_chg_soc
    data[4] = offroad_mode
    message = can.Message(arbitration_id=INFO_ID, data=data, is_extended_id=False)
    bus.send(message)

def get_bms_soc():
    """Read BMS_SOC from the VCU."""
    vcu_data = read_vcu_data()
    if vcu_data:
        return vcu_data[0]
    return None

def get_u_bat():
    """Read BMS_U_BAT from the VCU."""
    vcu_data = read_vcu_data()
    if vcu_data:
        return vcu_data[1]
    return None

def get_i_bat():
    """Read BMS_I_BAT from the VCU."""
    vcu_data = read_vcu_data()
    if vcu_data:
        return vcu_data[2]
    return None

def get_state():
    """Read STATE from the VCU."""
    vcu_data = read_vcu_data()
    if vcu_data:
        return vcu_data[3]
    return None

def set_dmc_max_trq(dmc_trq_rq):
    """Set DMC_TrqRq in the INFO message."""
    write_info_data(dmc_trq_rq, 0, 0, 0)

def set_nlg_ac_max_curr(nlg_ac_curr_lim_max):
    """Set NLG_AcCurrLimMax in the INFO message."""
    write_info_data(0, nlg_ac_curr_lim_max, 0, 0)

def set_bat_max_soc(max_chg_soc):
    """Set MAX_CHG_SOC in the INFO message."""
    write_info_data(0, 0, max_chg_soc, 0)

def set_offroad_mode(offroad_mode):
    """Set OFFROAD_MODE in the INFO message."""
    write_info_data(0, 0, 0, offroad_mode)

# Example usage
if __name__ == "__main__":
    # Reading VCU data
    bms_soc = get_bms_soc()
    if bms_soc is not None:
        print(f"BMS_SOC: {bms_soc}")

    u_bat = get_u_bat()
    if u_bat is not None:
        print(f"BMS_U_BAT: {u_bat}")

    i_bat = get_i_bat()
    if i_bat is not None:
        print(f"BMS_I_BAT: {i_bat}")

    state = get_state()
    if state is not None:
        print(f"STATE: {state}")

    # Writing INFO data
    set_dmc_max_trq(1000)
    set_nlg_ac_max_curr(20)
    set_bat_max_soc(80)
    set_offroad_mode(1)