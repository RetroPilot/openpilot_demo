# flake8: noqa

from selfdrive.car import dbc_dict
from cereal import car
Ecu = car.CarParams.Ecu

# Steer torque limits
class SteerLimitParams:
  STEER_MAX = 1500
  STEER_DELTA_UP = 25       # 1.5s time to peak torque
  STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
  STEER_ERROR_MAX = 1500    # max delta between torque cmd and torque motor

class CAR:
  SMART_ROADSTER_COUPE = "SMART ROADSTER COUPE 2003-2006"
  ALBATROSS = "ALBATROSS"

BUTTON_STATES = {
  "accelCruise": False,
  "decelCruise": False,
  "cancel": False,
  "setCruise": False,
  "resumeCruise": False,
  "gapAdjustCruise": False
}

FINGERPRINTS = {
  CAR.SMART_ROADSTER_COUPE: [{
     36: 8, 37: 8, 170: 8, 180: 1, 186: 4, 253:8, 254:8, 255: 8, 426: 6, 452: 8, 464: 8, 466: 8, 467: 8, 512: 6, 513:6, 547: 8, 548: 8, 552: 4, 608: 8, 610: 5, 643: 7, 705: 8, 740: 5, 800: 8, 835: 8, 836: 8, 849: 4, 869: 7, 870: 7, 871: 2, 896: 8, 897: 8, 900: 6, 902: 6, 905: 8, 911: 8, 916: 2, 921: 8, 933: 8, 944: 8, 945: 8, 951: 8, 955: 4, 956: 8, 979: 2, 998: 5, 999: 7, 1000: 8, 1001: 8, 1017: 8, 1041: 8, 1042: 8, 1043: 8, 1044: 8, 1056: 8, 1059: 1, 1114: 8, 1161: 8, 1162: 8, 1163: 8, 1196: 8, 1222: 8, 1224:8, 1227: 8, 1235: 8, 1279: 8, 1552: 8, 1553: 8, 1556: 8, 1557: 8, 1561: 8, 1562: 8, 1568: 8, 1569: 8, 1570: 8, 1571: 8, 1572: 8, 1592: 8, 1596: 8, 1597: 8, 1600: 8, 1664: 8, 1779: 8, 1904: 8, 1912: 8, 1990: 8, 1998: 8, 2016: 8, 2017: 8, 2018: 8, 2019: 8, 2020: 8, 2021: 8, 2022: 8, 2023: 8, 2024: 8
   },
   {
     36: 8, 37: 8, 170: 8, 180: 4, 186: 4, 253:8, 254:8, 255: 8, 426: 6, 452: 8, 464: 8, 466: 8, 467: 8, 512: 6, 513:6, 547: 8, 548: 8, 552: 4, 608: 8, 610: 5, 643: 7, 705: 8, 740: 5, 800: 8, 835: 8, 836: 8, 849: 4, 869: 7, 870: 7, 871: 2, 896: 8, 897: 8, 900: 6, 902: 6, 905: 8, 911: 8, 916: 2, 921: 8, 933: 8, 944: 8, 945: 8, 951: 8, 955: 4, 956: 8, 979: 2, 998: 5, 999: 7, 1000: 8, 1001: 8, 1017: 8, 1041: 8, 1042: 8, 1043: 8, 1044: 8, 1056: 8, 1059: 1, 1114: 8, 1161: 8, 1162: 8, 1163: 8, 1196: 8, 1222: 8, 1224:8, 1227: 8, 1235: 8, 1279: 8, 1552: 8, 1553: 8, 1556: 8, 1557: 8, 1561: 8, 1562: 8, 1568: 8, 1569: 8, 1570: 8, 1571: 8, 1572: 8, 1592: 8, 1596: 8, 1597: 8, 1600: 8, 1664: 8, 1779: 8, 1904: 8, 1912: 8, 1990: 8, 1998: 8, 2016: 8, 2017: 8, 2018: 8, 2019: 8, 2020: 8, 2021: 8, 2022: 8, 2023: 8, 2024: 8
   }],
  CAR.ALBATROSS: [{
     36: 8, 37: 8, 170: 8, 180: 8, 186: 4, 253:8, 254:8, 255: 8, 426: 6, 452: 8, 464: 8, 466: 8, 467: 8, 512: 6, 513:6, 547: 8, 548: 8, 552: 4, 608: 8, 610: 5, 643: 7, 705: 8, 740: 5, 800: 8, 835: 8, 836: 8, 849: 4, 869: 7, 870: 7, 871: 2, 896: 8, 897: 8, 900: 6, 902: 6, 905: 8, 911: 8, 916: 2, 921: 8, 933: 8, 944: 8, 945: 8, 951: 8, 955: 4, 956: 8, 979: 2, 998: 5, 999: 7, 1000: 8, 1001: 8, 1017: 8, 1041: 8, 1042: 8, 1043: 8, 1044: 8, 1056: 8, 1059: 1, 1114: 8, 1161: 8, 1162: 8, 1163: 8, 1196: 8, 1222: 8, 1224:8, 1227: 8, 1235: 8, 1279: 8, 1552: 8, 1553: 8, 1556: 8, 1557: 8, 1561: 8, 1562: 8, 1568: 8, 1569: 8, 1570: 8, 1571: 8, 1572: 8, 1592: 8, 1596: 8, 1597: 8, 1600: 8, 1664: 8, 1779: 8, 1904: 8, 1912: 8, 1990: 8, 1998: 8, 2016: 8, 2017: 8, 2018: 8, 2019: 8, 2020: 8, 2021: 8, 2022: 8, 2023: 8, 2024: 8
   }],
}

STEER_THRESHOLD = 100

DBC = {
    CAR.SMART_ROADSTER_COUPE: dbc_dict('ocelot_can', None, 'ocelot_smart_roadster_pt'),
    CAR.ALBATROSS: dbc_dict('ocelot_controls', None, 'ford_transit_connect_2015'),
}

# addr: (bus, 1/freq*100, vl)
TSS2_RADAR_INIT = [
  (0x128, 1,   3, b'\xf4\x01\x90\x83\x00\x37'),
  (0x141, 1,   2, b'\x00\x00\x00\x46'),
  (0x160, 1,   7, b'\x00\x00\x08\x12\x01\x31\x9c\x51'),
  (0x161, 1,   7, b'\x00\x1e\x00\x00\x00\x80\x07'),
  (0x283, 2,   3, b'\x00\x00\x00\x00\x00\x00\x8c'),
  (0x344, 2,   5, b'\x00\x00\x01\x00\x00\x00\x00\x50'),
  (0x365, 2,  20, b'\x00\x00\x00\x80\xfc\x00\x08'),
  (0x366, 2,  20, b'\x00\x72\x07\xff\x09\xfe\x00'),
  (0x4CB, 2, 100, b'\x0c\x00\x00\x00\x00\x00\x00\x00'),
]

