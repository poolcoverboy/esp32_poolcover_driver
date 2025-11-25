import os
import re
from SCons.Script import Import

Import("env")

HEADER_PATH = os.path.join(env["PROJECT_INCLUDE_DIR"], "ota_config.h")
PASSWORD_PATTERN = re.compile(r'#define\s+OTA_PASSWORD\s+"([^"]+)"')


def read_password():
  if not os.path.exists(HEADER_PATH):
    print("Warning: ota_config.h not found; OTA password upload flag not set.")
    return None
  with open(HEADER_PATH, "r", encoding="utf-8") as header:
    contents = header.read()
  match = PASSWORD_PATTERN.search(contents)
  if not match:
    print("Warning: OTA_PASSWORD define missing; OTA password upload flag not set.")
    return None
  return match.group(1)


password = read_password()
if password:
  env.Append(UPLOADERFLAGS=["--auth=%s" % password])
  print("Using OTA password from ota_config.h for espota uploads.")
