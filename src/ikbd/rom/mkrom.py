#!/usr/bin/env python3
# regenerate MCU_BIROM.v (case table) from ikbd.hex; run from src/
import re
src = open('ikbd/rom/MCU_BIROM.v').read()
head = src[:src.index("        case (AD)") + len("        case (AD)\n")]
tail = src[src.index("          default:"):]
b = [int(x, 16) for x in open('ikbd/rom/ikbd.hex').read().split()]
assert len(b) == 4096
body = "".join("          12'h%03x: DO <= 8'h%02x;\n" % (i, v) for i, v in enumerate(b))
open('ikbd/rom/MCU_BIROM.v', 'w').write(head + body + tail)
