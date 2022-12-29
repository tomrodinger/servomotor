#!/usr/bin/env python3

import json

with open('motor_commands.json') as f:
    data = json.load(f)

for item in data:
    print(item)

