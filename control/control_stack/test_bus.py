#!/usr/bin/env python3
"""Verify the cereal message bus works on the Jetson: publish modelV2, receive it."""
import time
import openpilot.cereal.messaging as messaging

# publisher for modelV2, subscriber for it
pm = messaging.PubMaster(['modelV2'])
sm = messaging.SubMaster(['modelV2'])
time.sleep(0.5)

# build a modelV2 message
msg = messaging.new_message('modelV2')
md = msg.modelV2
# fill a couple of fields to prove structured data survives the bus
md.frameId = 42

pm.send('modelV2', msg)
time.sleep(0.2)
sm.update(100)

print("modelV2 received:", sm.updated['modelV2'])
print("frameId round-trip:", sm['modelV2'].frameId)
print("BUS OK" if sm['modelV2'].frameId == 42 else "BUS FAIL")
