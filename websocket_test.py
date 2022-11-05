# import asyncio
# import time
# import websockets

# arr = []

# async def hello():
#     async with websockets.connect("ws://192.168.1.173/") as websocket:
#         for i in range(61):
#             data = bytearray([0,i,70,0,i,70,0,i,70,0,i,70,0,i,70,0,i,70])
#             start = time.time()
#             await websocket.send(data)
#             time.sleep(0.1)
#             end = time.time()
#             arr.append(end-start)

# t1 = time.time()
# asyncio.run(hello())
# t2 = time.time()
# print("First Connection Time: ", t2-t1-sum(arr))
# print("Avg: ", sum(arr)/len(arr))

import time
import requests

t1 = time.time()
s = requests.Session()
s.headers.update({'Connection': 'Keep-Alive', 'Keep-Alive': "timeout=5, max=50000"})
t2 = time.time()
arr = []

for i in range(1000):
    start = time.time()
    url = "http://172.26.171.164"
    payload = {'dtype': 'speed', 'servo1': i % 180, "servo2": i % 180}
    s.post(url, data=payload)
    end = time.time()
    arr.append(end-start)
print("First Connection Time: ", t2-t1)
print("Avg: ", sum(arr)/len(arr))
print("Max: ", max(arr))
print("Min: ", min(arr))
