import asyncio
import time
import websockets

data = bytearray([0,89,202])
arr = []

async def hello():
    async with websockets.connect("ws://192.168.1.173/") as websocket:
        for i in range(25):
            data[1] = i
            start = time.time()
            await websocket.send(data)
            end = time.time()
            arr.append(end-start)

t1 = time.time()
asyncio.run(hello())
t2 = time.time()
print("First Connection Time: ", t2-t1-sum(arr))
print("Avg: ", sum(arr)/len(arr))
