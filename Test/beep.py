import requests
from ftplib import FTP
import winsound
import time

#you dont have to understand this code:
def readInt():
	res = requests.get("http://robocup.evb-gymnasium.de/int.bin", stream=True)
	return int.from_bytes(res.raw.read(1), byteorder="big")


i = 0
while i == 0:
	print("Waiting for number...")
	time.sleep(2)
	i = readInt()


frequency = 800  # Set Frequency To 600 Hertz
duration = 1000 * i #calculate how long the beep should be

print(f"Beeping for {i}s")
winsound.Beep(frequency, duration) #beep
print("Beep finished")
