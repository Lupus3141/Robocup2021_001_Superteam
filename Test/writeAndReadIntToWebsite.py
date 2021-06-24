import requests
from ftplib import FTP

def writeInt(i):
	#create temp file and write int to it
	with open("temp", "wb") as f:
		f.write(i.to_bytes(1, byteorder="big"))

	#upload file to website
	ftp = FTP("w007d83c.kasserver.com")
	ftp.login(user="f01406d9", passwd="lukaskraemer")
	
	with open("temp", "rb") as f:
		ftp.storbinary(f"STOR int.bin", f)
	ftp.close()

def readInt():
	res = requests.get("http://robocup.evb-gymnasium.de/int.bin", stream=True)
	return int.from_bytes(res.raw.read(1), byteorder="big")

i = int(input("Number: "))

writeInt(i)

print(readInt())