import socket
import asyncio
import json

from rover_common import aiolcm
from rover_common.aiohelper import run_coroutines
from rover_msgs import ArmPosition

lcm_ = aiolcm.AsyncLCM()


async def listen_blender():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('0.0.0.0', 8019))
    sock.listen(5)
    sock.settimeout(0.1)

    conn = None
    while conn is None:
        try:
            conn, addr = sock.accept()
        except socket.timeout:
            await asyncio.sleep(0.1)

    while True:
        try:
            rawData = str(conn.recv(1024)).replace('\'', '"')
            rawData = rawData[2:len(rawData)-1]
            data = json.loads(rawData)

            # Send data to onboard rover
            msg = ArmPosition()
            msg.joint_a = data['A']
            msg.joint_b = data['B']
            msg.joint_c = data['C']
            msg.joint_d = data['D']
            msg.joint_e = data['E']

            lcm_.publish('/ik_ra_control', msg.encode())

        except socket.timeout:
            await asyncio.sleep(0.1)


def main():
    run_coroutines(lcm_.loop(), listen_blender())
