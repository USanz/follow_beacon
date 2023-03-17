from zenoh_flow.interfaces import Sink
from zenoh_flow import Input
from zenoh_flow.types import Context
from typing import Dict, Any
import asyncio
import json

class SinkMotors(Sink):
    def finalize(self):
        return None

    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
    ):
        self.input = inputs.get("QR_Data", None)

    async def iteration(self) -> None:
        data_msg = await self.input.recv()

        dict_result = json.loads(data_msg.data) # json.loads() takes string, bytes or byte array.
        print(f"Received {dict_result['Hobbies']}")
        #print(f"Received {str_from_bytes(data_msg.data)}")


def int_from_bytes(xbytes: bytes) -> int:
    return int.from_bytes(xbytes, "big")

def str_from_bytes(xbytes: bytes) -> str:
    return xbytes.decode('utf-8')

def register():
    return SinkMotors
