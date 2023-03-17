from zenoh_flow.interfaces import Source
from zenoh_flow import Output
from zenoh_flow.types import Context
from typing import Any, Dict
import asyncio
import random

actions_dict = {
    "w": 1, #forward
    "a": 2, #left
    "x": 3, #backwards
    "d": 4, #right
    "s": 5  #stop
}

class SourceImage(Source):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        outputs: Dict[str, Output],
    ):
        configuration = {} if configuration is None else configuration
        #self.value = int(configuration.get("value", 0))
        self.output = outputs.get("Image", None)

    def finalize(self) -> None:
        return None

    def produce_data(self) -> bytes:
        #self.value += 1
        self.act = random.randint(1, 5) # corresponding to the actions_dict
        #print(f"Sending {self.value}: {self.act}")
        return int_to_bytes(self.act)
    
        #return int_to_bytes(self.value)

    async def iteration(self) -> None:
        await asyncio.sleep(0.5)
        await self.output.send(self.produce_data())
        return None


def int_to_bytes(x: int) -> bytes:
    return x.to_bytes((x.bit_length() + 7) // 8, "big")


def register():
    return SourceImage
