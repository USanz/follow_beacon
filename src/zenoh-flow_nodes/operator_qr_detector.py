from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

actions_dict = {
    "w": 1, #forward
    "a": 2, #left
    "x": 3, #backwards
    "d": 4, #right
    "s": 5  #stop
}

class OperatorQRDetector(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        print(f"Context: {context}")
        self.in_stream = inputs.get("Image", None)
        self.output = outputs.get("QR_data", None)

    def finalize(self) -> None:
        return None

    async def iteration(self) -> None:
        # in order to wait on multiple input streams use:
        # https://docs.python.org/3/library/asyncio-task.html#asyncio.gather
        # or
        # https://docs.python.org/3/library/asyncio-task.html#asyncio.wait
        data_msg = await self.in_stream.recv()
        new_value = int_from_bytes(data_msg.data)

        #new_key_string = list(actions_dict.keys())[new_value - 1]
        
        #my_bytes_value = b'[{"Date": "2016-05-21T21:35:40Z", "CreationDate": "2012-05-05", "Classe": ["Email addresses", "Passwords"]}]'

        #my_json = json.load(my_bytes_value.decode('utf-8'))


        json_str = """{
            "Name": "Jennifer Smith",
            "Contact Number": 7867567898,
            "Email": "jen123@gmail.com",
            "Hobbies":["Reading", "Sketching", "Horse Riding"]
            }"""

        #await self.output.send(str_to_bytes(new_key_string))
        await self.output.send(str_to_bytes(json_str))
        return None


def int_to_bytes(x: int) -> bytes:
    return x.to_bytes((x.bit_length() + 7) // 8, "big")


def int_from_bytes(xbytes: bytes) -> int:
    return int.from_bytes(xbytes, "big")


def str_to_bytes(string: str) -> bytes:
    return bytes(string, 'utf-8')

def register():
    return OperatorQRDetector
