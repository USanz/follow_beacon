from zenoh_flow.interfaces import Source
from zenoh_flow import Output
from zenoh_flow.types import Context
from typing import Any, Dict
import asyncio
import cv2, numpy as np

class SourceImage(Source):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        outputs: Dict[str, Output],
    ):
        configuration = {} if configuration is None else configuration
        self.cam_num = int(configuration.get("cam_num", 0))
        self.rate = int(configuration.get("rate", 15))
        self.quality = int(configuration.get("compression_quality", 90))
        self.img_resize_size = tuple(configuration.get("img_resize_size", [640, 480]))
        print("cam_num received: ", self.cam_num)
        print("quality received: ", self.rate)
        print("rate received: ", self.quality)
        print("img_resize_size received: ", self.img_resize_size)
        self.output = outputs.get("Image", None)

        self.camera = cv2.VideoCapture(self.cam_num)
        if (not self.camera.isOpened()): 
            print("Error opening video stream or file")

    def finalize(self) -> None:
        #It's giving me this error:
        #AttributeError("type object 'SourceImage' has no attribute 'camera'")
        self.camera.release()
        return None

    def produce_data(self) -> bytes:
        frame_grabbed, frame = self.camera.read()
        if frame_grabbed:
            return img_to_bytes(frame, 90, [640, 480])
            #return img_to_bytes(frame, self.quality, self.img_resize_size)
        else:
            return bytes()

    async def iteration(self) -> None:
        await asyncio.sleep(0.5)
        #await asyncio.sleep(1.0/self.rate)
        img = self.produce_data()
        if img != None:
            await self.output.send(img)
        return None

#def int_to_bytes(x: int) -> bytes:
#    return x.to_bytes((x.bit_length() + 7) // 8, "big")

def img_to_bytes(img: np.ndarray, quality: int, img_resize_size: tuple) -> bytes:
    resized_frame = cv2.resize(img, img_resize_size, interpolation = cv2.INTER_AREA)
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    result, encimg = cv2.imencode('.jpg', resized_frame, encode_param)
    return encimg.tobytes()


def register():
    return SourceImage
