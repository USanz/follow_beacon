from zenoh_flow.interfaces import Source
from zenoh_flow import Output
from zenoh_flow.types import Context
from typing import Any, Dict
import asyncio
import cv2
import numpy as np



class SourceImage(Source):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        outputs: Dict[str, Output],
    ):
        self.output = outputs.get("Image", None)

        configuration = {} if configuration is None else configuration
        self.rate = float(configuration.get("rate", 15))
        self.img_resize_size = tuple(configuration.get("img_resize_size", []))
        self.quality = int(configuration.get("compression_quality", 90))
        self.cam_num = int(configuration.get("cam_num", 0))
        self.camera = cv2.VideoCapture(self.cam_num)
        if (not self.camera.isOpened()):
            raise Exception("Error opening video stream or file")

    def produce_data(self) -> bytes:
        # Get frame:
        frame_grabbed, frame = self.camera.read()

        if not frame_grabbed:
            return bytes()
        
        # Resize it
        if self.img_resize_size:
            resized_frame = cv2.resize(frame, self.img_resize_size, interpolation = cv2.INTER_AREA)
        else:
            resized_frame = frame
        # Compress it
        compressed_frame = img_compress(resized_frame, '.jpg', self.quality)
        # Serialize it
        return compressed_frame.tobytes()

    async def iteration(self) -> None:
        await asyncio.sleep(1.0/self.rate)
        img = self.produce_data()
        if img != None:
            await self.output.send(img)
        return None

    def finalize(self) -> None:
        self.camera.release()
        return None


def img_compress(img: np.ndarray, format: str, quality: int) -> np.ndarray:
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
    ret_val, enc_img = cv2.imencode(format, img, encode_param)
    if not ret_val:
        return np.empty(0, dtype=np.uint8)
    return enc_img


def register():
    return SourceImage
