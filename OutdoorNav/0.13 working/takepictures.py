import requests
from datetime import datetime
import cv2 as cv
import requests
import numpy as np


AXIS_HOST = "192.168.131.10"
AXIS_SNAPSHOT_API_URL = "http://" + AXIS_HOST + "/axis-cgi/jpg/image.cgi"
SAVE_IMAGE_DIRECTORY= "/home/administrator/nathan_ws/pictures/"
FLIR_URL = "http://192.168.131.11/api/image/current?imgformat=JPEG"


def captureSnapshot():
    resp = requests.get(AXIS_SNAPSHOT_API_URL, stream=True).raw
    image = np.asarray(bytearray(resp.read()), dtype="uint8")
    image = cv.imdecode(image, cv.IMREAD_COLOR)
    return image

def saveImageQ62():
    file_name = str(datetime.now()) + ".jpg"
    img = captureSnapshot()
    dir = SAVE_IMAGE_DIRECTORY
    img_path = dir + file_name

    print("Saving file {}".format(img_path))
    cv.imwrite(img_path, img)

def captureThermalImg():
    response = requests.get(FLIR_URL)
    file_name = str(datetime.now()) + "_thermal.jpg"
    img_path = SAVE_IMAGE_DIRECTORY + file_name
    print(f"   Saving thermal file {img_path}")
    with open(img_path, 'wb') as f:
        f.write(response.content)

def main():
    saveImageQ62()
    captureThermalImg
    
if __name__ == '__main__':
    main()

