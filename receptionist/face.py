#!/usr/bin/env python
# coding: UTF-8
# Created by Jun
# date: 2023/4/10

import cv2
from aip import AipFace
import pykinect_azure as pykinect
from io import BytesIO
import base64
from PIL import Image

class Face:
    def __init__(self):
        self.app_id = 'xxxxxxxxxxx'
        self.api_key = 'xxxxxxxxxxxx'
        self.secret_key = 'xxxxxxxxxxxxxxxxx'
        # 设置人脸库信息
        self.image_type = 'BASE64'
        self.group_id = 'xxxxxxxx'

        # 初始化百度AI人脸识别API
        self.client = AipFace(self.app_id, self.api_key, self.secret_key)

        # 设置百度AI人脸识别API参数
        self.options = {
            'max_face_num': 3,
            'face_field': 'age, beauty, expression, face_shape, gender, glasses, landmark, landmark150, quality, eye_status, emotion, face_type'
        }

    # 注册人脸
    def add_user(self, image, user_id):
        result = self.client.addUser(str(frame2base64(image), 'UTF-8'), self.image_type, self.group_id, user_id)
        return result['error_code'] == 0

    # 检测人脸
    def detect(self, image):
        result = self.client.detect(str(frame2base64(image), 'UTF-8'), self.image_type, self.options)
        if result['error_code'] == 0 and result['result']['face_num'] > 0:
            print(result)
            # return result['result']['face_list'][0]['face_token']
            return result
        else:
            return None

    # 搜索人脸
    def multi_search(self, image):
        result = self.client.multiSearch(str(frame2base64(image), 'UTF-8'), self.image_type,  self.group_id, self.options)
        if result['error_code'] == 0 and result['result']['face_num'] > 0:
            return result['result']['face_list'][0]['user_list'][0]['user_id']
        else:
            return None

    # 实时检测人脸
    def real_time_detect(self):
        pykinect.initialize_libraries()
        device_config = pykinect.default_configuration
        device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
        device = pykinect.start_device(config=device_config)
        cv2.namedWindow('Color Frame', cv2.WINDOW_NORMAL)
        while True:
            capture = device.update()
            ret, frame = capture.get_color_image()
            if not ret:
                continue
            cv2.imshow("Color Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # 检测人脸
            # face_token = self.detect(frame)
            face_data = self.detect(frame)
            print(face_data)
            face_token = face_data['result']['face_list'][0]['face_token']
            if face_token is not None:
                # 搜索人脸所属的用户
                user_id = self.multi_search(frame)
                if user_id is not None:
                    print('Detected user:', user_id)
                    break
                else:
                    print('No detected')
                    user_id = 'false'

        return user_id

# image转码
def frame2base64(frame):
    img = Image.fromarray(frame)
    output_buffer = BytesIO()  # 创建一个BytesIO
    img.save(output_buffer, format='JPEG')  # 写入output_buffer
    byte_data = output_buffer.getvalue()  # 在内存中读取
    base64_data = base64.b64encode(byte_data)  # 转为BASE64
    return base64_data  # 转码成功 返回base64编码

def main():
    # 创建Face对象
    face = Face()
    # 实时检测人脸
    face.real_time_detect()
    # user_id = face.real_time_detect()
    # print(user_id)

if __name__ == "__main__":
    main()
