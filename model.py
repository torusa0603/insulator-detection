# -*- coding: utf-8 -*-
"""
物体検出モデルの推論クラス
"""
import time
import numpy as np
import cv2

import onnxruntime as ort
from PIL import Image

class Model():
    def __init__(self, names, colors, height, width, model_path):
        # モデル読み込み
        self.__ort = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])

    def inference(self, frame, threshold):

        # ikeya
        labels = ["isn_t", "ins_t_b"]
        min_width, min_height = 640, 640
        height, width = frame.shape[0], frame.shape[1]
        frame_shape = frame.shape

        def cv2pil(img_cv):
            im_rgb = cv2.cvtColor(img_cv, cv2.COLOR_BGR2RGB)
            return im_rgb
        
        def draw_detection(frame, shape, box):
            xc, yc, w, h = box[:4]
            left = (xc-w/2)/min_width*width
            top = (yc-h/2)/min_height*height
            right = (xc+w/2)/min_width*width
            bottom = (yc+h/2)/min_height*height
            prob_t = box[4]
            prob_t_b = box[5]

            if prob_t >= threshold:
                cv2.rectangle(img=frame, pt1=(int(left), int(top)), pt2=(int(right), int(bottom)), color=(0, 255, 0), thickness=1)
            if prob_t_b >= threshold:
                cv2.rectangle(img=frame, pt1=(int(left), int(top)), pt2=(int(right), int(bottom)), color=(255, 0, 0), thickness=1)
            return [int(xc/min_width*width), int(yc/min_height*height), int(w/min_width*width), int(h/min_height*height)]
    
        onnx_frame = cv2.medianBlur(frame, 1)
        onnx_image = cv2pil(cv2.resize(onnx_frame, (min_width, min_height)))
        input = np.array(onnx_image).transpose(2,0,1).reshape(1,3,min_width,min_height)/255.0
        input = input.astype(np.float32)
        input_name = self.__ort.get_inputs()[0].name
        output_name = self.__ort.get_outputs()[0].name
        
        # 推論 
        start = time.time()
        
        outputs = self.__ort.run([output_name], {input_name: input})

        output_boxes = outputs[0][0].transpose()
        output_boxes = np.delete(output_boxes, np.all(output_boxes[:, 4:] < threshold , axis=1), axis=0)
        ret_prob_pix = []

        for box in output_boxes:
            ret_prob_pix.append(draw_detection(onnx_frame, frame_shape, box))

        elapsed_time = time.time() - start

        return onnx_frame, ret_prob_pix
