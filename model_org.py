# -*- coding: utf-8 -*-
"""
物体検出モデルの推論クラス
"""
import time
import numpy as np
import os
import cv2
import mxnet as mx
import sys, os
import numpy as np
from collections import namedtuple

class Model():
    def __init__(self, names, colors, height, width, model_path):

        # モデル設定
        self.__names = names
        self.__colors = colors
        self.__shape = 512
        self.__h_magnification = self.__shape/height
        self.__w_magnification = self.__shape/width
        print("magnification H:{} W:{}".format(1/self.__h_magnification, 1/self.__w_magnification))

        # モデル読み込み
        input_shapes=[('data', (1, 3, self.__shape, self.__shape))]
        self.__Batch = namedtuple('Batch', ['data'])
        sym, arg_params, aux_params = mx.model.load_checkpoint(model_path, 0)
        self.__mod = mx.mod.Module(symbol=sym, label_names=[], context=mx.cpu())
        self.__mod.bind(for_training=False, data_shapes=input_shapes)
        self.__mod.set_params(arg_params, aux_params)

    def inference(self, frame):

        # 入力インターフェースへの画像変換
        img = cv2.resize(frame, dsize=(self.__shape, self.__shape)) # height * width => 512 * 512
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # BGR => RGB
        img = img.transpose((2, 0, 1)) # 512,512,3 => 3,512,512
        img = img[np.newaxis, :] # 3,512,512 => 1,3,512,512
        print("img.shape: {}".format(img.shape))

        # 推論
        start = time.time()

        # 映像入力および推論実行
        self.__mod.forward(self.__Batch([mx.nd.array(img)]))
        prob = self.__mod.get_outputs()[0].asnumpy()
        prob = np.squeeze(prob)

        elapsed_time = time.time() - start
        print("{} [Sec]".format(elapsed_time))

        result = []
        for det in prob:
            if(det[0]==-1):
                continue
            index = int(det[0])
            confidence = det[1]
            # 検出座標（縮尺に合わせて座標を変換する）
            x1 = int(det[2] * self.__shape /self.__w_magnification)
            y1 = int(det[3] * self.__shape /self.__h_magnification)
            x2 = int(det[4] * self.__shape /self.__w_magnification)
            y2 = int(det[5] * self.__shape /self.__h_magnification)
            print("[{}] {:.1f} {}, {}, {}, {}".format(self.__names[index], confidence, x1, y1, x2, y2))
#             if(confidence > 0.3):
            # if(confidence > 0.3):
            # if(confidence > 0.2):
            # if(confidence > 0.3): #20220301
            # if(confidence > 0.2): #20220310
            if(confidence > 0.1): #20220330
                frame = cv2.rectangle(frame,(x1, y1), (x2, y2), self.__colors[index],2)
                frame = cv2.rectangle(frame,(x1, y1), (x1 + 150,y1-20), self.__colors[index], -1)
                label = "{} {:.2f}".format(self.__names[index], confidence)
                frame = cv2.putText(frame,label,(x1+2, y1-2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
                result.append(self.__names[index])
        return (result, frame, prob)