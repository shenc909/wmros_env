#!/usr/bin/env python
import sys

sys.path.append('../src')
from worldmodel_bullet.SingleRacecar import SingleRacecar
import cv2

sr = SingleRacecar(render_mode='human')

img = sr.reset()

cv2.imshow('image', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
cv2.waitKey(0)