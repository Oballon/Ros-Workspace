import os
import sys
import time
import numpy as np

sys.path.append(os.path.dirname(__file__) + "/../")

from scipy.misc import imread, imsave

from config import load_config
from dataset.factory import create as create_dataset
from nnet import predict
from util import visualize
from dataset.pose_dataset import data_to_input

from multiperson.detections import extract_detections
from multiperson.predict import SpatialModel, eval_graph, get_person_conf_multicut
from multiperson.visualize import PersonDraw, visualize_detections

import matplotlib.pyplot as plt


cfg = load_config("demo/pose_cfg_multi.yaml")

dataset = create_dataset(cfg)

sm = SpatialModel(cfg)
sm.load()

draw_multi = PersonDraw()

# Load and setup CNN part detector
sess, inputs, outputs = predict.setup_pose_prediction(cfg)

# Read image from file
file_name = "/home/randoms/Pictures/sports1.jpeg"
image = imread(file_name, mode='RGB')

image_batch = data_to_input(image)
# Compute prediction with the CNN
outputs_np = sess.run(outputs, feed_dict={inputs: image_batch})
scmap, locref, pairwise_diff = predict.extract_cnn_output(
    outputs_np, cfg, dataset.pairwise_stats)

detections = extract_detections(cfg, scmap, locref, pairwise_diff)
unLab, pos_array, unary_array, pwidx_array, pw_array = eval_graph(
    sm, detections)
person_conf_multi = get_person_conf_multicut(sm, unLab, unary_array, pos_array)

num_people = person_conf_multi.shape[0]
conf_min_count = 3  # 至少三个点才被认为能够接受
multi_body_rect = []
for pidx in range(num_people):
    if np.sum(person_conf_multi[pidx, :, 0] > 0) < conf_min_count:
        continue
    body_rect = person_conf_multi[pidx, 0:4].tolist(
    ) + person_conf_multi[pidx, 5:6].tolist() + person_conf_multi[pidx, 11:12].tolist()
    body_rect = list(filter(lambda x: x[0] > 0 and x[1] > 0,  body_rect))
    if len(body_rect) < 3:
        continue
    multi_body_rect.append(body_rect)
print(multi_body_rect)
