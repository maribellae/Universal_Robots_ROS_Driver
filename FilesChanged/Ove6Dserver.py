import os
import cv2
import sys
import PIL
import json
import math
import time
import torch
import warnings
import numpy as np
from PIL import Image, ImageDraw
from pathlib import Path
import matplotlib
from matplotlib import pyplot as plt
import torchvision
import detectron2 
from detectron2 import model_zoo
from detectron2.config import get_cfg
from detectron2.engine import DefaultPredictor
from os.path import join as pjoin
from bop_toolkit.bop_toolkit_lib import inout
warnings.filterwarnings("ignore")

base_path = os.path.dirname(os.path.abspath("."))
sys.path.append(base_path)

from lib import rendering, network

from dataset import LineMOD_Dataset
from evaluation import utils
from evaluation import config as cfg

gpu_id = 0

os.environ["CUDA_DEVICE_ORDER"]="PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = str(gpu_id)
os.environ['EGL_DEVICE_ID'] = str(gpu_id)
DEVICE = torch.device('cuda')
cfg.HEMI_ONLY = True 
#########   FOR RCNN   ######### 
rcnnIdx_to_lmIds_dict = {0:1, 1:2, 2:3, 3:4, 4:5, 5:6, 6:7}
rcnnIdx_to_lmCats_dict ={0:'Flask-0', 1:'Glassbeaker-1', 2:'Graduatedcylinder-2', 3:'Pipette-3', 4:'Testtube-4', 5:'Tuberackthin-5', 6:'Tuberackwide-6'}
rcnn_cfg = get_cfg()
rcnn_cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
rcnn_cfg.MODEL.WEIGHTS = os.path.abspath(os.path.join(base_path, 'OVE6D-pose','model_own','model_0004999.pth'))
rcnn_cfg.MODEL.ROI_HEADS.NUM_CLASSES = len(rcnnIdx_to_lmCats_dict)
rcnn_cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.01 #0.5 # the predicted category scores
predictor = DefaultPredictor(rcnn_cfg)

#########   FOR OVE6D   ######### 
ckpt_file = pjoin(base_path, 'OVE6D-pose',
                'checkpoints', 
                "OVE6D_pose_model.pth"
                )
model_net = network.OVE6D().to(DEVICE)

model_net.load_state_dict(torch.load(ckpt_file))
model_net.eval()

cfg.ZOOM_DIST_FACTOR = 0.01
cfg.MODEL_SCALING=1

#########   FOR OVE6D'S CODEBOOKS   #########
cfg.VIEWBOOK_BATCHSIZE = 100 # reduce this if out of GPU memory, 
codebook_saving_dir = pjoin(base_path, 'OVE6D-pose','evaluation1/object_codebooks',
                            cfg.DATASET_NAME, 
                            'zoom_{}'.format(cfg.ZOOM_DIST_FACTOR), 
                            'views_{}'.format(str(4000)))

object_codebooks = utils.OVE6D_codebook_generation(codebook_dir=codebook_saving_dir, 
                                                    model_func=model_net,
                                                    dataset=eval_dataset, 
                                                    config=cfg, 
                                                    device=DEVICE)


Raw_bb =[]
Raw_shape=[]

def VisualAllLoop(view_id, color_file,depth_file):


    #test_data_dir = datapath / 'MixedAll' / 'test'          # path to the test dataset of BOP

    for t in range(7):
        obj_renderer = rendering.Renderer(width=cfg.RENDER_WIDTH, height=cfg.RENDER_HEIGHT)
        scene_id = t+1 # range [1, 15]

        tar_obj_id = scene_id # object id equals the scene id for LM dataset
        tar_obj_codebook = object_codebooks[tar_obj_id]

        ############## read the camera information ##############
        '''cam_info_file = pjoin(scene_dir, 'scene_camera.json')
        with open(cam_info_file, 'r') as cam_f:
            scene_camera_info = json.load(cam_f)
        view_cam_info = scene_camera_info[str(view_id)]  # scene camera information        '''


        ############## read the depth images and covert it from meter to millimeter ##############

        view_depth = torch.tensor(depth_file, dtype=torch.float32)
        view_depth *= view_cam_info['depth_scale']
        #print(view_depth) 
        view_depth *= cfg.MODEL_SCALING # convert to meter scale from millimeter scale
        view_camK = torch.tensor(view_cam_info['cam_K'], dtype=torch.float32).view(3, 3)[None, ...] # 1x3x3
        cam_K = view_camK.to(DEVICE)
        #print(cam_K)
        view_depth = view_depth.to(DEVICE)

        ############## read rgb image for object segmentation ##############
        output = predictor(color_file)
      
        rcnn_pred_ids = output["instances"].pred_classes
        rcnn_pred_masks = output["instances"].pred_masks
        rcnn_pred_scores = output["instances"].scores

        ###################### object segmentation ######################

        tar_rcnn_d = tar_obj_id - 1
        obj_masks = rcnn_pred_masks # NxHxW
      
        obj_depths = view_depth[None, ...] * obj_masks
     
        tar_obj_depths = obj_depths[tar_rcnn_d==rcnn_pred_ids]
        tar_obj_masks = rcnn_pred_masks[tar_rcnn_d==rcnn_pred_ids]
        tar_obj_scores = rcnn_pred_scores[tar_rcnn_d==rcnn_pred_ids]
        
        mask_pixel_count = tar_obj_masks.view(tar_obj_masks.size(0), -1).sum(dim=1)
  
        valid_idx = (mask_pixel_count >= 100)                                              #?
        if valid_idx.sum() == 0:
            mask_visib_ratio = mask_pixel_count / mask_pixel_count.max()
            valid_idx = mask_visib_ratio >= 0.05

        tar_obj_masks = tar_obj_masks[valid_idx] # select the target object instance masks
        tar_obj_depths = tar_obj_depths[valid_idx]
        tar_obj_scores = tar_obj_scores[valid_idx]
  
        pose_ret, rcnn_idx = utils.OVE6D_rcnn_full_pose(model_func=model_net, 
                                            obj_depths=tar_obj_depths,
                                            obj_masks=tar_obj_masks,
                                            obj_rcnn_scores=tar_obj_scores,
                                            obj_codebook=tar_obj_codebook, 
                                            cam_K=cam_K,
                                            config=cfg, 
                                            device=DEVICE,
                                            obj_renderer=obj_renderer, 
                                            return_rcnn_idx=True
                                            )
        del obj_renderer
       
        raw_pose_R = pose_ret['icp1_R'] # without ICP
        raw_pose_t = pose_ret['icp1_t'] # without ICP

        return (raw_pose_R,  raw_pose_t)
        




