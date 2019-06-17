from os import listdir, makedirs
from os.path import isfile, join, dirname, realpath, isdir
import json
import numpy as np
from models import Frame
import tensorflow as tf
import math

def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / math.sqrt(np.dot(axis, axis))
    a = math.cos(theta / 2.0)
    b, c, d = -axis * math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])


def extractBB(label,gt=True):
    label_split = label.split(" ")
    if gt:
        return label_split[0],float(label_split[-7]),float(label_split[-6]),float(label_split[-5]),float(label_split[-4]),float(label_split[-3]),float(label_split[-2]),float(label_split[-1])
    else:
        return label_split[0],float(label_split[-8]),float(label_split[-7]),float(label_split[-6]),float(label_split[-5]),float(label_split[-4]),float(label_split[-3]),float(label_split[-2])

def extractscore(label):
    label_split = label.split(" ")
    return label_split[0],float(label_split[-1])

def extractscoreonly(label):
    label_split = label.split(" ")
    print(label_split[-1])
    return float(label_split[-1])

def getCorners(height,width,length,x,y,z,θ,rotation=True):
    
    corners = np.array([[-length / 2, -length / 2, length / 2, length / 2, -length / 2, -length / 2, length / 2, length / 2],
                        [width / 2, -width / 2, -width / 2, width / 2, width / 2, -width / 2, -width / 2, width / 2],
                        [0, 0, 0, 0, height, height, height, height]])
    
    rotMat = np.array([[np.cos(θ) , -np.sin(θ) , 0],
                       [np.sin(θ) ,  np.cos(θ) , 0],
                       [    0     ,     0      , 1]])
    if rotation:
        cornersPos = (np.dot(rotMat,corners)+np.tile([x,y,z],(8,1)).T).transpose()
        corner1,corner2,corner3,corner4,corner5,corner6,corner7,corner8 = cornersPos[0],cornersPos[1],cornersPos[2],cornersPos[3],cornersPos[4],cornersPos[5],cornersPos[6],cornersPos[7]
    else:
        cornersPos = (corners + np.tile([x,y,z],(8,1)).T).transpose()
        corner1,corner2,corner3,corner4,corner5,corner6,corner7,corner8 = cornersPos[0],cornersPos[1],cornersPos[2],cornersPos[3],cornersPos[4],cornersPos[5],cornersPos[6],cornersPos[7]
    
    return list(corner1),list(corner2),list(corner3),list(corner4),list(corner5),list(corner6),list(corner7),list(corner8)

def createBBox(bounding_box,C1,C2,C3,C4,C5,C6,C7,C8,color="yellow"):
    bounding_box.append(
        {
            "color":color,
            "vertices":[C1,C2,C3,C4,C1]
        })
    bounding_box.append(
        {
            "color":color,
            "vertices":[C1,C4,C8,C5,C1]
        })
    bounding_box.append(
        {
            "color":color,
            "vertices":[C1,C2,C6,C5,C1]
        })
    bounding_box.append(
        {
            "color":color,
            "vertices":[C2,C6,C7,C3,C2]
        })
    bounding_box.append(
        {
            "color":color,
            "vertices":[C3,C7,C8,C4,C3]
        })
    return bounding_box


class FrameHandler():
	CUR_DIR =dirname(realpath(__file__))
	DATASET_DIR = join(CUR_DIR, "test_dataset")
	INPUT_BIN_DIR = "bin_data"
	GROUND_REMOVED_DIR = "ground_removed"
	OUTPUT_ANN_DIR = join(CUR_DIR, "output")
	POINTRCCN_PREDS = "eval"
	VELODYNE_HEIGHT = 1.73
	KITTI_PATH = join(CUR_DIR, "test_dataset/PointRCNN_tracking/bin_data/") 
	PointRCNN_PATH = join(CUR_DIR, "test_dataset/PointRCNN_tracking/eval/")
	PC_ID = '%06d' % 102 # [0 : 7480] 
	# points = io_bin.read_bin(KITTI_PATH+"velodyne/"+PC_ID+".bin",shape=(-1,4))['points']
	# points['z'] += VELODYNE_HEIGHT

	def __init__(self):
		self.drives = dict()
		# self.bin_data  = sorted([f for f in listdir(self.INPUT_BIN_DIR) 
		# 					if isfile(join(self.INPUT_BIN_DIR, f)) and '.bin' in f])
		# self.image_data  = sorted([f for f in listdir(self.INPUT_IMAGE_DIR) 
		# 					if isfile(join(self.INPUT_IMAGE_DIR, f))])
		# self.frame_names = [f.split(".")[0] for f in self.bin_data]
		for drive in listdir(self.DATASET_DIR):
			if 'sync' in drive or 'Point' in drive:
				bin_dir = join(self.DATASET_DIR, drive, self.INPUT_BIN_DIR)
				self.drives[drive] = []
				for f in listdir(bin_dir):
					if isfile(join(bin_dir, f)) and '.bin' in f:
						self.drives[drive].append(f.split('.bin')[0])
				self.drives[drive] = sorted(self.drives[drive])

	def get_frame_names(self):
		"""
		Get all the frame names
		"""
		# return ",".join(self.frame_names)
		return str(self.drives)

	def get_pointcloud(self, drivename, fname, dtype=str, ground_removed=False):
		"""
		Gets point cloud as list of floats

		Input:
		- fname: Frame name. Can have file extension. 

		Returns a string of comma-separated floats. The number of floats 
		is 4N, where N is the number of points in the point cloud. 
		Each point is represented by 4 numbers - the x, y, z coordinates 
		as well as the intensity.
		"""
		bin_dir = join(self.DATASET_DIR, drivename, self.INPUT_BIN_DIR)
		filename = join(bin_dir, fname.split(".")[0] + ".bin")
		self.PC_ID = fname.split(".")[0]
		data = np.fromfile(filename, dtype=np.float32)
		if ground_removed:
			filename = join(self.DATASET_DIR, drivename, self.GROUND_REMOVED_DIR, fname.split(".")[0] + ".bin")
			data = np.fromfile(filename, dtype=np.float32)	
		if dtype == str:
			data = data.flatten(order="C").tolist()
			data_str = (",").join([str(x) for x in data])
			return data_str
		else:
			if ground_removed:
				return data.reshape((-1,4))
			else:
				return data.reshape((-1,4))[:,:3]

	def load_annotation(self, drivename, fname, dtype='object'):
		fname = fname.split('.')[0] + '.json'
		try:
			with open(join(self.OUTPUT_ANN_DIR, drivename, fname), "r") as read_file:
				print("file: ", read_file)
				try:
					frame = json.load(read_file)
					if dtype == 'object':
						return Frame.parse_json(frame)
					else:
						return frame
				except json.JSONDecodeError:
					return ""
		except:
			return ""

	def generate_pred_boxes(self):
	    from pyntcloud import PyntCloud
	    from pyntcloud.io import bin as io_bin
	    import numpy as np
	    import pandas as pd
	    import os
	    import numpy as np
	    import json

	    if os.path.exists(self.KITTI_PATH+self.PC_ID+".txt"):
	        pass
	    else:
	        print("File "+self.KITTI_PATH+self.PC_ID+".txt doesn't exist !")
	        
	    
	    file_label_gt = open(self.KITTI_PATH+self.PC_ID+".txt","r")
	    labels_gt = file_label_gt.readlines()
	    file_label_gt.close()

	    if os.path.exists(self.PointRCNN_PATH+self.PC_ID+".txt"):
	        pass
	    else: 
	        print("File "+self.PointRCNN_PATH+self.PC_ID+".txt doesn't exist !")
	        
	    
	    file_label_pred = open(self.PointRCNN_PATH+self.PC_ID+".txt","r")
	    labels_pred = file_label_pred.readlines()
	    file_label_pred.close()

	    labels_clean_gt = []
	    for i,label in enumerate(labels_gt):
	        labels_gt[i]=label[:-2]
	        if labels_gt[i].split(" ")[0] in ["Car", "Van", "Truck"]:
	            labels_clean_gt.append(extractBB(labels_gt[i],gt=True))

	    labels_clean_pred = []
	    for i,label in enumerate(labels_pred):
	        labels_pred[i]=label[:-2]
	        if labels_pred[i].split(" ")[0] == "Car":
	            labels_clean_pred.append(extractBB(labels_pred[i],gt=False))

	    bounding_box = []
	    colors = {"GT":'red',"Pred":"blue"}

	    boxes_gt = list()
	    for label in labels_clean_gt: 
	        object_type,height,width,length,x_tmp,y_tmp,z_tmp,θ = label
	        x,y,z,θ = z_tmp,-x_tmp,y_tmp-self.VELODYNE_HEIGHT,np.pi/2-θ
	        C1,C2,C3,C4,C5,C6,C7,C8 = getCorners(height,width,length,x,y,z,θ,rotation=True)
	        bounding_box = createBBox(bounding_box,C1,C2,C3,C4,C5,C6,C7,C8,colors["GT"])
	        boxes_gt.append(list((getCorners(height,width,length,x,y,z,θ,rotation=True))))

	    boxes_pred = list()
	    for label in labels_clean_pred: 
	        object_type,height,width,length,x_tmp,y_tmp,z_tmp,θ = label
	        x,y,z,θ = z_tmp,-x_tmp,y_tmp-self.VELODYNE_HEIGHT,np.pi/2-θ
	        C1,C2,C3,C4,C5,C6,C7,C8 = getCorners(height,width,length,x,y,z,θ,rotation=True)
	        bounding_box = createBBox(bounding_box,C1,C2,C3,C4,C5,C6,C7,C8,colors["Pred"])
	        boxes_pred.append(list((getCorners(height,width,length,x,y,z,θ,rotation=True))))

	    labels_clean_pred_score = []
	    for i,label in enumerate(labels_pred):
	        labels_pred[i]=label[:-2]
	        if labels_pred[i].split(" ")[0] == "Car":
	            labels_clean_pred_score.append(extractscore(labels_pred[i]))


	    conf = []
	    for each in labels_clean_pred_score:
	        conf.append(each[1])
	    conf = np.array(conf)

	    if len(conf)==0 or np.max(conf) < 0.5:
	        maxconf = 1
	    else:
	        maxconf = np.max(conf)

	    if len(conf) == 0:
	        conf = np.array([0])
	        
	    else:
	        conf = (conf-np.min(conf))/(maxconf-np.min(conf))

	    all_box = list()
	    #for each in range(len(bounding_box)):
	    if len(boxes_gt):
	        all_box.append(boxes_gt)
	    # if len(boxes_pred):
	    #     all_box.append(boxes_pred)

	    thres = len(boxes_gt)
	    all_boxes = np.concatenate(all_box)
	    gt_boxes3d = all_boxes
	    num = len(gt_boxes3d)

	    axis = [0, 0, -1]
	    theta = np.pi/2+0.017

	    all_boxes2 = list()
	    for each in all_boxes:
	        #print(np.dot( each, rotation_matrix(axis, theta)))
	        all_boxes2.append(np.dot( each, rotation_matrix(axis, theta)))

	    axis = [-1, 0, 0]
	    theta = np.pi/2+0.017

	    all_boxes3 = list()
	    for each in all_boxes2:
	        #print(np.dot( each, rotation_matrix(axis, theta)))
	        all_boxes3.append(np.dot( each, rotation_matrix(axis, theta)))
	    print(np.array(all_boxes3))
	    # axis = [1, 0, 0]
	    # theta = 2*3.14 

	    # all_boxes4 = list()
	    # for each in all_boxes3:
	    #     #print(np.dot( each, rotation_matrix(axis, theta)))
	    #     all_boxes4.append(np.dot( each, rotation_matrix(axis, theta)))
	    # all_boxes4 = list()
	    for each in all_boxes3:
	        each[:,0] = -each[:,0]
	    print()
	    print(np.array(all_boxes3))
	    
	    #print(all_boxes3)
	    # draw_text = False
	    dict1 = {"data": np.array(all_boxes3).tolist()}
	    with open('data.json', 'w') as outfile:
	        out = json.dumps(dict1)
	        outfile.write(out)
	        outfile.close()
	        return out
	        


	    # for n in range(num):
	    #     b = gt_boxes3d[n]
	    #     if color_list is not None:
	    #         color = color_list[n] 
	    #     if n >= thres:
	    #         color = (0,1,0)
	    #     else:
	    #         color = (1,0,0)
	    #     if draw_text and n >= thres and n-thres <= len(conf): mlab.text3d(b[3,0], b[3,1], b[3,2], '%.1f'%float(conf[n-thres]), scale=text_scale, color=color, figure=fig)
	    #     for k in range(0,4):
	    #         #http://docs.enthought.com/mayavi/mayavi/auto/mlab_helper_functions.html
	    #         i,j=k,(k+1)%4
	    #         mlab.plot3d([b[i,0], b[j,0]], [b[i,1], b[j,
	    #                                                  1]], [b[i,2], b[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)

	    #         i,j=k+4,(k+1)%4 + 4
	    #         mlab.plot3d([b[i,0], b[j,0]], [b[i,1], b[j,1]], [b[i,2], b[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)

	    #         i,j=k,k+4
	    #         mlab.plot3d([b[i,0], b[j,0]], [b[i,1], b[j,1]], [b[i,2], b[j,2]], color=color, tube_radius=None, line_width=line_width, figure=fig)

	def save_annotation(self, drivename, fname, json_str):
		"""
		Saves json string to output directory. 

		Inputs:
		- fname: Frame name. Can have file extension. 
		- json_str: String in json to be saved

		Returns 1 if successful, 0 otherwise
		"""
		assert type(json_str) == str, "json must be a string"
		if not isdir(self.OUTPUT_ANN_DIR):
			try:  
			    makedirs(self.OUTPUT_ANN_DIR)
			except OSError:  
			    print ("Creation of the directory {} failed".format(self.OUTPUT_ANN_DIR))

		output_drive_dir = join(self.OUTPUT_ANN_DIR, drivename)
		if not isdir(output_drive_dir):
			try:  
			    makedirs(output_drive_dir)
			except OSError:  
			    print ("Creation of the directory {} failed".format(output_drive_dir))

		try:
			json_object = json.loads(json_str)
		except ValueError:
			print("Annotation not a valid json")
			return 0
		fname = fname.split(".")[0] + ".json"
		save_filename = join(self.OUTPUT_ANN_DIR, drivename, fname)
		with open(save_filename, "w") as f:
			f.write(json_str)
			return 1
		return 0


