#!/usr/bin/env python

import rospy
import numpy 
import cPickle 
import sklearn
import operator
from std_msgs.msg import String
from ros_visual_msgs.msg import FusionMsg
from pyAudioAnalysis import audioTrainTest

global SVM 
global Mean, Std, ClassNames, counter, dicti, fps, publisher, events

counter = 0

'''
This function loads an SVM model either for classification or training.
ARGMUMENTS:
    - SVMmodelName:     the path of the model to be loaded
    - isRegression:        a flag indigating whereas this model is regression or not
'''
def loadSVModel(SVMmodelName):    
    try:
        fo = open(SVMmodelName+"MEANS", "rb")
    except IOError:
            print "Load SVM Model: Didn't find file"
            return
  
    MEAN = cPickle.load(fo)
    STD = cPickle.load(fo)
    classNames = cPickle.load(fo)

    fo.close()

    MEAN = numpy.array(MEAN)
    STD = numpy.array(STD)

    COEFF = []
    with open(SVMmodelName, 'rb') as fid:
        SVM = cPickle.load(fid)    
    return(SVM, MEAN, STD, classNames)


def callback(data):
    global SVM, Mean, Std, ClassNames, counter, dicti, fps, publisher
    
    #Classify
    fv = numpy.array(
    [
        data.boxes[0].pos.ratio, 
        data.boxes[0].pos.ratio_diff, 
        data.boxes[0].pos.distance, 
        data.boxes[0].pos.distance_diff,
        data.boxes[0].pos.x_diff,
        data.boxes[0].pos.x_delta,
        data.boxes[0].pos.y_diff,
        data.boxes[0].pos.y_delta,
        data.boxes[0].pos.y_norm,
        data.boxes[0].pos.y_norm_diff,
        data.boxes[0].pos.depth_std,
        data.boxes[0].pos.z_diff,
        data.boxes[0].pos.z_diff_norm
    ])
    curFV = (fv - Mean) / Std                
    [Result, P] = audioTrainTest.classifierWrapper(SVM, "gradientboosting", curFV) 

    
    dicti[ClassNames[int(Result)]] += 1
    counter += 1
    if counter == fps:
        m = max(dicti.iteritems(), key=operator.itemgetter(1))[0]
        #~ print m
        publisher.publish(m)
        counter = 0
        for key, value in dicti.iteritems():
            dicti[key] = 0

if __name__ == '__main__':
    global SVM, Mean, Std, ClassNames, fps, publisher, dicti
    rospy.init_node('classifier')
    fps         = int(rospy.get_param('~fps')/10)
    svm_path    = rospy.get_param('~svm_path')
    path        = rospy.get_param('~classifier_path')
    input_topic = rospy.get_param('~input_topic')
    
    publisher = rospy.Publisher('/classifier/result', String, queue_size = 1)
    rospy.Subscriber(input_topic, FusionMsg, callback)
    (SVM, Mean, Std, ClassNames) = loadSVModel(path + svm_path)
    
    dicti = {}
    for c in ClassNames:
        dicti[c] = 0
    
    rospy.spin()
