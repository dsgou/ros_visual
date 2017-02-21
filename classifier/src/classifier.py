#!/usr/bin/env python

import rospy
import numpy 
import cPickle 
import sklearn
import operator
from std_msgs.msg import String
from fusion.msg import FusionMsg
from pyAudioAnalysis import audioTrainTest

global SVM, Mean, Std, ClassNames, counter, dicti, fps, publisher, events

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
    fv = numpy.array([data.boxes[0].pos.area, data.boxes[0].pos.area_diff, data.boxes[0].pos.ratio, data.boxes[0].pos.ratio_diff, data.boxes[0].pos.distance, data.boxes[0].pos.distance_diff])
    curFV = (fv - Mean) / Std                
    [Result, P] = audioTrainTest.classifierWrapper(SVM, "svm", curFV) 

    
    dicti[ClassNames[int(Result)]] += 1
    counter += 1
    if counter == fps:
        m = max(dicti.iteritems(), key=operator.itemgetter(1))[0]
        print m
        publisher.publish(m)
        counter = 0
        for key, value in dicti.iteritems():
            dicti[key] = 0
    #~ print Mean
    #~ print curFV 

if __name__ == '__main__':
    global SVM, Mean, Std, ClassNames, fps, publisher, dicti
    rospy.init_node('classifier')
    svm_path = rospy.get_param('~svm_path')
    input_topic = rospy.get_param('~input_topic')
    fps = rospy.get_param('~fps')
    events = rospy.get_param('~events')
    dicti = {}
    for e in events.split(' '):
        dicti[e] = 0
    print dicti
    publisher = rospy.Publisher('/classifier/result', String, queue_size = 1)
    rospy.Subscriber(input_topic, FusionMsg, callback)
    (SVM, Mean, Std, ClassNames) = loadSVModel(svm_path)
    rospy.spin()
