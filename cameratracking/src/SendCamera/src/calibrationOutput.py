""" Module use to import and export data """
import json
import time
import datetime
import os
import threading
import csv
import errno

#If no calibration filepath is given we will have this one
CALIBRATION_FOLDER = 'CalibrationFiles'

def getFormattedTimeStamp():
    ts = time.time()
    return datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')

def getScriptPath():
    #return os.path.dirname(os.path.realpath(sys.argv[0]))
    return os.path.dirname(os.path.realpath(__file__))

def getCalibrationFilename():
    return os.path.join(CALIBRATION_FOLDER,"calibrationFactors.txt")

def saveObject(obj, filename):
    """Dumps the object to a Json file named filename, if no filename is
    provided then a name with a timestap is created
    """
    if (filename==''):
            filename = getCalibrationFilename()
    with open(filename, 'w+') as output:
        json.dump(obj, output)

def saveObjectThreaded(obj, filename):
    thread = threading.Thread(target=saveObject, args=(obj, filename))
    thread.daemon = True
    thread.start()

def loadObject(filename):
    print("Loading object "+filename)
    if (filename!=""):
        with open(filename, 'r') as input:
            return json.load(input)
    else:
        print("Filename was empty")

def loadObjectThreaded(filename):
    """ This would not work well as returning values from a thread is something
        special, and actually not desired for our application
    """
    thread = threading.Thread(target=loadObject, args=(filename,))
    thread.daemon = True
    thread.start()

def saveCalibration(calibrationTuple):
    saveObjectThreaded(calibrationTuple, getCalibrationFilename())

def loadCalibration():
    """ Here we dont use the threaded version because we need to make sure the
        parameters are read in the program
    """
    return loadObject(getCalibrationFilename())

def getCalibrationListFromFile(filename):
    """Load calibration point list from file. If no filename given
    the program takes the last file of the folder
    """
    fileList = getListOfFilesInDir("") #No path to get the default
    if (filename!=""):
        calibrationPoints = json.load(filename)
    elif (fileList): #if fileList is not empty
        calibrationPoints = json.load(fileList[-1])
        #We get the last element in fileList, because we will sort them with timestamp
        #Maybe sorting could be useful in this case if the filenames are not sorted already
    if calibrationPoints is None:
        print("There was no file found")
    return calibrationPoints

def getListOfFilesInDir(self, filePath):
    fileList = []
    if (filePath != ""):
        path = filePath
    else:
        path = CALIBRATION_FOLDER
    for (dirpath, dirnames, filenames) in walk(path):
        fileList.extend(filenames)
        break #Just to get the top folder and not recurse inside
    return fileList

def make_sure_path_exists(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise

class CSVWriter:
    """Responsible to writing a serie of tuples (rows) to a CSV file. Depends
       on some functions on CalibrationOutput Python script.
    """
    def __init__(self):
        self.dataOutputFolder = os.path.join(getScriptPath(),'csvOutput')
        self.filename = os.path.join(self.dataOutputFolder,getFormattedTimeStamp()+'.csv')
        self.rows = []

    def append(self, tupleToAppend):
        self.rows.append(tupleToAppend)

    def setHeader(self, headerTuple):
        self.rows.insert(0,headerTuple)

    def setFilenamePostLetters(self,postletters):
        self.filename = os.path.join(self.dataOutputFolder,getFormattedTimeStamp()+postletters+'.csv')

    def saveToFile(self):
        make_sure_path_exists(self.dataOutputFolder)
        csvFile = open(self.filename, "a")
        csvWriter = csv.writer(csvFile, delimiter=',', quotechar='"', quoting=csv.QUOTE_ALL)
        csvWriter.writerows(self.rows)
        csvFile.close()
