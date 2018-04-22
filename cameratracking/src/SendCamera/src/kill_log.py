import calibrationOutput
import time
import threading

class KillLog:
    def __init__(self):
        self.initialTime = time.time()
        self.inDeadZone = False
        self.outputFile = calibrationOutput.CSVWriter()
        self.outputFile.setFilenamePostLetters('_kill_log')
        self.outputFile.setHeader(['time','timeDifference','inDeadZone','x','y','theta'])

    def setPosition(self, position):
        currentTime = time.time()
        timeDifference = currentTime - self.initialTime
        self.outputFile.append([currentTime,timeDifference,self.inDeadZone,
                                position[0],position[1],position[2]])

    def setPositionThreaded(self, position):
        thread = threading.Thread(target=self.setPosition, args=(position,))
        thread.daemon = True
        thread.start()

    def saveLog(self):
        print("Saving kill log")
        self.outputFile.saveToFile()
