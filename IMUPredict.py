import serial
import time
from torch import nn
import torch
import torch.nn.functional as F
# Open the serial port with PySerial
ser = serial.Serial('/dev/ttyUSB0', 9600)    #IMU data
ser1 = serial.Serial('/dev/ttyACM0',115200)       #GSM and bluetooth

start = time.time()
queue = []
count = 0
# load model
#
#
# class FallDetector(nn.Module):
#     def __init__(self):
#         super(FallDetector, self,).__init__()
#         self.lstm = nn.LSTM(6, 30, num_layers=1)
#         self.hidden2tag = nn.Linear(30, 1)
#
#     def forward(self, seq):
#         output, (h_n, c_n) = self.lstm(seq.view(len(seq), 1, -1))
#         tag_space = self.hidden2tag(c_n.view(1, -1))
#         tag_scores = torch.sigmoid(tag_space)
#         return tag_scores.view(-1)



#
# class FallDetector(nn.Module):
#     def init(self):
#         super(FallDetector, self).init()
#         self.gru = nn.GRU(6, 30, num_layers=1)
#         self.hidden2tag = nn.Linear(30, 1)
#
#     def forward(self, seq):
#         (h_n, c_n) = self.gru(seq.view(len(seq), 1, -1))
#         tag_space = self.hidden2tag(c_n.view(1, -1))
#         # print(tag_space.data)
#         tag_scores = torch.sigmoid(tag_space)
#         return tag_scores.view(-1)
#
# model = FallDetector()
# model.load_state_dict(torch.load('accuracy78.7.pt'))
# model.eval()

class FallDetector(nn.Module):
    def __init__(self):
        super(FallDetector, self,).__init__()
        self.lstm = nn.LSTM(6, 5, num_layers=1)
        self.hidden2tag = nn.Linear(5, 1)

    def forward(self, seq):
        output, (h_n, c_n) = self.lstm(seq.view(len(seq), 1, -1))
        tag_space = self.hidden2tag(c_n.view(1, -1))
        # print(tag_space.data)
        tag_scores = torch.sigmoid(tag_space)
        return tag_scores.view(-1)

model = FallDetector()
model.load_state_dict(torch.load("model2.pt"))
model.eval()


def model_time():
    # print(queue)
    t = torch.tensor(queue, dtype=torch.float)
    global count
    # print("about to eval")
    result = model(t)
    if result > 0.5:
        print("Fall detected")
        count +=1
    else:
        count = 0
    if count == 10:
        print("fall")
        ser1.write('s'.encode('utf-8'))
        print("Message")
        ser.close()
        ser1.close()




while True:
    try:
        line = ser.readline().decode()

        if line:
            # Split the line into a timestamp and data
            timestamp = line.strip().split(',')
            if (len(timestamp) == 6):
                varlist = [timestamp[0], timestamp[1],
                                    timestamp[2], timestamp[3], timestamp[4], timestamp[5]]
                for i in range(6):
                    varlist[i] = float(varlist[i])
                if (len(queue)<100):
                    queue.append(varlist)
                else:
                    print([timestamp[0],
                           timestamp[1], timestamp[2], timestamp[3], timestamp[4], timestamp[5]])
                    queue.pop(0)
                    queue.append(varlist)
                    # print("anju")
                    model_time()
                    # print("chettri")
                    # break
        # if (time.time()-start >100):
        #     break


    except:
        continue



#
# queue = []
#
# queue.append([1,2])
# queue.append(2)
# queue.append(3)
# queue.append(4)
#
# print(queue)
#
# queue.pop(0)
# print(queue)















# import serial
# import time
# from torch import nn
# import torch
# import torch.nn.functional as F
# # Open the serial port with PySerial
# ser = serial.Serial('/dev/ttyUSB0', 9600)    #IMU data
# #ser1 = serial.Serial('/dev/ttyACM0',115200)       #GSM and bluetooth
# start = time.time()
# queue = []
#
# count = 0
# class FallDetector(nn.Module):
#     def __init__(self):
#         super(FallDetector, self,).__init__()
#         self.gru = nn.GRU(6, 30, num_layers=2)
#         self.hidden2tag = nn.Linear(30, 1)
#
#     def forward(self, seq):
#         output, (h_n, c_n) = self.gru(seq.view(len(seq), 1, -1))
#         tag_space = self.hidden2tag(c_n.view(1, -1))
#         # print(tag_space.data)
#         tag_scores = torch.sigmoid(tag_space)
#         return tag_scores.view(-1)
#
# model = FallDetector()
# model.load_state_dict(torch.load("model_weights.pt"))
# model.eval()
#
#
# def model_time() :
#     global count
#     # print(queue)
#     t = torch.tensor(queue, dtype=torch.float)
#     # print("about to eval")
#     result = model(t)
#     #print("eval done")
#     if result > 0.5:
#         print("Fall detected")
#     #      count +=1
#     # else:
#     #     count = 0
#     # if count == 10:
#     #     print("fall")
#     #     ser1.write('s'.encode('utf-8'))
#     #     print("Message")
#     #     ser.close()
#     #     ser1.close()
#
#
#
#     # print("fall" if result > 0.5 else "non-fall")
#
#
# while True:
#     try:
#         line = ser.readline().decode()
#
#         if line:
#             # Split the line into a timestamp and data
#             timestamp = line.strip().split(',')
#             if (len(timestamp) == 6):
#                 varlist = [timestamp[0], timestamp[1],
#                                     timestamp[2], timestamp[3], timestamp[4], timestamp[5]]
#                 for i in range(6):
#                     varlist[i] = float(varlist[i])
#                 if (len(queue)<100):
#                     queue.append(varlist)
#                 else:
#                     print([timestamp[0],
#                            timestamp[1], timestamp[2], timestamp[3], timestamp[4], timestamp[5]])
#                     queue.pop(0)
#                     queue.append(varlist)
#                     # print("anju")
#                     model_time()
#                     # print("chettri")
#                     # break
#         if (time.time()-start >100):
#             break
#
#
#     except:
#         continue
#
#
#
