import torch
import numpy as np
import rospkg
import os
import sys

class LSTM(torch.nn.Module):
    def __init__(self):
        # LSTM Configuration
        self.input_dim = 6
        self.hidden_dim = 90
        self.layers = 4

        super(LSTM, self).__init__()
        self.lstm = torch.nn.LSTM(
            input_size=self.input_dim,
            hidden_size=self.hidden_dim,
            num_layers=self.layers,
            batch_first=True,
            dropout=0.0,
            bidirectional=False,
        )  

        # Fully Connected 
        fc_out_dim = 2
        self.fc = torch.nn.Linear(self.hidden_dim, fc_out_dim)
        # Softmax
        self.softmax = torch.nn.Softmax(dim=0)

        # Hidden Layers
        self.h = None

        ###### CUDA Optimization ######
        if torch.cuda.is_available():  
            dev = "cuda:0" 
        else:  
            dev = "cpu"  
        self.device = torch.device(dev)  


        package_path = rospkg.RosPack().get_path('imu_odometry')
        model = torch.load(os.path.join(package_path, 'data/pretrained-models/zv_lstm_model.tar'))
        my_dict = self.state_dict()
        for key,value in my_dict.items(): 
            my_dict[key]=model[key]
        self.load_state_dict(my_dict)
        self.eval()

    def forward(self, x, h=None, mode="train"):
        # Removing time stamp
        x = x[-1,1:]

        batch_size = 1
        seq_len = 1 #x.shape[0]
    
        x = torch.FloatTensor(x).view((batch_size, seq_len, self.input_dim))

        #initalize the hidden output if not already initialized
        if self.h is None:
            if h is None:    
                h_n = x.data.new(self.layers, batch_size, self.hidden_dim).normal_(0, 0.1)
                h_c = x.data.new(self.layers, batch_size, self.hidden_dim).normal_(0, 0.1)
                # h_n = h_n.cuda()
                # h_c = h_c.cuda()
            else:
                h_n, h_c = h 
        else:              
            h_n, h_c = self.h

        self.lstm.flatten_parameters()
        
        # LSTM
        r_out, (h_n, h_c) = self.lstm(x, (h_n, h_c))   # r_out 
        
        # Updating hidden states
        self.h = (h_n, h_c)
        
        # Extracting Last Output
        lstm_out = r_out.squeeze() #[-1, :]        # Dim: [90]     #r_out[0,:,:]
        # Fully Connected Layer
        fc_out = self.fc(lstm_out)              # Dim: [2]
        # Softmax Layer
        softmax_out = self.softmax(fc_out)      # Dim: [2]

        # Classify : Class 0: zv=0  Class 1: zv=1
        prob, zv = torch.max(softmax_out.cpu().data, 0)
        
        # Retrieve data from tensors
        zv = zv.numpy()
        prob = prob.numpy()

        # Ignoring low confidence estimations
        # zv[np.where(prob<=0.85)] = 0
        # if prob <= 0.85:
            # zv = 0
        
        return zv
