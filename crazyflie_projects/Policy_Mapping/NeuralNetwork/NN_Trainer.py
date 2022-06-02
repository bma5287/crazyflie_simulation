## STANDARD IMPORTS
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

## PYTORCH IMPROTS
import torch
import torch.nn as nn
import torch.nn.functional as F

## SKLEARN IMPORTS
from sklearn.model_selection import train_test_split
from sklearn.metrics import *
from sklearn import preprocessing

import plotly.graph_objects as go

np.set_printoptions(suppress=True)
BASEPATH = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Policy_Mapping"

class NN_Trainer():
    def __init__(self,model,model_initials,learning_rate=0.001,LR_bound=0.9):

        self.model = model
        self.model_initials = model_initials
        self.LR_bound = LR_bound # Landing Rate bound
        self.learning_rate = learning_rate # Learning Rate

        self.scaler = preprocessing.StandardScaler()
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=self.learning_rate)

    def createScaler(self,X):
        """Determine scalers to scale training data to mean = 0 and std = 1 before passing through NN

        Args:
            X (np.float array): training data set
        """        

        ## GENERATE SCALERS
        self.scaler = preprocessing.StandardScaler().fit(X)

        ## SAVE SCALER TO FILE 
        np.savetxt(f"{BASEPATH}/NeuralNetwork/Info/Scaler_{self.model_initials}.csv",
            np.stack((self.scaler.mean_,self.scaler.scale_),axis=1),
            fmt='%.6f',
            delimiter=',',
            comments='',
            header=f'mean,std',
            footer='')

    def loadScaler(self):
        """Load scaler terms from csv file
        """        
        
        arr = np.loadtxt(
            open(f"{BASEPATH}/Info/Scaler_Flip_{self.model_initials}.csv", "rb"),
            delimiter=",",
            skiprows=1)

        self.scaler = preprocessing.StandardScaler()
        self.scaler.mean_ = arr[:,0]
        self.scaler.scale_ = arr[:,1]

    def scaleData(self,X):
        """Scale the given data
        """        

        return self.scaler.transform(X)

    def unscaleData(self,X_scaled):
        """Unscale the given data
        """        

        return self.scaler.inverse_transform(X_scaled)

    def modelPredict(self,X):
        X_scaled = torch.FloatTensor(self.scaleData(X))

        with torch.no_grad():
            y_pred = self.model.forward(X_scaled)

        return y_pred

    def saveParams(self,path):
        f = open(path,'a')
        f.truncate(0) ## Clears contents of file
        f.write("static char NN_Params_Flip[] = {\n")
        

        ## EXTEND SCALER ARRAY DIMENSIONS
        scaler_means = self.scaler.mean_.reshape(-1,1)
        scaler_stds = self.scaler.scale_.reshape(-1,1)
        
        ## SAVE SCALER ARRAY VALUES
        np.savetxt(f,scaler_means,
                    fmt='"%.6f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaler_means.shape[0]},"\t"{scaler_means.shape[1]},"',
                    footer='"*"\n')

        np.savetxt(f,scaler_stds,
                    fmt='"%.6f,"',
                    delimiter='\t',
                    comments='',
                    header=f'"{scaler_stds.shape[0]},"\t"{scaler_stds.shape[1]},"',
                    footer='"*"\n')

        ## SAVE NN LAYER VALUES
        with torch.no_grad():
            ii = 0
            for name, layer in self.model.named_modules():
                if ii > 0: # Skip initialization layer

                    W = layer.weight.numpy()
                    np.savetxt(f,W,
                        fmt='"%.6f,"',
                        delimiter='\t',
                        comments='',
                        header=f'"{W.shape[0]},"\t"{W.shape[1]},"',
                        footer='"*"\n')


                    b = layer.bias.numpy().reshape(-1,1)
                    np.savetxt(f,b,
                        fmt='"%.6f,"',
                        delimiter='\t',
                        comments='',
                        header=f'"{b.shape[0]},"\t"{b.shape[1]},"',
                        footer='"*"\n')

                ii+=1

        f.write("};")
        f.close()

    def loadModelFromParams(self,path):
        """Parse model params from C header file into Pytorch model

        Args:
            path (string): string to C header file
        """        
        
        ## READ PARAMS FROM FILE AS STRING
        f = open(path,"r")
        f.readline() # Skip first line
        file_string = f.read()

        ## CLEAN STRING FROM EXTRA CHARACTERS
        char_remove = ['"','\n','\t']
        for char in char_remove:
            file_string = file_string.replace(char,'')

        ## CREATE LIST OF MATRIX STRINGS
        matrix_list = file_string.split("*")

        ## SPLIT STRINGS INTO PROPER SCALERS AND MATRICES
        arr_list = []
        for ii,matrix_str in enumerate(matrix_list[:-1]):
            val_list = matrix_str[:-1].split(",")   # Break string up into components
            num_rows = int(val_list[0])             # Extract number of rows
            num_cols = int(val_list[1])             # Extract number of columns
            arr = np.array(val_list[2:]).astype(np.float64).reshape(num_rows,num_cols) # Extract matrix

            ## EXTRACT SCALER FOR MEAN
            if ii == 0:
                self.scaler.mean_ = arr[:,0]

            ## EXTRACT SCALER FOR VARIANCE
            elif ii == 1:
                self.scaler.scale_ = arr[:,0]

            ## ADD MATRIX TO ARRAY LIST
            else:
                arr_list.append(arr)

        ## EXTRACT LAYERS FROM PYTORCH MODEL
        layer_list = list(dict(self.model.named_modules()).values())[1:]

        ## FILL MATRICES INTO PYTORCH MODEL
        for ii in range(len(arr_list)):
            if ii%2 == 0:
                layer_list[ii//2].weight.data = torch.FloatTensor(arr_list[ii])
            else:
                layer_list[ii//2].bias.data = torch.FloatTensor(arr_list[ii].flatten())


    def trainModel(self,X_train,y_train,X_test,y_test,LR_bound=0.8,epochs=500):

        ## CONVERT DATA ARRAYS TO TENSORS
        X_train = torch.FloatTensor(self.scaleData(X_train))
        X_test = torch.FloatTensor(self.scaleData(X_test))

        y_train = torch.FloatTensor(y_train)
        y_test = torch.FloatTensor(y_test)


        y_train_My = y_train
        y_test_My = y_test

    
        ## DEFINE TRAINING LOSS
        criterion_train_My = nn.MSELoss()
        losses_train = []
        accuracies_train = []


        ## DEFINE VALIDATION LOSS
        criterion_test_My = nn.MSELoss()
        losses_test = []
        accuracies_test = []

        for ii in range(epochs):


            ## CALC TRAINING LOSS
            y_pred_train = self.model.forward(X_train)

            loss_train_My = criterion_train_My(y_pred_train,y_train_My)
            loss = loss_train_My
            losses_train.append(loss_train_My.item())

            ## CALC VALIDATION LOSS
            with torch.no_grad():
                y_pred_test = self.model.forward(X_test)
                loss_test_My = criterion_test_My(y_pred_test,y_test_My)
                losses_test.append(loss_test_My.item())



            # ## CALC TRAINING ACCURACY
            # y_pred_train_class = np.where(y_pred_train[:,0].detach().numpy() > LR_bound,1,0)
            # accuracy_train = balanced_accuracy_score(np.where(y_train[:,0] > LR_bound,1,0),y_pred_train_class)
            # accuracies_train.append(accuracy_train)

            # ## CALC TESTING ACCURACY
            # y_pred_test_class = np.where(y_pred_test[:,0].detach().numpy() < LR_bound,0,1)
            # accuracy_test = balanced_accuracy_score(np.where(y_test[:,0] < LR_bound,0,1),y_pred_test_class)
            # accuracies_test.append(accuracy_test)

            if ii%10 == 1:
                print(f"epoch {ii} and loss is: {losses_train[-1]}")

            ## BACKPROPAGATION
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()


        ## PLOT LOSSES AND ACCURACIES
        fig = plt.figure(1,figsize=(12,8))
        ax1 = fig.add_subplot(2,1,1)
        ax1.plot(losses_train,label="Training Loss")
        ax1.plot(losses_test,label="Validation Loss")
        ax1.set_ylabel("Loss")
        ax1.set_title("Losses")
        # ax1.set_ylim(0)
        ax1.set_yscale('log')
        ax1.legend()
        ax1.grid()

        ax2 = fig.add_subplot(2,1,2)
        ax2.plot(accuracies_train,label="Training Accuracry")
        ax2.plot(accuracies_test,label="Validation Accuracy")
        ax2.set_ylim(0,1.1)
        ax2.set_ylabel("Classification Accuracy")
        ax2.set_title("Balanced Accuracy")
        ax2.grid()
        ax2.legend(loc="lower right")


        plt.tight_layout()
        plt.show()

    def evalModel(self,X,y,LR_bound=0.9):
        """Evaluate the model

        Args:
            X (_type_): Evaluation dataset
            y (_type_): Evaluation dataset
        """        

        ## PREDICT VALUES FROM EVALUATION DATASET
        y_pred = self.modelPredict(X)
        y_pred = np.where(y_pred[:,0].detach().numpy() < LR_bound,0,1)
        y_class = np.where(y[:,0] < LR_bound,0,1)

        cfnMatrix = confusion_matrix(y_class,y_pred,normalize=None)
        print("\n=========== Model Evaluation ===========")
        print(f"Balanced Accuracy: {balanced_accuracy_score(y_class,y_pred):.3f}")
        print(f"Confusion Matrix: \n{cfnMatrix}")
        print(f"False Positives: {cfnMatrix[0,1]}")
        print(f"False Negatives: {cfnMatrix[1,0]}")
        print(f"Classification Report: {classification_report(y_class,y_pred)}")

    def plotClassification(self,df_custom,LR_bound=0.9):

        ## CREATE GRID
        Tau_grid, OF_y_grid, d_ceil_grid = np.meshgrid(
            np.linspace(0, 0.5, 30),
            np.linspace(-15, 0, 30),
            np.linspace(0, 1.5, 30)
        )

        ## CONCATENATE DATA TO MATCH INPUT
        X_grid = np.stack((
            Tau_grid.flatten(),
            OF_y_grid.flatten(),
            d_ceil_grid.flatten()),axis=1)

        y_pred_grid = self.modelPredict(X_grid)

        fig = go.Figure()

        # fig.add_trace(
        #     go.Volume(
        #         ## ISO SURFACE
        #         x=X_grid[:,1].flatten(),
        #         y=X_grid[:,0].flatten(),
        #         z=X_grid[:,2].flatten(),
        #         value=y_pred_grid[:,0].flatten(),
                
        #         surface_count=10,
        #         opacity=0.1,
        #         isomin=LR_bound-0.05,
        #         isomax=LR_bound,
        #         # colorscale='Viridis',  
        #         cmin=0,
        #         cmax=1,     
        #         caps=dict(x_show=False, y_show=False),
        #         colorbar=dict(title='Title',) , 
        #         hoverinfo='skip'
        #     )
        # )

        ## PLOT DATA POINTS
        fig.add_trace(
            go.Scatter3d(
                ## DATA
                x=df_custom["OFy_flip_mean"],
                y=df_custom["Tau_flip_mean"],
                z=df_custom["D_ceil_flip_mean"],

                ## HOVER DATA
                customdata=df_custom,
                hovertemplate=" \
                    <b>LR: %{customdata[3]:.3f}</b> \
                    <br>OFy: %{customdata[10]:.3f} Vel: %{customdata[0]:.2f} </br> \
                    <br>Tau: %{customdata[8]:.3f} Phi: %{customdata[1]:.0f}</br> \
                    <br>D_ceil: %{customdata[12]:.3f}</br>",

                ## MARKER
                mode='markers',
                marker=dict(
                    size=3,
                    color=df_custom["LR_4leg"],
                    colorscale='Viridis',   # choose a colorscale
                    opacity=0.8)
            )
        )

        # ## PLOT DATA POINTS
        # fig.add_trace(
        #     go.Scatter3d(
        #         ## DATA
        #         x=[-7.754],
        #         y=[0.259],
        #         z=[0.298],

        #         ## MARKER
        #         mode='markers',
        #         marker=dict(
        #             size=3,
        #             color=[1.0],
        #             colorscale='Viridis',   # choose a colorscale
        #             opacity=1.0)
        #     )
        # )

        fig.update_layout(
            scene=dict(
                xaxis_title='OFy [rad/s]',
                yaxis_title='Tau [s]',
                zaxis_title='D_ceiling [m]',
                xaxis_range=[-20,1],
                yaxis_range=[0.4,0.1],
                zaxis_range=[0,1.2],
            ),
        )
        fig.show()

    def plotPolicy(self,df_custom,LR_bound=0.9,PlotRegion=False):

        fig = go.Figure()

        if PlotRegion:
            ## CREATE GRID
            Tau_grid, OF_y_grid, d_ceil_grid = np.meshgrid(
                np.linspace(0.15, 0.35, 45),
                np.linspace(-15, 1, 45),
                np.linspace(0.0, 1.0, 45)
            )

            ## CONCATENATE DATA TO MATCH INPUT
            X_grid = np.stack((
                Tau_grid.flatten(),
                OF_y_grid.flatten(),
                d_ceil_grid.flatten()),axis=1)

            y_pred_grid = self.modelPredict(X_grid).numpy()
            valid_idx = np.where(y_pred_grid[:,0] >= LR_bound)
            X_grid = X_grid[valid_idx]
            y_pred_grid = y_pred_grid[valid_idx]

            ## PLOT DATA POINTS
            fig.add_trace(
                go.Scatter3d(
                    ## DATA
                    x=X_grid[:,1].flatten(),
                    y=X_grid[:,0].flatten(),
                    z=X_grid[:,2].flatten(),

                    ## MARKER
                    mode='markers',
                    marker=dict(
                        size=3,
                        color=np.abs(y_pred_grid[:,1].flatten()),                # set color to an array/list of desired values
                        cmin=0,
                        cmax=10,
                        showscale=True,
                        colorscale='Viridis',   # choose a colorscale
                        opacity=1.0
                    )
                )
            )
        else:
            X_grid = np.stack((
                df_custom["Tau_flip_mean"],
                df_custom["OFy_flip_mean"],
                df_custom["D_ceil_flip_mean"]),axis=1)
            y_pred = self.modelPredict(X_grid)[:,1].numpy().flatten()
            error = df_custom["My_mean"].to_numpy().flatten() - y_pred
        
            ## PLOT DATA POINTS
            fig.add_trace(
                go.Scatter3d(
                    ## DATA
                    x=X_grid[:,1].flatten(),
                    y=X_grid[:,0].flatten(),
                    z=X_grid[:,2].flatten(),

                    ## HOVER DATA
                    customdata=np.stack((error,df_custom["My_mean"].to_numpy().flatten(),y_pred),axis=1),
                    hovertemplate=
                    "<br>My: %{customdata[1]:.3f}</br> \
                     <br>Error: %{customdata[0]:.3f}</br> \
                     <br>Pred: %{customdata[2]:.3f}</br>",
                    
                        

                    ## MARKER
                    mode='markers',
                    marker=dict(
                        size=3,
                        # color=np.abs(df_custom["My_mean"].to_numpy().flatten()),
                        color=np.abs(y_pred),
                        # color = np.abs(error),
                        cmin=0,
                        cmax=10,
                        showscale=True,
                        colorscale='Viridis',   # choose a colorscale
                        opacity=1.0
                    )
                )
            )

        fig.update_layout(
            scene=dict(
                xaxis_title='OFy [rad/s]',
                yaxis_title='Tau [s]',
                zaxis_title='D_ceiling [m]',
                xaxis_range=[-20,1],
                yaxis_range=[0.4,0.1],
                zaxis_range=[0,1.2],
            ),
        )
        fig.show()

    def plotLandingRate(self,df_custom,PlotRegion=False):

        fig = go.Figure()

        if PlotRegion:
            ## CREATE GRID
            Tau_grid, OF_y_grid, d_ceil_grid = np.meshgrid(
                np.linspace(0.15, 0.35, 45),
                np.linspace(-15, 1, 45),
                np.linspace(0.0, 1.0, 45)
            )

            ## CONCATENATE DATA TO MATCH INPUT
            X_grid = np.stack((
                Tau_grid.flatten(),
                OF_y_grid.flatten(),
                d_ceil_grid.flatten()),axis=1)

            y_pred_grid = self.modelPredict(X_grid).numpy()
            valid_idx = np.where(y_pred_grid[:,0] > 0.2)
            X_grid = X_grid[valid_idx]
            y_pred_grid = y_pred_grid[valid_idx]

            ## PLOT DATA POINTS
            fig.add_trace(
                go.Scatter3d(
                    ## DATA
                    x=X_grid[:,1].flatten(),
                    y=X_grid[:,0].flatten(),
                    z=X_grid[:,2].flatten(),

                    ## MARKER
                    mode='markers',
                    marker=dict(
                        size=3,
                        color=np.abs(y_pred_grid[:,0].flatten()),                # set color to an array/list of desired values
                        cmin=0,
                        cmax=1,
                        showscale=True,
                        colorscale='Viridis',   # choose a colorscale
                        opacity=1.0
                    )
                )
            )
        else:
            X_grid = np.stack((
                df_custom["Tau_flip_mean"],
                df_custom["OFy_flip_mean"],
                df_custom["D_ceil_flip_mean"]),axis=1)
            y_pred = self.modelPredict(X_grid)[:,0].numpy().flatten()
            error = df_custom["LR_4leg"].to_numpy().flatten() - y_pred
        
            ## PLOT DATA POINTS
            fig.add_trace(
                go.Scatter3d(
                    ## DATA
                    x=X_grid[:,1].flatten(),
                    y=X_grid[:,0].flatten(),
                    z=X_grid[:,2].flatten(),

                    ## HOVER DATA
                    customdata=np.stack((error,df_custom["LR_4leg"].to_numpy().flatten(),y_pred),axis=1),
                    hovertemplate=
                    "<br>LR: %{customdata[1]:.3f}</br> \
                     <br>Error: %{customdata[0]:.3f}</br> \
                     <br>Pred: %{customdata[2]:.3f}</br>",
                    
                        

                    ## MARKER
                    mode='markers',
                    marker=dict(
                        size=3,
                        # color=np.abs(df_custom["My_mean"].to_numpy().flatten()),
                        color=np.abs(y_pred),
                        # color = np.abs(error),
                        cmin=0,
                        cmax=1,
                        showscale=True,
                        colorscale='Viridis',   # choose a colorscale
                        opacity=1.0
                    )
                )
            )

        fig.update_layout(
            scene=dict(
                xaxis_title='OFy [rad/s]',
                yaxis_title='Tau [s]',
                zaxis_title='D_ceiling [m]',
                xaxis_range=[-20,1],
                yaxis_range=[0.4,0.1],
                zaxis_range=[0,1.2],
            ),
        )
        fig.show()





np.set_printoptions(suppress=True)
BASEPATH = "/home/bhabas/catkin_ws/src/crazyflie_simulation/crazyflie_projects/Policy_Mapping"

## DEFINE NN MODEL
class NN_Model(nn.Module):
    def __init__(self,in_features=3,h=10,out_features=1):
        super().__init__()
        self.fc1 = nn.Linear(in_features,h)     # Layer 1
        self.fc2 = nn.Linear(h,h)               # Layer 2
        self.fc3 = nn.Linear(h,h)               # Layer 3
        self.out = nn.Linear(h,out_features)    # Layer 4

    def forward(self,x):

        # PASS DATA THROUGH NETWORK
        x = F.elu(self.fc1(x))
        x = F.elu(self.fc2(x))
        x = F.elu(self.fc3(x))
        x = self.out(x)

        return x



if __name__ == "__main__":

    ## SET SEEDS
    torch.manual_seed(0)
    np.random.seed(0)

    LR_bound = 0.8 # Classify states with LR higher than this value as valid
    model_initials = "NL_DR"
    learning_rate = 0.01
    model = NN_Model()
    NN_Policy_Trainer = NN_Trainer(model,model_initials,learning_rate=learning_rate,LR_bound=LR_bound)

    ## LOAD DATA
    df_raw = pd.read_csv(f"{BASEPATH}/Data_Logs/NL_DR/NL_LR_Trials_DR.csv").dropna() # Collected data
    # df_raw = df_raw.query("phi_IC >= 86")
    df_bounds = pd.read_csv(f"{BASEPATH}/Data_Logs/NL_Raw/Boundary.csv") # Manufactured data to mold policy region
    df_all = pd.concat([df_raw,df_bounds])

    ## ORGANIZE DATA
    Tau = df_all["Tau_flip_mean"]
    OF_y = df_all["OFy_flip_mean"]
    d_ceil = df_all["D_ceil_flip_mean"]
    learning_rate = df_all["LR_4leg"]
    My = df_all["My_mean"]

    X = np.stack((Tau,OF_y,d_ceil),axis=1)
    y = np.stack((learning_rate,My),axis=1)

    ## SPLIT DATA FEATURES INTO TRAINING AND TESTING SETS
    data_array = np.stack((Tau,OF_y,d_ceil,My),axis=1)
    df = pd.DataFrame(data_array,columns=['Tau','OFy','d_ceil','My_mean'])

    train_df, test_df = train_test_split(df,test_size=0.10,random_state=73)
    X_train = train_df[['Tau','OFy','d_ceil']].to_numpy()
    y_train = train_df[['My_mean']].to_numpy()

    X_test = test_df[['Tau','OFy','d_ceil']].to_numpy()
    y_test = test_df[['My_mean']].to_numpy()


    Param_Path = f'{BASEPATH}/NeuralNetwork/Info/NN_Layers_{model_initials}.h'
    NN_Policy_Trainer.createScaler(X)
    NN_Policy_Trainer.trainModel(X_train,y_train,X_test,y_test,LR_bound=LR_bound,epochs=1000)
    # NN_Policy_Trainer.saveParams(Param_Path)
    # NN_Policy_Trainer.evalModel(X,y)

    # NN_Policy_Trainer.loadModelFromParams(Param_Path)
    # print(NN_Policy_Trainer.modelPredict(np.array([[0.259,-7.754,0.298]])))
    # NN_Policy_Trainer.evalModel(X,y,LR_bound=LR_bound)
    # NN_Policy_Trainer.plotClassification(df_raw,LR_bound=0.8)
    # NN_Policy_Trainer.plotPolicy(df_raw,PlotRegion=True,LR_bound=0.8)
    # NN_Policy_Trainer.plotPolicy(df_raw,PlotRegion=False,LR_bound=0.8)

    # NN_Policy_Trainer.plotLandingRate(df_raw,PlotRegion=True)


