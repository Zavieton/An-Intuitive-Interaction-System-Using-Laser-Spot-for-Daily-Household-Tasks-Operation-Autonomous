import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
from torch.nn import functional as F
from torch.autograd import Variable
from sklearn.model_selection import train_test_split


class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.fc1 = nn.Linear(26, 500)
        self.fc2 = nn.Linear(500, 500)
        self.fc3 = nn.Linear(500, 100)
        self.fc4 = nn.Linear(100, 4)
        self.dropout = nn.Dropout(0.3)

    def forward(self, x):
        x = F.tanh(self.fc1(x))
        x = self.dropout(x)
        x = F.tanh(self.fc2(x))
        x = self.dropout(x)
        x = F.tanh(self.fc3(x))
        x = self.fc4(x)
        return x


class Net2(nn.Module):
    def __init__(self):
        super(Net2, self).__init__()
        self.fc1 = nn.Linear(26, 100)
        self.fc2 = nn.Linear(100, 100)
        self.fc3 = nn.Linear(100, 4)
        self.dropout = nn.Dropout(0.3)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = self.dropout(x)
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x

path = '/home/zavieton/catkin_ws/Excel_test3.xls'
data = pd.read_excel(path)
print(data.head(10))
features = ['1.1',  '1.2',  '1.3',      0,  '0.1',  '0.2',  '0.3',  '0.4',
        '0.5',  '0.6',  '0.7',  '0.8',  '0.9', '0.10', '0.11', '0.12', '0.13',
       '0.14', '0.15', '0.16', '0.17', '0.18', '0.19', '0.20', '0.21', '0.22']
X = data[features]
y = data[1]

print(X.head(10))
X_train,X_cv, y_train, y_cv = train_test_split(X,y,test_size = 0.2, random_state = 0)  #0.2为测试集占比
X_train = torch.tensor(X_train.values,dtype = torch.float)
X_cv = torch.tensor(X_cv.values,dtype = torch.float)
y_train = torch.tensor(y_train.values,dtype = torch.long)
model = Net2()
criterion = nn.CrossEntropyLoss()
optimizer = torch.optim.Adam(model.parameters(),lr = 0.001)
batch_size = 32
n_epochs = 5000

batch_no = len(X_train) // batch_size

train_loss = 0
train_loss_min = np.Inf  # 无穷大
fig = plt.figure(1)

for epoch in range(n_epochs):
    for i in range(batch_no):
        start = i * batch_size
        end = start + batch_size
        x_var = Variable(X_train[start:end])
        y_var = Variable(y_train[start:end])

        optimizer.zero_grad()
        output = model(x_var)
        loss = criterion(output, y_var)
        loss.backward()
        optimizer.step()
        values, labels = torch.max(output, 1)
        num_right = np.sum(labels.data.numpy() == y_train[start:end].numpy())

        train_loss += loss.item() * batch_size

    train_loss = train_loss / len(X_train)

    plt.plot(epoch, train_loss, 'g.-')
    fig.canvas.draw()
    # fig.canvas.flush_events()

    if train_loss <= train_loss_min:
        print("Validation loss decreased ({:6f} ===> {:6f}). Saving the model...".format(train_loss_min, train_loss))
        torch.save(model.state_dict(), "model.pt")
        train_loss_min = train_loss

    if epoch % 200 == 0:
        print('')
        print("Epoch: {} \tTrain Loss: {} \t".format(epoch + 1, train_loss))
        print('Accuracy is %f' % (float(num_right) / float(len(y_train[start:end]))))

print('Training Ended! ')

with torch.no_grad():
    test_result = model(X_cv)

values, labels = torch.max(test_result, 1)
testlabel = list(labels.numpy())
yy = list(y_cv)
score = 0

for i in range(len(testlabel)):
    if testlabel[i] == yy [i]:
        score += 1
accuracy = float(score)/len(testlabel)
print(accuracy)