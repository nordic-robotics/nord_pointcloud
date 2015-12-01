import numpy as np
f = open("confusion_3nn_voxelized.txt",'rb')

lines = f.readlines()
mat = []


mat = np.array([ map(int, line.split()) for line in lines ])
#print mat
mat2 = mat[0:7,:] + mat[7:14,:] + mat[14:21,:] + mat[21:28,:] + mat[28:,:]
#print mat2
summa = np.sum(mat2.astype('float'),1)
confusion = np.transpose(np.divide(np.transpose(mat2),summa))
#print (100*np.transpose(np.divide(np.transpose(mat2),summa)) )

color = np.array([0.5,0,0.5,0,0,0,0]) 
shape = np.array([0,0,0,0,0,1,0]) 
avail = np.array([[1,0,0,0,0,0,1],[0,1,0,0,0,0,0],[1,0,0,1,0,1,1],[0,0,0,0,1,0,0],[1,0,0,0,0,0,0],[0,1,1,0,0,0,0],[0,0,0,0,0,1,0]])

availShape = np.dot(avail,color)

confusionShapes = np.multiply(confusion[:,shape>0],shape[shape>0])

sums = np.sum(confusionShapes,1)

possibleOutcomes = np.multiply(sums,availShape)

guess = np.argmax(possibleOutcomes)

print possibleOutcomes
print guess

probs = np.transpose(np.multiply(np.transpose(np.multiply(color,avail)),possibleOutcomes))
print probs
# shape = np.zeros(1,7)
# shape[guess] = 1

# availColour = np.dot(shape, avail)