import numpy as np, statistics, math, matplotlib.pyplot as plt, sklearn
from sklearn.cluster import KMeans
import librosa, librosa.display

def LoadSong():


    #x, sr = librosa.load('SafenSound.mp3')
    print("librosa load")
    x, sr = librosa.load('PartyRock.mp3')

    filter_length=2048
    #hop_length = int(sr * .01) #creates a hop length of 10ms
    #win_Length = int(sr * .05) #creates a window frame of 50ms

    print("stft")
    win_Length = 1024
    hop_length = win_Length//4
    S_ = librosa.stft(x,n_fft=filter_length, win_length=win_Length, hop_length=hop_length)
    #S_ = librosa.stft(x)

    _S = librosa.amplitude_to_db(abs(S_))
    S = np.transpose(_S)



    ## start
    print("clustering")
    clusters = 6
    kmeans = KMeans(n_clusters=clusters, random_state=0).fit(S)
    t = kmeans.labels_


    #used when graphing
    y = np.arange(len(t))
    y = y * hop_length/sr


    persecond = 0
    for i in range (0,len(y)):
        if(y[i]<1):
            persecond = persecond + 1
            #print(str(y[i]) + " " + str(persecond))

    #print(persecond)
    print("smoothing")
    test = t.copy()
    findmedian = []

    yes = 1
    # amount = 50 #amuont of concurent things to check
    # length = len(test)-amount-1
    splitAmount = persecond
    splitAmount = persecond * 4 # amount of data points to check per second. * how many seconds for each move
    # splitAmount = 200
    totalSplits = math.floor(len(y)/splitAmount)
    print(totalSplits)
    count = 0
    total = 0
    counter = 0
    while yes:
        for i in range(0, totalSplits): #goes through it all
            for n in range(0+splitAmount*i, splitAmount+splitAmount*i):
                total = test[n] + total
                findmedian.append(test[n])
                #print(str(test[n]) +" "+ str(total)+" "+str(count))
                #print(total)
                count +=1
                counter +=1
            avg = round(total/count)
            median = statistics.median(findmedian)
            findmedian.clear()
            #print(avg)
            for p in range(0+splitAmount*i, splitAmount+splitAmount*i):
                #test[p] = avg
                test[p] = median
            
            total = 0
            count = 0
        yes = 0




    # BELOW IS NOT THE CLUSTERINT
    # finds how often the dance changes moves
    switches = 0

    for n in range(len(test)):
        if(test[n] != test[n-1]):
            switches +=1

    # creates the time array
    _d = np.zeros((2,switches+1))
    #print(_d)
    _d[1,0] = (test[0])
 

    count = 0
    for n in range(len(test)):
        if(test[n] != test[n-1]):
            count +=1
            #print(test[n])
            #print(n)
            _d[1,count] = test[n]
            _d[0,count] = n*hop_length/sr

    print(_d)

    return _d, switches