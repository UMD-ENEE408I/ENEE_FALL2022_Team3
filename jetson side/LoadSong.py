import numpy as np, statistics, math, matplotlib.pyplot as plt, sklearn
from sklearn.cluster import KMeans, BisectingKMeans
import librosa, librosa.display
import random

import os.path


def noCache():

    print("librosa load")
    x, sr = librosa.load('SafenSound.mp3')
    #x, sr = librosa.load('PartyRock.mp3')

    
    #hop_length = int(sr * .01) #creates a hop length of 10ms
    #win_Length = int(sr * .05) #creates a window frame of 50ms

    print("stft")
    filter_length = 2048
    win_Length = 1024
    hop_length = win_Length

    D = np.abs(librosa.stft(x))**2
    _S = librosa.feature.melspectrogram(S=D, sr=sr,n_fft=filter_length, win_length=win_Length, hop_length=hop_length)
    S = np.transpose(_S)

    ## start
    print("clustering")
    clusters = 5
    kmeans = BisectingKMeans(n_clusters=clusters, init='random', random_state=42,n_init = 1000,).fit(S)
    t = kmeans.labels_


    #used when graphing
    y = np.arange(len(t))
    y = y * hop_length/sr/2

    # finds how many data points are used persecond
    persecond = 0
    for i in range (0,len(y)):
        if(y[i]<1):
            persecond = persecond + 1


    print("smoothing")
    test = t.copy()
    findmedian = []

    yes = 1
    splitAmount = persecond # amount of data points to check per second. * how many seconds for each move
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
                count +=1
                counter +=1
            median = statistics.median(findmedian)
            findmedian.clear()
            for p in range(0+splitAmount*i, splitAmount+splitAmount*i):
                test[p] = median
            
            total = 0
            count = 0
        yes = 0

    #find hte most common, make the circle
    amount = [0,0,0,0,0]    
    for p in kmeans.labels_:
        if(p == 0):
            amount[0] = amount[0] + 1
        if(p == 1):
            amount[1] = amount[1] + 1
        if(p == 2):
            amount[2] = amount[2] + 1
        if(p == 3):
            amount[3] = amount[3] + 1
        if(p == 4):
            amount[4] = amount[4] + 1
                #amount[i] =+1
    print(amount)


    #finds the two most common
    most_common = max(amount)
    most_common_index = amount.index(most_common)
    #print(most_common)
    #print(most_common_index)

    without_most_common = amount.copy()
    #print(without_most_common)
    #print('test')
    without_most_common.remove(most_common)
    #print(without_most_common)

    new_most_common = max(without_most_common)
    new_most_common_index = amount.index(new_most_common)

    #print(most_common_index)
    #print(new_most_common_index)

    new = t.copy()
    for i in range(0, totalSplits): #goes through it all
        for p in range(0+splitAmount*i, splitAmount+splitAmount*i):
            if test[p] == most_common_index:
                new[p] = 1 #our best and most basic move
            if test[p] == new_most_common_index:
                new[p] = 2 #our second best and most basic move
            if test[p] == 1: #swapping with most common
                new[p] = most_common_index
            if test[p] == 2:
                new[p] = new_most_common_index
            
        #test[p] = median

    #swap the next popular ones to our best ones

    #will try and make a move happen every 4 seconds
    last_value = new[0]
    count = 0
    print("total splits " + str(totalSplits))

    #every three seconds we want to switch
    number_of_seconds_to_change = 3
    seconds_of_same_move_until_switch = 6
    for i in range(0, totalSplits): #goes through it all
        
        
        if last_value == new[splitAmount*i]:
            count = count + 1
        if last_value != new[splitAmount*i]:
            count = 0
            last_value = new[splitAmount*i]
        
        if count > seconds_of_same_move_until_switch:
            new_int = random.randint(0,clusters)
            while(new_int == last_value):
                new_int = random.randint(0,clusters)
                #print(new_int)
            
            if(splitAmount*number_of_seconds_to_change+splitAmount*i < len(new)):
                for n in range(0+splitAmount*i, splitAmount*number_of_seconds_to_change+splitAmount*i):
                    new[n] = new_int
            count = 0

                
        #print(count)




    # End of Smoothing
    # finds how often the dance changes moves
    switches = 0

    for n in range(len(new)):
        if(new[n] != new[n-1]):
            switches +=1

    # creates the time array
    _d = np.zeros((2,switches+1))
    #print(_d)
    _d[1,0] = (new[0])
 

    count = 0
    for n in range(len(new)):
        if(new[n] != new[n-1]):
            count +=1
            #print(new[n])
            #print(n)
            _d[1,count] = new[n]
            _d[0,count] = n*hop_length/sr/2

    #print(_d)

    createCache(_d,switches)

    return _d, switches


def createCache(array,switches):
    print("create the caches ")
    np.save("CacheArray.npy",array,allow_pickle=True)
    np.save("CacheSwitches.npy",switches,allow_pickle=True)
    print("created the caches")



def LoadSong():
    #check cache
    file_exists = os.path.exists('CacheArray.npy')
    if(file_exists):
        print("found cache")
        array = np.load("CacheArray.npy",allow_pickle=True)
        length = np.load("CacheSwitches.npy",allow_pickle=True)
    else:
        print("no cache, must process song")
        array, length = noCache()

    print("Song loaded")
    return array, length

