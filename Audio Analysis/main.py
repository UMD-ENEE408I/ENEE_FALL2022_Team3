import time
from LoadSong import LoadSong


print("calling load song")
array, _length = LoadSong()


input("Press Enter to continue...")

_time = time.time()
true = 1
length = _length+1
count = 0
while(true):



    if(time.time()-_time > array[0,count]):
        count +=1
        
        print("increased by another, went at time " + str(time.time()-_time) + " to dance move " + str(array[1,count]) + " this is change number: " + str(count))
        #print(count)
        #print(time.time()-_time)
        if count > length-1:
            true = 0

    #print("This will be the ")