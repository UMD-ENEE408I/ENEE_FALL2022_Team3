main.py will need to be integrated with the main file on the jetson.
Both the song.mp3 and the LoadSong.py need to be in the same root folder as the main python file

Make sure the LoadSong.py has the correct song targeted at ~line 11
To change the number of clusters just change the clusers variable on line 31. Clusers go from 0->num-1.
To get a higher change rate the easiest thing to do is lower the split amount on line 57. either use the normal amount or divide by 1/2/3/4.

WARNING:
EVERY new song you must delete both cache files
Every change of load song you must delete both cache files