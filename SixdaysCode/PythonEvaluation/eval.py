import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import sys
import itertools
import operator

#usage python eval.py GT_FILE_LASER_1 EX_FILE_LASER_1 ...

from numpy import genfromtxt

matplotlib.rcParams.update({'font.size': 22})
fig=plt.figure()


args_num=len(sys.argv)
num_lasers=(args_num-1)/2
axes=fig.subplots(num_lasers,1,sharex=True)
for laser in range(num_lasers):

    def readFile(filename, time_scale,offset):
        data=genfromtxt(filename, delimiter=' ')
        all_data_points=[]
        for key, group in itertools.groupby(data,key=operator.itemgetter(1)):
            if(key > 0.):
                gr=list(group)
                all_data_points.append((gr[0][0]/time_scale+offset,gr[-1][0]/time_scale+offset))
        return all_data_points


    gt_data=np.array(readFile(sys.argv[laser*2+1],1024.,0))
    ep_data=np.array(readFile(sys.argv[laser*2+2],1.,0.))
    #print(gt_data.shape)
    #print(ep_data.shape)

    #print(ep_data)

    for data in [gt_data,ep_data]:
        data=np.mean(data,axis=1)
        #print(data)

    gt_data=np.mean(gt_data,axis=1)
    ep_data=np.mean(ep_data,axis=1)

    def find_nearest(array, value):
        array = np.asarray(array)
        idx = (np.abs(array - value)).argmin()
        return idx

    pairs=[]
    pairs_s=[]
    for i in range(gt_data.shape[0]):
        pairs.append((find_nearest(ep_data,gt_data[i]),i))
    
    offsets=[pair[0]-pair[1] for pair in pairs]
    if max(offsets)!=min(offsets):
        print(pairs)
        print(gt_data)
        print(ep_data)
    #print(gt_data)
    #print(ep_data)
    errors=[]
    errors_s=[]
    for i,j in pairs:
        errors.append(ep_data[i]-gt_data[j])


    #print(errors)
    print( "%1.3f"%(np.mean(errors)))    
    print("%1.3f"%(np.mean(np.abs(errors)))) 
    print("%1.3f"%(np.sqrt(np.mean(np.square(errors)))))
    print("%1.3f"%errors[np.argmax(np.abs(errors))])
    print("%1.3f"%np.std(errors))
    print( " ")
    l_index=sys.argv[laser*2+1].find("Laser")
    ax=axes[laser]
    ax.plot(errors, label="L"+sys.argv[laser*2+1][l_index+6])
    ax.legend(loc=1)
    plt.xlim(0,73)


fig.text(0.5, 0.04, 'Round Number', ha='center', va='center')
fig.text(0.04, 0.5, 'Time error (s)', ha='center', va='center', rotation='vertical')
#plt.show()
fig.set_size_inches([15,12])	
plt.savefig("time_error.pdf", bbox_inches='tight')
