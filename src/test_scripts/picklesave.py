import pickle as pkl
import numpy as np
import time


if __name__== '__main__':
    name = "testpickle.pkl"

    wf = open(name, 'wb')

    t = np.array(['a','b', 'c', 'd', 'f','g','h','i', 'j', 'k'])

    pkl.dump(t, wf)

    indices = ['x']
    pkl.dump(indices,wf)
    wf.close()

    for i in range(10):
        indices.append(i) #esto representa la tomsa de medición, el momento más lento
        time.sleep(2)
        rf = open(name, 'rb')
        a = pkl.load(rf)
        b = pkl.load(rf)
        print(f'a loaded {i} =', a    )
        print(f'b loaded {i} =', b    )
        rf.close()
        print(i)

        wf = open(name, 'wb')
        pkl.dump(t, wf)
        pkl.dump(indices, wf)
        wf.close()




