import numpy as np
import math
from scipy.signal import savgol_filter
#from scipy.interpolate import interp1d
from scipy.interpolate import CubicSpline, interp1d

emptyCurve = np.array([22,12.0526,19,20,22,23,19.4737,18.5789,12.9474,17.42105,
    	12,11.0526,13,10,12,12,12,10,13,11,11,12,11,9,11,12,13,10,13,15,10,
        11,10,10,8.2632,10,7.2632,7.3684,12,11.05263,8,10,10,7.0526,9,9,8,
        10,10,8,12,11,9,9,8,9,8,8,8,9,13,9,10,12,9,9,10,9,8,9,9,6,10,9,9,
        9,9,9,7,8,7,7,8,8,7,8,8,9,8,7,12,10,10,11,8,8,10,9,7,9,10,9,8,10,
        5.0526,6,9,5.0526,10,9,8,9,8,6,7,9,7,8,8,6,9,9,7,8,7,6,7,6])

        
#////backwardDer=======================================================================================
def backwardDer(curve_array):
    element_zero = 0
    bckwrd = []
    bckwrd.append((curve_array[0]-element_zero))
    for i in range(len(curve_array)-1):
        diff = curve_array[i+1]-curve_array[i]
        bckwrd.append(diff)
    bckwrd = np.array(bckwrd)
    return bckwrd
#////END: backwardDer====================================================================================


#////forwardDer==========================================================================================
def forwardDer(curve_array):
    final_element = 0
    frwrd = []
    l = len(curve_array)
    for i in range(l-1):
        diff = curve_array[i+1]-curve_array[i]
        frwrd.append(diff)
    frwrd.append(-curve_array[l-1])
    frwrd = np.array(frwrd)
    return frwrd
#////END: forwardDer=====================================================================================


#////LocalMaxDetector====================================================================================
def LocalMaxDetector(curve_array):
    backward_derivative = backwardDer(curve_array)
    forward_derivative = forwardDer(curve_array)
    boolList = []
    for i in range(len(curve_array)):
        if backward_derivative[i] > 0:
            if forward_derivative[i] < 0:
                boolList.append(True)
            else:
                boolList.append(False)
        else:
            boolList.append(False)
    return boolList
#////END: LocalMaxDetector===============================================================================


#////distanceFinder================================================================================
def distanceFinder(curve, minAmp, MINDISTANCE, MAXDISTANCE, jump):
    if jump < MINDISTANCE or jump > MAXDISTANCE:
        print('Jump distance (in meters) not in range, using normal curve')
        jump = MINDISTANCE
        
    print(f'Jumping distance set to {jump} meters')
    x = np.linspace(MINDISTANCE,MAXDISTANCE,128)
    x_interpl = np.linspace(MINDISTANCE,MAXDISTANCE,2048)
    xstep = x_interpl[1] - x_interpl[0]

    corrected_curve = curve#-emptyCurve
    
    xy_spline = interp1d(x,corrected_curve)
    interpl_curve = xy_spline(x_interpl)
    smoothed_curve = savgol_filter(interpl_curve,16,2)

    jump_indx = math.ceil((jump - MINDISTANCE)/xstep)
    cleaned_curve = smoothed_curve[jump_indx:]

    maxPeak = max(cleaned_curve)
    maxIndex = np.argmax(cleaned_curve)
    localMax = LocalMaxDetector(cleaned_curve)

    i = 0
    if minAmp > 0:
        while i< len(localMax):
            if localMax[i]:
                if cleaned_curve[i] >= maxPeak - minAmp:
                    return i*xstep + MINDISTANCE
            i +=1
    distance = maxIndex*xstep + jump
    return distance
#////END: distanceFinder===========================================================================


#////DistanceSplines ==============================================================================

def distanceSplines(curve_og, minAmp, MINDISTANCE, MAXDISTANCE, jump ,spline =0):

    x = np.linspace(MINDISTANCE,MAXDISTANCE,128)
    x_interpl = np.linspace(MINDISTANCE,MAXDISTANCE,2048)
    xstep = x_interpl[1] - x_interpl[0]

    curve_mean = np.zeros_like(curve_og)

    x = np.linspace(MINDISTANCE,MAXDISTANCE,128)

    # curve meaing --------------------------------------
    curve_mean[0] = (curve_og[0]+curve_og[1])/3
    for i in range(len(curve_og)-2):
        curve_mean[i+1] = (curve_og[i]+curve_og[i+1]+curve_og[i+2])/3 

    i += 1

    curve_mean[i+1] = (curve_og[i]+curve_og[i+1])/3
    # end: curve meaning --------------------------------

    xy_spline = interp1d(x, curve_og)
    mean_spline = interp1d(x, curve_mean)

    interpl_curve = xy_spline(x_interpl)
    interpl_meancurve = mean_spline(x_interpl)

    cubicSP = CubicSpline(x, curve_og)
    
    interpl_cubicSP_curve = cubicSP(x_interpl)

    # Jumping method -----------------------------------
    jump_indx = math.ceil((jump - MINDISTANCE)/xstep)
    cleaned_curve = interpl_cubicSP_curve[jump_indx:] # < Cambiar segun curva que se use

    maxPeak = max(cleaned_curve)
    maxIndex = np.argmax(cleaned_curve)
    localMax = LocalMaxDetector(cleaned_curve)


    i = 0
    if minAmp > 0:
        while i< len(localMax):
            if localMax[i]:
                if cleaned_curve[i] >= maxPeak - minAmp:
                    return i*xstep + MINDISTANCE
            i +=1
    distance = maxIndex*xstep + jump
    
    return distance

        

#////END: DistanceSplines =========================================================================