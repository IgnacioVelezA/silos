from src.radar_communication import serial_radar
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse
from src.scripts import distanceFinder



#////curve_analizer ==========================================================================
def curve_analizer(curve, thr, mindist, maxdist):
    std = round(np.std(curve),2)
    mean = round(np.mean(curve),2)
    max_index = np.argmax(curve)
    distance = distanceFinder.distanceSplines(curve, thr, MINDISTANCE, MAXDISTANCE,0)
    return distance, mean, std

#////END: curve_analizer =====================================================================

def animate(i, xs, thr, mindist, maxdist):
    """
    This function is called periodically from FuncAnimation
    """
    serial_radar.start_reading_curve()
    curve = serial_radar.read_curve()
    #stop_reading_curve()
    # Add x and y to lists

    ys = np.array(curve)
    max_power = max(ys)
    #distance, mean, std = curve_analizer(ys, thr, mindist, maxdist)
    distance, cleaned_curve, x_interpl= distanceFinder.distanceSplines(ys, thr ,MINDISTANCE,MAXDISTANCE, 0)
    # Draw x and y lists
    ax.clear()
    #ax.plot(xs, ys,label="original")
    ax.plot(x_interpl, cleaned_curve)
    ax.vlines(distance,0,50, label=f'radial={distance}')
    # ax.axvline(x = distance, color = 'red', label = f'distance: {distance}')
    # ax.axhline(y = mean, color = 'green', label = f'average: {mean} \n standar dev: {std} \n maxPW = {max_power}')
    ax.set_ylim([0,100])
    #ax.plot(new_x, new_y,label="smooth")

    # Format plot
	#plt.xticks(rotation=45, ha='right')
	#plt.subplots_adjust(bottom=0.30)
    plt.title('Power received per distance')
    plt.ylabel('Power')
    plt.xlabel('Distance [m]')
    plt.legend()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='plots a silos measurement'+
                                                 ' from a pkl file')
    parser.add_argument('arduino_usb_port', help='arduino usb port for serial comm, tipically /dev/ttyUSB1')
    parser.add_argument('-min', '--MINDISTANCE', default = 0, type = int)
    parser.add_argument('-max', '--MAXDISTANCE', default = 30, type = int)
    parser.add_argument('-thr', '--THRESHOLD', default = 0, type = int)
    # Create figure for plotting
    args = parser.parse_args()
    MINDISTANCE = args.MINDISTANCE
    MAXDISTANCE = args.MAXDISTANCE
    THRESHOLD = args.THRESHOLD
    xs = np.array(np.arange(start=MINDISTANCE, stop=MAXDISTANCE, step=MAXDISTANCE/128))

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    
    serial_radar.open_serial(args.arduino_usb_port)
    print(serial_radar.is_serial_open())
    # Set up plot to call animate() function periodically
    ani = animation.FuncAnimation(fig, animate, fargs=(xs, THRESHOLD, MINDISTANCE, MAXDISTANCE), interval=(800))
    plt.show()

    serial_radar.close_serial()
