
import subprocess
import argparse
import os
import csv
import numpy as np


if __name__=='__main__':
    parser = argparse.ArgumentParser(description='recovers a pickle dump generated'+
                                                 ' by silos_measurement.py script')
    parser.add_argument('file_name', help='name of the file')
    parser.add_argument('-i','--ip', help ='ip where the raspi is')
    parser.add_argument('-min', '--MINDISTANCE', default = 0, type = int)
    parser.add_argument('-max', '--MAXDISTANCE', default = 30, type = int)
    parser.add_argument('--save', action='store_true', help="write this if you want to save as CSV", default=False)
    parser.add_argument('--no-save', dest='save', action='store_false', help="write this if you DON'T want to save as CSV")
    args = parser.parse_args()

    if args.ip:
        ip = args.ip
    else:
        ip = '192.168.0.21'
    
    MINDISTANCE = args.MINDISTANCE
    MAXDISTANCE = args.MAXDISTANCE
    save = args.save 

    full_name = args.file_name
    date_measured = full_name[-10:]

    path = 'measurements/' + date_measured

    try:
        os.mkdir(path)
    except:
        print(f'{date_measured} file already exist')
    
    path = path + '/' + args.file_name + '.csv'

    server='silos@'+ip+':silos/measurements/'+ full_name +'.csv'
    command = ['scp',  server, path]
    subprocess.run(command)

    # file = open(path, 'rb')

    # traj_angle_dict = 0
    # distances_date_tuple_list = []
    # file_len_counter = 0
    # while True:
    #     try:
    #         if file_len_counter == 0:
    #             traj_angle_dict = pkl.load(file)
    #         else:
    #             distances_date_tuple_list.append(pkl.load(file))
    #         file_len_counter += 1
    #     except:
    #         break
    # file.close()

    # if save:
    #     pathCSV = path[:-4]+'.csv'

    #     with open(pathCSV, 'w') as csvfile:#, newline=''
        
    #         newCSV = csv.writer(csvfile)
    #         params = traj_angle_dict['params']
    #         traj = traj_angle_dict['traj']
    #         n_points = len(distances_date_tuple_list[0])
    #         print(f'n_point = {n_points}')
    #         listCSV = [distance_vector]
    #         print(f'len measured = {len(distances_date_tuple_list)}')
    #         for i in range(n_points):
    #             rowCSV = [f'{traj[i][0]}',f'{traj[i][1]}']
    #             print(f'n{i}')
    #             point_and_measurement_i = list(np.array(distances_date_tuple_list[0][i][1]))
    #             rowCSV= rowCSV + point_and_measurement_i
    #             listCSV.append(rowCSV)
    #         newCSV.writerows(listCSV)
    #         traj
    #         csvfile.close() # file = open(path, 'rb')

    # traj_angle_dict = 0
    # distances_date_tuple_list = []
    # file_len_counter = 0
    # while True:
    #     try:
    #         if file_len_counter == 0:
    #             traj_angle_dict = pkl.load(file)
    #         else:
    #             distances_date_tuple_list.append(pkl.load(file))
    #         file_len_counter += 1
    #     except:
    #         break
    # file.close()

    # if save:
    #     pathCSV = path[:-4]+'.csv'

    #     with open(pathCSV, 'w') as csvfile:#, newline=''
        
    #         newCSV = csv.writer(csvfile)
    #         params = traj_angle_dict['params']
    #         traj = traj_angle_dict['traj']
    #         n_points = len(distances_date_tuple_list[0])
    #         print(f'n_point = {n_points}')
    #         listCSV = [distance_vector]
    #         print(f'len measured = {len(distances_date_tuple_list)}')
    #         for i in range(n_points):
    #             rowCSV = [f'{traj[i][0]}',f'{traj[i][1]}']
    #             print(f'n{i}')
    #             point_and_measurement_i = list(np.array(distances_date_tuple_list[0][i][1]))
    #             rowCSV= rowCSV + point_and_measurement_i
    #             listCSV.append(rowCSV)
    #         newCSV.writerows(listCSV)
    #         traj
    #         csvfile.close()