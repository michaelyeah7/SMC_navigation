import csv
import os
import numpy as np

class TsvLoader():
    def __init__(self,file_path):
        #load valuable word from dataset 
        traj_list = []
        with open(file_path) as tsvfile:
            reader = csv.reader(tsvfile, delimiter='\t')
            i = 0
            for row in reader:
                if(i==10):
                    col_name = row
                if(i>10):
                    traj = [float(j) for j in row]
                    traj_list.append(traj)
                i+=1
        name_dict = self.check_col(col_name)
        
        self.col_name = col_name
        self.name_dict = name_dict
        self.traj_list = list(traj_list)
        self.traj_array = np.asarray(traj_list)
        # print(self.traj_array.shape)
        # print(self.traj_array[5,-1])
        # print(self.traj_array[5,0])
        # print(self.traj_array[5,])
        # print(len(self.col_name))
        # print(name_dict.values())
        # print(self.traj_array[:,name_dict.values()[0]])

    def check_col(self,col_name):
        """
        find index of string containing substring
        return: name_idct: dictionary of {name of colum, corresponding index list}
        """
        name_list = []
        name_dict = {}
        for i in range(2,11):
            name_list.append("Helmet_%d"%i)
        
        for name in name_list:
            index = 0
            index_list = []
            for col in col_name:
                if name in col:
                    index_list.append(index)
                index += 1
            name_dict.update({name:index_list})
        return name_dict
    
    def extract_trajectory(self):
        """
        return nine trajectory of each persion, each trajectort includes x,y position
        """
        traj_length = self.traj_array.shape[0]
        trajs = np.zeros(shape=(traj_length,18))
        num_obj = 0
        for value in self.name_dict.values():
            # print(num_obj,value[0:2])
            trajs[:,[num_obj*2,num_obj*2+1]] = self.traj_array[:,value[0:2]]
            num_obj +=1
        # print(trajs[26610,[8,9]])
        return np.array(trajs)

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(os.getcwd()))
    rel_path = '/THOR_dataset/Exp_1_run_1.tsv'
    file_path = script_dir+rel_path
    loader = TsvLoader(file_path = file_path)
    trajs = loader.extract_trajectory()
    print(trajs.shape)