from math import e
import numpy as np
from Model import NN
from examine_sample import load_training_data, get_n_to_1

fk_save_file = "./models/training_data_rot_fk_full_aug"
ik_save_file = "./models/training_data_rot_ik_full_aug"
fk_load_file = ""
ik_load_file = ""

files = ["training_data_rot.npz","training_data_rot_sparse.npz","training_data_rot_sparse^2.npz"]

model = NN(ik_save_file,fk_save_file,
			ik_save_file,fk_save_file,load=True)

#act_pos,ee_pos,ee_rot = load_training_data(files[0])
#model.train_networks(act_pos,ee_pos,ee_rot)
#model.save_all()

traj = np.linspace([0,0,2],[1,2,2],10)
pred,valid = model.predict_ik_traj(traj)
breakpoint()
'''
test_data = np.random.uniform(low=0,high=4,size=(100,3))
e1 = m1.fk(test_data)
e2 = m2.fk(test_data)
e3 = m3.fk(test_data)
e4 = m4.fk(test_data)

print("1,2:",np.mean(np.linalg.norm(e1-e2,axis=1)))
print("3,2:",np.mean(np.linalg.norm(e3-e2,axis=1)))
print("1,3:",np.mean(np.linalg.norm(e1-e3,axis=1)))
print("1,4:",np.mean(np.linalg.norm(e1-e4,axis=1)))
print("3,4:",np.mean(np.linalg.norm(e3-e4,axis=1)))
print("2,4:",np.mean(np.linalg.norm(e4-e2,axis=1)))

'''