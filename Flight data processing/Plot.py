from matplotlib import pyplot
from pyulog.core import ULog

filename = './data/' +'log_59_2019-4-13-10-49-40.ulg'
ulog = ULog(filename)

'Create key_dict={para_name: data file index}'
key=[]
fn=[]
for d in ulog.data_list:
    u = []
    keyd = list(d.data.keys())
    fn.append(d.name)
    key.append(keyd)
ind2key = dict(list(enumerate(key)))

key2ind = []
for i, j in ind2key.items():
    for k in range(0,len((j,i)[0])):
        key2ind.append([(j,i)[0][k],(j,i)[1]])
key_dict = dict()
for line in key2ind:
    if line[0] in key_dict:
        key_dict[line[0]].append(line[1])
    else:
        key_dict[line[0]] = [line[1]]

'Application of key_dict, observe para_name, file_index, and file_name'
print(list(key_dict.keys()))
index = key_dict['alt']
print(index)
[print(fn[i])for i in index]

'Plot desired properties'
index=10
y = ulog.data_list[index].data['alt']
t = ulog.data_list[index].data['timestamp']

pyplot.plot(t, y)
pyplot.xlabel('Timestamp')
pyplot.ylabel('Altitude')
pyplot.grid()
pyplot.show()