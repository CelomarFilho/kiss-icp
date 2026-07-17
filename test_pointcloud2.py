import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import glob, os


BAG = '/tmp/conv_check'
STORAGE = 'mcap' if glob.glob(os.path.join(BAG, '*.mcap')) else 'sqlite3'


reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=BAG, storage_id=STORAGE),
            rosbag2_py.ConverterOptions('', ''))

# lista tópicos e tipos
tipos = {t.name: t.type for t in reader.get_all_topics_and_types()}
print('=== tópicos na bag ===')
for n, ty in tipos.items():
    print(f'  {n}   ->   {ty}')

# acha o primeiro tópico de PointCloud2
pc_topics = [n for n, ty in tipos.items() if ty == 'sensor_msgs/msg/PointCloud2']
print('\ntópicos PointCloud2:', pc_topics or 'NENHUM (a bag pode ter só CustomMsg cru)')

if pc_topics:
    TOPIC = pc_topics[0]
    dt = {1:'INT8',2:'UINT8',3:'INT16',4:'UINT16',5:'INT32',6:'UINT32',7:'FLOAT32',8:'FLOAT64'}
    msg = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == TOPIC:
            msg = deserialize_message(data, PointCloud2); break
    print(f'\n=== primeiro scan de {TOPIC!r} ===')
    print('point_step =', msg.point_step, ' n_points =', msg.width*msg.height)
    for f in msg.fields:
        print(f'  campo name={f.name!r:14} tipo={dt.get(f.datatype)} offset={f.offset}')
    ok = {'t','timestamp','time','time_stamp'}
    rec = [f.name for f in msg.fields if f.name in ok]
    print('reconhecido pelo KISS-ICP?', rec or 'NAO -> deskew DESLIGADO')
    cand = rec[0] if rec else [f.name for f in msg.fields if 'time' in f.name.lower()][0]
    ts = np.array(list(point_cloud2.read_points(msg, field_names=[cand], skip_nans=False)),
                  dtype=np.float64).ravel()
    print(f'\ncampo usado: {cand!r}')
    print('  min/max/span =', ts.min(), ts.max(), ts.max()-ts.min())
    print('  unicos =', np.unique(ts).size, '/', ts.size)
    print('  NaN/Inf =', int(np.isnan(ts).sum()), int(np.isinf(ts).sum()))
    print('  monotonico?', bool(np.all(np.diff(ts) >= 0)))
    print('  primeiros 5:', ts[:5])