#!/usr/bin/env python3
"""
bag_to_csv.py

Extrai (tempo, x, y, z) de topicos nav_msgs/Odometry de uma bag ROS2 e grava
CSV pronto para MATLAB/Python.

O tempo e normalizado: t=0 no primeiro timestamp visto na bag (considerando
todos os topicos extraidos), para as curvas ficarem no mesmo eixo.

Uso:
  python3 bag_to_csv.py /tmp/run_baseline
  python3 bag_to_csv.py /tmp/run_baseline --topics /kiss/odometry /spool_cable_odom
  python3 bag_to_csv.py /tmp/run_anchor --out-prefix anchor

Saida:
  <prefix>_kiss_odometry.csv     colunas: t,x,y,z
  <prefix>_spool_cable_odom.csv  colunas: t,x,y,z
"""

import argparse
import glob
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def detect_storage(uri):
    if glob.glob(os.path.join(uri, '*.mcap')):
        return 'mcap'
    if glob.glob(os.path.join(uri, '*.db3')):
        return 'sqlite3'
    raise RuntimeError(f'Nenhum .mcap ou .db3 encontrado em {uri}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='pasta da bag (com metadata.yaml)')
    ap.add_argument('--topics', nargs='*', default=None,
                    help='topicos Odometry a extrair (default: todos os Odometry)')
    ap.add_argument('--out-prefix', default=None,
                    help='prefixo dos CSV (default: nome da pasta da bag)')
    args = ap.parse_args()

    uri = os.path.abspath(args.bag.rstrip('/'))
    storage = detect_storage(uri)
    prefix = args.out_prefix or os.path.basename(uri)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage),
                rosbag2_py.ConverterOptions('', ''))

    tipos = {t.name: t.type for t in reader.get_all_topics_and_types()}
    odom_topics = [n for n, ty in tipos.items() if ty == 'nav_msgs/msg/Odometry']
    if args.topics:
        alvo = [t for t in args.topics if t in odom_topics]
        faltando = [t for t in args.topics if t not in odom_topics]
        if faltando:
            print(f'AVISO: nao encontrados (ou nao sao Odometry): {faltando}', file=sys.stderr)
    else:
        alvo = odom_topics

    if not alvo:
        print('Nenhum topico Odometry para extrair.', file=sys.stderr)
        print(f'Topicos na bag: {list(tipos)}', file=sys.stderr)
        return 1

    print(f'storage={storage}  topicos={alvo}')

    dados = {t: [] for t in alvo}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic in dados:
            m = deserialize_message(raw, Odometry)
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            p = m.pose.pose.position
            dados[topic].append((t, p.x, p.y, p.z))

    # tempo relativo comum
    t0 = min((v[0][0] for v in dados.values() if v), default=None)
    if t0 is None:
        print('Nenhuma mensagem lida.', file=sys.stderr)
        return 1

    for topic, linhas in dados.items():
        if not linhas:
            print(f'  {topic}: 0 msgs (pulado)')
            continue
        nome = topic.strip('/').replace('/', '_')
        path = f'{prefix}_{nome}.csv'
        with open(path, 'w') as f:
            f.write('t,x,y,z\n')
            for (t, x, y, z) in linhas:
                f.write(f'{t - t0:.6f},{x:.6f},{y:.6f},{z:.6f}\n')
        zs = [l[3] for l in linhas]
        print(f'  {topic}: {len(linhas)} msgs -> {path}  (z: {min(zs):.3f} a {max(zs):.3f})')

    return 0


if __name__ == '__main__':
    sys.exit(main())
