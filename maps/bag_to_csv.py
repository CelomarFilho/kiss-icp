#!/usr/bin/env python3

import argparse
import csv
import re
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = [
    "/kiss/odometry",
    "/spool_cable_odom",
]


def detect_storage_id(bag_path: Path) -> str:
    """
    Detecta automaticamente sqlite3 ou mcap.
    """

    if bag_path.is_file():
        if bag_path.suffix == ".mcap":
            return "mcap"
        if bag_path.suffix == ".db3":
            return "sqlite3"

    metadata_path = bag_path / "metadata.yaml"

    if metadata_path.exists():
        metadata = metadata_path.read_text(encoding="utf-8")

        match = re.search(
            r"^\s*storage_identifier:\s*['\"]?([^'\"\s]+)",
            metadata,
            re.MULTILINE,
        )

        if match:
            return match.group(1)

    if bag_path.is_dir():
        if list(bag_path.glob("*.mcap")):
            return "mcap"

        if list(bag_path.glob("*.db3")):
            return "sqlite3"

    raise RuntimeError(
        "Não foi possível detectar o formato da bag. "
        "Use --storage-id sqlite3 ou --storage-id mcap."
    )


def get_header_timestamp_ns(message):
    """
    Retorna o timestamp do header em nanossegundos.
    Retorna None se a mensagem não possuir header.
    """

    try:
        stamp = message.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except AttributeError:
        return None


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Extrai pose.pose.position.z dos tópicos "
            "/kiss/odometry e /spool_cable_odom."
        )
    )

    parser.add_argument(
        "bag",
        type=Path,
        help="Diretório da rosbag contendo metadata.yaml",
    )

    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path("odometry_z.csv"),
        help="Arquivo CSV de saída",
    )

    parser.add_argument(
        "--storage-id",
        choices=["sqlite3", "mcap"],
        default=None,
        help="Formato da bag. Por padrão, é detectado automaticamente.",
    )

    args = parser.parse_args()

    if not args.bag.exists():
        print(f"Erro: bag não encontrada: {args.bag}", file=sys.stderr)
        return 1

    storage_id = args.storage_id or detect_storage_id(args.bag)

    print(f"Bag: {args.bag}")
    print(f"Storage: {storage_id}")

    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=str(args.bag),
        storage_id=storage_id,
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {
        topic_info.name: topic_info.type
        for topic_info in topic_types
    }

    missing_topics = [
        topic
        for topic in TOPICS
        if topic not in type_map
    ]

    if missing_topics:
        print(
            "Erro: os seguintes tópicos não foram encontrados na bag:",
            file=sys.stderr,
        )

        for topic in missing_topics:
            print(f"  {topic}", file=sys.stderr)

        print("\nTópicos disponíveis:", file=sys.stderr)

        for topic in sorted(type_map):
            print(f"  {topic}: {type_map[topic]}", file=sys.stderr)

        return 1

    print("\nTipos encontrados:")

    for topic in TOPICS:
        print(f"  {topic}: {type_map[topic]}")

    # Evita percorrer e retornar os tópicos de câmera, point cloud etc.
    reader.set_filter(
        rosbag2_py.StorageFilter(topics=TOPICS)
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)

    first_bag_timestamp_ns = None
    counts = {topic: 0 for topic in TOPICS}

    with args.output.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as csv_file:

        writer = csv.writer(csv_file)

        writer.writerow(
            [
                "topic",
                "message_type",
                "bag_time_ns",
                "bag_time_s",
                "time_from_start_s",
                "header_time_ns",
                "header_time_s",
                "z",
            ]
        )

        while reader.has_next():
            topic, serialized_data, bag_timestamp_ns = reader.read_next()

            message_class = get_message(type_map[topic])
            message = deserialize_message(
                serialized_data,
                message_class,
            )

            try:
                z = float(message.pose.pose.position.z)
            except AttributeError:
                print(
                    f"Erro: o tópico {topic}, do tipo "
                    f"{type_map[topic]}, não possui o campo "
                    "pose.pose.position.z.",
                    file=sys.stderr,
                )
                return 1

            if first_bag_timestamp_ns is None:
                first_bag_timestamp_ns = bag_timestamp_ns

            time_from_start_s = (
                bag_timestamp_ns - first_bag_timestamp_ns
            ) / 1_000_000_000.0

            header_timestamp_ns = get_header_timestamp_ns(message)

            if header_timestamp_ns is None:
                header_timestamp_s = ""
                header_timestamp_ns_csv = ""
            else:
                header_timestamp_s = (
                    header_timestamp_ns / 1_000_000_000.0
                )
                header_timestamp_ns_csv = header_timestamp_ns

            writer.writerow(
                [
                    topic,
                    type_map[topic],
                    bag_timestamp_ns,
                    f"{bag_timestamp_ns / 1_000_000_000.0:.9f}",
                    f"{time_from_start_s:.9f}",
                    header_timestamp_ns_csv,
                    (
                        f"{header_timestamp_s:.9f}"
                        if header_timestamp_s != ""
                        else ""
                    ),
                    f"{z:.12f}",
                ]
            )

            counts[topic] += 1

    print(f"\nCSV criado: {args.output.resolve()}")

    for topic, count in counts.items():
        print(f"  {topic}: {count} mensagens")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())