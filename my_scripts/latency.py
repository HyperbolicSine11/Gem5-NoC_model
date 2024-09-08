#######################################################################
# Description: extract latency statistics from a series of stats.txt  #
#              and generate csv file for further analysis.            #
#                                                                     #
# Usage: python3.8 latency.py stats1.txt <stats2.txt ...>             #
#######################################################################


import sys
import re
import os

# STATS_FILES = sys.argv[1:]

STATS_PATH = "../8by8_alg1/all_stats/"  # Use your own path.
CSV_PATH = "../8by8_alg1/all_stats/"  # Use your own path.


avg_pkt_latency = []
avg_pkt_net_latency = []
avg_pkt_q_latency = []
# avg_pkt_vq_latency = []

avg_flit_latency = []
avg_flit_net_latency = []
avg_flit_q_latency = []
# avg_flit_vq_latency = []

avg_hops = []

buffer_read = []
buffer_write = []


# extract the file name from the path
FILES = []
# pattern = r"(?<=/)[^/.]+(?=\.txt)"

for file in os.listdir(STATS_PATH):
    # 检查文件扩展名是否为.txt，并且确保它是一个文件而不是目录
    if file.endswith(".txt") and os.path.isfile(
        os.path.join(STATS_PATH, file)
    ):
        # 将.txt文件的完整路径添加到列表中
        FILES.append(os.path.join(STATS_PATH, file))
#    FILES.append(re.findall(r'(\w+)\.txt', file)[0])

# for name in STATS_FILES:
#     match = re.search(pattern, name)
#     if match:
#         FILES.append(match.group(0))


statistic = {
    "average_packet_latency": avg_pkt_latency,
    "average_packet_network_latency": avg_pkt_net_latency,
    "average_packet_queueing_latency": avg_pkt_q_latency,
    # "average_packet_vqueue_latency": avg_pkt_vq_latency,
    "average_flit_latency": avg_flit_latency,
    "average_flit_network_latency": avg_flit_net_latency,
    "average_flit_queueing_latency": avg_flit_q_latency,
    # "average_flit_vqueue_latency": avg_flit_vq_latency,
    "average_hops": avg_hops,
}


def extract_traffic_stats():
    for FILE in os.listdir(STATS_PATH):
        with open(STATS_PATH + FILE, "r") as f:
            lines = f.readlines()
            for line in lines:
                if "average_packet_latency" in line:
                    stats = float(line.split()[1])
                    avg_pkt_latency.append(stats)
                if "average_packet_network_latency" in line:
                    stats = float(line.split()[1])
                    avg_pkt_net_latency.append(stats)
                if "average_packet_queueing_latency" in line:
                    stats = float(line.split()[1])
                    avg_pkt_q_latency.append(stats)
                if "average_flit_latency" in line:
                    stats = float(line.split()[1])
                    avg_flit_latency.append(stats)
                if "average_flit_network_latency" in line:
                    stats = float(line.split()[1])
                    avg_flit_net_latency.append(stats)
                if "average_flit_queueing_latency" in line:
                    stats = float(line.split()[1])
                    avg_flit_q_latency.append(stats)
                if "average_hops" in line:
                    stats = float(line.split()[1])
                    avg_hops.append(stats)


if __name__ == "__main__":
    # Table head
    table_head = "traffics"
    for i in range(len(FILES)):
        table_head += ",%s" % FILES[i]

    extract_traffic_stats()

    with open(CSV_PATH + "latency.csv", "w") as f:
        f.write("%s\n" % table_head)
        for key, value in statistic.items():
            f.write("%s" % key)
            for v in value:
                f.write(",%s" % v)
            f.write("\n")
