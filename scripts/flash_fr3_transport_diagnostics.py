"""Read-only Linux UDP counters belonging to the current process only."""
import os
import socket
from pathlib import Path


def own_udp_sockets():
    descriptors = {}
    for name in os.listdir('/proc/self/fd'):
        try:
            link = os.readlink('/proc/self/fd/' + name)
            if link.startswith('socket:['):
                descriptors[link[8:-1]] = int(name)
        except OSError:
            pass
    rows = []
    for table in ('udp', 'udp6'):
        try:
            lines = Path('/proc/self/net/' + table).read_text().splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            fields = line.split()
            if len(fields) < 13 or fields[9] not in descriptors:
                continue
            fd = descriptors[fields[9]]
            row = {'table': table, 'inode': fields[9], 'fd': fd,
                   'local_address_hex': fields[1], 'drops': int(fields[-1]),
                   'receive_queue_bytes': int(fields[4].split(':')[1], 16)}
            try:
                duplicate = socket.fromfd(fd, socket.AF_INET, socket.SOCK_DGRAM)
                try:
                    row['receive_buffer_bytes'] = duplicate.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
                finally:
                    duplicate.close()
            except OSError as error:
                row['receive_buffer_error'] = str(error)
            rows.append(row)
    return rows


def udp_drop_increment(before, after):
    baseline = {row['inode']: row['drops'] for row in before}
    return sum(max(0, row['drops'] - baseline.get(row['inode'], 0)) for row in after)
