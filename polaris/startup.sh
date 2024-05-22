sudo bash -c 'for i in $(seq 0 $(($(nproc)-1))); do cpufreq-set -c $i -g performance; done'

