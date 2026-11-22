# UR5 VLM demo: Gazebo Harmonic + Python controller + web GUI  (Ubuntu 24.04, x86_64)
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates wget gnupg curl \
 && wget -q https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic \
      python3 python3-numpy python3-pil python3-protobuf python3-gz-transport13 python3-gz-msgs10 \
      libgl1-mesa-dri libglx-mesa0 libegl1 libgbm1 mesa-utils x11-utils xauth \
      libxcb-xinerama0 libxcb-cursor0 libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-render-util0 libxcb-xkb1 libxkbcommon-x11-0 \
 && rm -rf /var/lib/apt/lists/*

# fail the build early (clear message) if the Python bindings are not what the code expects
RUN python3 -c "from gz.transport13 import Node; \
from gz.msgs10.image_pb2 import Image; from gz.msgs10.pose_v_pb2 import Pose_V; from gz.msgs10.model_pb2 import Model; \
from gz.msgs10.double_pb2 import Double; from gz.msgs10.empty_pb2 import Empty; from gz.msgs10.pose_pb2 import Pose; \
from gz.msgs10.boolean_pb2 import Boolean; from gz.msgs10.world_stats_pb2 import WorldStatistics; print('gz python bindings OK')" \
 && gz sim --versions

COPY app /opt/ur5/app
COPY sim /opt/ur5/sim
COPY scripts /opt/ur5/scripts
RUN chmod +x /opt/ur5/scripts/*.sh && python3 /opt/ur5/sim/gen_world.py /tmp/build_check.sdf \
 && (gz sdf -k /tmp/build_check.sdf || echo "WARNING: gz sdf -k reported problems (see above)")

ENV PYTHONPATH=/opt/ur5/app QT_X11_NO_MITSHM=1 PORT=8080
WORKDIR /opt/ur5/app
EXPOSE 8080
ENTRYPOINT ["/opt/ur5/scripts/entrypoint.sh"]
