#!/usr/bin/env bash

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo $0 <username>" >&2
  exit 1
fi

TARGET_USER="${1:-${SUDO_USER:-}}"
if [[ -z "${TARGET_USER}" ]]; then
  echo "Missing target user." >&2
  echo "Usage: sudo $0 <username>" >&2
  exit 1
fi

if ! id "${TARGET_USER}" >/dev/null 2>&1; then
  echo "User not found: ${TARGET_USER}" >&2
  exit 1
fi

if ! getent group realtime >/dev/null 2>&1; then
  groupadd --system realtime
fi

usermod -aG realtime "${TARGET_USER}"

LIMITS_FILE="/etc/security/limits.d/99-ros2-control-realtime.conf"
cat > "${LIMITS_FILE}" <<'EOF'
# ROS 2 control realtime permissions.
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft memlock unlimited
@realtime hard memlock unlimited
EOF

ROS2_CONTROL_NODE="/opt/ros/humble/lib/controller_manager/ros2_control_node"
if [[ ! -x "${ROS2_CONTROL_NODE}" ]]; then
  echo "ros2_control_node not found: ${ROS2_CONTROL_NODE}" >&2
  exit 1
fi

setcap cap_sys_nice+ep "${ROS2_CONTROL_NODE}"

# ros2_control_node gains file capabilities above, which enables secure-exec mode.
# In secure-exec mode, LD_LIBRARY_PATH is ignored, so ROS shared libs must be in
# the dynamic linker cache (ld.so.conf + ldconfig) to avoid startup failures like:
# "error while loading shared libraries: libbackward.so: cannot open ..."
ROS2_LIB_DIR="/opt/ros/humble/lib"
LDCONF_FILE="/etc/ld.so.conf.d/ros2-humble.conf"
if [[ -d "${ROS2_LIB_DIR}" ]]; then
  if [[ ! -f "${LDCONF_FILE}" ]] || ! grep -Fxq "${ROS2_LIB_DIR}" "${LDCONF_FILE}"; then
    echo "${ROS2_LIB_DIR}" > "${LDCONF_FILE}"
  fi
  ldconfig
fi

echo "Applied realtime permissions."
echo "Capability on ros2_control_node:"
getcap "${ROS2_CONTROL_NODE}" || true
echo "Dynamic linker entry:"
grep -Fx "${ROS2_LIB_DIR}" "${LDCONF_FILE}" 2>/dev/null || true
echo "User '${TARGET_USER}' groups:"
id -nG "${TARGET_USER}"
echo
echo "Log out and log back in for group + limits changes to apply."
