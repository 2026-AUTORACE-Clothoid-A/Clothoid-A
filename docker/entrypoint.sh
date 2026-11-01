#!/bin/bash
# Remap the container user's uid to the host user so files in the mounted repo keep their owner.
if [ -n "${LOCAL_USER_ID}" ] && [ "${LOCAL_USER_ID}" != "0" ]; then
    if [ "$(id -u user)" != "${LOCAL_USER_ID}" ]; then
        usermod -u "${LOCAL_USER_ID}" user
        chown -R "${LOCAL_USER_ID}" /home/user 2>/dev/null || true
    fi
    exec gosu user "$@"
else
    exec "$@"
fi
