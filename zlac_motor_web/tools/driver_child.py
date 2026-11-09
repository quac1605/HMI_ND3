#!/usr/bin/env python3
"""Linux child: request graceful driver shutdown if the guard parent dies."""
import ctypes
import os
import signal
import sys

parent=int(sys.argv[1])
libc=ctypes.CDLL(None,use_errno=True)
if libc.prctl(1,signal.SIGINT,0,0,0) != 0:  # PR_SET_PDEATHSIG
    raise OSError(ctypes.get_errno(),'prctl(PR_SET_PDEATHSIG) failed')
# Parent could die between spawn and prctl. Never start the driver in that case.
if os.getppid()!=parent: sys.exit(1)
os.execv(sys.argv[2],sys.argv[2:])
