"""Bounded ordered geometry work for the selected camera estimator."""
import concurrent.futures
import multiprocessing
import os
import time
from collections import deque

_worker_estimator=None

def _init_worker(urdf,calibration,barrier):
    global _worker_estimator
    from uppercam_square_far20_estimator import SquareFar20Estimator
    _worker_estimator=SquareFar20Estimator(urdf,calibration)
    # Neither worker accepts jobs until both estimators have initialized.
    barrier.wait(timeout=10)

def warm_worker():
    # Keep each worker busy briefly so both spawned processes initialize.
    time.sleep(.10)
    if _worker_estimator is None:raise RuntimeError('worker initialization failed')
    return os.getpid()

def _compute(frame,K):
    start=time.monotonic()
    try:
        return _worker_estimator.geometry(frame,K),None,(time.monotonic()-start)*1000
    except Exception as exc:
        return None,str(exc),(time.monotonic()-start)*1000

class OrderedGeometryQueue:
    def __init__(self,executor,limit=2):
        self.executor=executor;self.limit=limit;self.pending=deque()
    def submit(self,frame,K,context):
        if len(self.pending)>=self.limit:return False
        self.pending.append((self.executor.submit(_compute,frame,K),context));return True
    def ready(self):
        while self.pending and self.pending[0][0].done():
            future,context=self.pending.popleft()
            try: result=future.result()
            except Exception as exc: result=(None,str(exc),0.)
            yield context,result
    def clear(self):
        while self.pending:
            future,_=self.pending.popleft();future.cancel()

def new_geometry_pool(urdf,calibration):
    context=multiprocessing.get_context('spawn')
    barrier=context.Barrier(2)
    return concurrent.futures.ProcessPoolExecutor(max_workers=2,mp_context=context,
        initializer=_init_worker,initargs=(str(urdf),str(calibration),barrier))
