"""Offline cost comparison only; no camera, production edits, or multi-target deployment."""
import json,time,statistics,os
from pathlib import Path
import cv2,numpy as np,torch
from stack_traffic.node import classify_signal_color
from stack_traffic.visual_tracker import ShortTermTemplateTracker,smooth_bbox
from stack_traffic.logic import select_tracking_candidate
from ultralytics import YOLO
rng=np.random.default_rng(20260917)
frame=rng.integers(0,256,(360,640,3),dtype=np.uint8)
result={'input':'deterministic synthetic 640x360 BGR; stationary textured boxes',
        'cpu_count':os.cpu_count(),'opencv_threads':cv2.getNumThreads(),
        'opencv_opencl_active':cv2.ocl.useOpenCL(),'torch_cuda_available':torch.cuda.is_available(),
        'scenarios':[]}
for width,height in [(16,12),(40,24),(100,60)]:
 boxes=[(160,100,160+width,100+height),(400,100,400+width,100+height)]
 # Ten retained candidates, two valid local matches and eight distant distractors.
 candidates=[(b,.19) for b in boxes]+[((10+i*12,280,18+i*12,290),.12) for i in range(8)]
 trackers=[ShortTermTemplateTracker() for b in boxes]
 for tr,b in zip(trackers,boxes):assert tr.initialize(frame,b)
 def work(count,i):
  for j in range(count):
   b=boxes[j]
   if i%2==0: # alternating fresh YOLO association and template fallback
    match=select_tracking_candidate(candidates,b,.1,.5,.5)
    assert match is not None
    b=smooth_bbox(b,match[0],frame.shape,.65)
    trackers[j].initialize(frame,b)
   else:
    tracked=trackers[j].track(frame)
    assert tracked.bbox is not None
    b=tracked.bbox
   classify_signal_color(frame,b,.004,.004)
 for i in range(100):work(2,i)
 measurements={1:[],2:[]}
 for rep in range(8):
  for count in ([1,2] if rep%2==0 else [2,1]):
   wall=time.perf_counter();cpu=time.process_time()
   for i in range(500):work(count,i)
   measurements[count].append({'wall_ms':(time.perf_counter()-wall)*1000/500,'cpu_ms':(time.process_time()-cpu)*1000/500})
 summary={str(count):{key:statistics.median(x[key] for x in samples) for key in ['wall_ms','cpu_ms']} for count,samples in measurements.items()}
 delta_wall=summary['2']['wall_ms']-summary['1']['wall_ms']
 delta_cpu=summary['2']['cpu_ms']-summary['1']['cpu_ms']
 result['scenarios'].append({'box_size':[width,height],'measurements':summary,'additional_ms':delta_wall,'additional_cpu_ms':delta_cpu,'cpu_percentage_point_increase_one_core_at_10fps':delta_cpu,'cpu_percentage_point_increase_whole_machine_at_10fps':delta_cpu/os.cpu_count(),'samples':measurements})
 print('completed box',width,height,flush=True)
model=YOLO('src/stack_traffic/models/yolov8n.pt')
for _ in range(3):model.predict(frame,imgsz=640,conf=.1,classes=[9],max_det=10,rect=True,verbose=False)
wall_times=[];cpu_times=[]
for _ in range(30):
 wall=time.perf_counter();cpu=time.process_time()
 model.predict(frame,imgsz=640,conf=.1,classes=[9],max_det=10,rect=True,verbose=False)
 wall_times.append((time.perf_counter()-wall)*1000);cpu_times.append((time.process_time()-cpu)*1000)
result['yolo']={'device':str(model.predictor.device),'median_wall_ms':statistics.median(wall_times),'median_cpu_ms':statistics.median(cpu_times),'runs':30}
for s in result['scenarios']:
 # Before red confirmation: one YOLO call per two processed frames.
 baseline=result['yolo']['median_wall_ms']/2+s['measurements']['1']['wall_ms']
 s['estimated_processing_increase_percent']=s['additional_ms']/baseline*100
 s['estimated_mean_frame_ms_one']=baseline
 s['estimated_mean_frame_ms_two']=baseline+s['additional_ms']
Path('analysis/traffic_two_target_benchmark/results.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2),flush=True)
