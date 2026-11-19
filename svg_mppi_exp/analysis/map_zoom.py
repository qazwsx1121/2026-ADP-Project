import sys, numpy as np, yaml, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from PIL import Image
D="/home/misys/forza_ws/race_stack/stack_master/maps/berlin/"
m=yaml.safe_load(open(D+"berlin.yaml")); img=np.array(Image.open(D+m["image"]).convert("L"))
H,W=img.shape; r=m["resolution"]; o=m["origin"]; ext=[o[0],o[0]+W*r,o[1],o[1]+H*r]
pts=[tuple(map(float,p.split(","))) for p in sys.argv[2:]]
fig,axs=plt.subplots(1,len(pts),figsize=(5*len(pts),5),squeeze=False)
for ax,(x,y) in zip(axs[0],pts):
    ax.imshow(img,cmap="gray",extent=ext,origin="upper",interpolation="nearest")
    ax.plot(x,y,"r+",ms=20,mew=2); ax.set_xlim(x-1.5,x+1.5); ax.set_ylim(y-1.5,y+1.5); ax.set_title(f"({x},{y})")
fig.savefig(sys.argv[1],dpi=80)
