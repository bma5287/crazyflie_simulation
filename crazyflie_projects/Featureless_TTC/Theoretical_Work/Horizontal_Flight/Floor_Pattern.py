import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from matplotlib.patches import Rectangle


## PLOT BRIGHTNESS PATTERN FROM 2.4.1 HORIZONTAL MOTION
I_0 = 255   # Brightness value (0-255)
L = 0.1     # Stripe Period [m]


## CAMERA PARAMETERS
WIDTH_PIXELS = 320
HEIGHT_PIXELS = 320
FPS = 60                # Frame Rate [1/s]
w = 3.6e-6              # Pixel width [m]
f = 0.66e-3             # Focal Length [m]
O_x = WIDTH_PIXELS/2    # Pixel X_offset [pixels]
O_y = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]

d = 0.573

Xi_Width = d*(WIDTH_PIXELS*w)/f 
Yi_Width = d*(HEIGHT_PIXELS*w)/f 

x = np.linspace(-1.0, 1.0, 1000)    # [m]
y = np.linspace(-1.0, 1.0, 1000)    # [m]
X, Y = np.meshgrid(x, y)

def Intensity(X):
    I = I_0/2*(np.sin(2*np.pi*X/L)+1)
    I = np.round(I/255,0)*255
    return I

fig, ax = plt.subplots()
im = ax.imshow(Intensity(X), interpolation='bilinear', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',
                extent=[-1,1,-1,1]
)

ax.add_patch(Rectangle((-Xi_Width/2,-Yi_Width/2),Xi_Width,Yi_Width,lw=2,fill=False,color="tab:blue"))

ax.set_title("Floor Pattern")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
fig.tight_layout()

plt.show()
