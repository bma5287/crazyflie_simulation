import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

BASEPATH = "crazyflie_projects/Featureless_TTC/"

## PLOT BRIGHTNESS PATTERN FROM 2.4.1 HORIZONTAL MOTION
I_0 = 255   # Brightness value (0-255)      60
L = 0.25   # Stripe Period [m]              0.5


## CAMERA PARAMETERS
WIDTH_PIXELS = 160
HEIGHT_PIXELS = 160
FPS = 120                # Frame Rate [1/s]
w = 3.6e-6              # Pixel width [m]
f = 0.66e-3/2             # Focal Length [m]
O_x = WIDTH_PIXELS/2    # Pixel X_offset [pixels]
O_y = HEIGHT_PIXELS/2   # Pixel Y_offset [pixels]

d = 0.6

Xi_Width = d*(WIDTH_PIXELS*w)/f 
Yi_Width = d*(HEIGHT_PIXELS*w)/f 

x = np.linspace(-1.0, 1.0, 1000)    # [m]
y = np.linspace(-1.0, 1.0, 1000)    # [m]
X, Y = np.meshgrid(x, y)

def Intensity(X):
    I_x = I_0/2*(np.sin(2*np.pi*X/L)+1)
    I_y = I_0/2*(np.sin(2*np.pi*Y/L)+1)
    I =  (I_x+I_y)/2

    # I = np.round(I/255,0)*255
    return I

fig, ax = plt.subplots()
im = ax.imshow(Intensity(X), interpolation='none', 
                vmin=0, vmax=255, cmap=cm.gray,
                origin='upper',
                extent=[-1,1,-1,1]
)

ax.add_patch(Rectangle((-Xi_Width/2,-Yi_Width/2),Xi_Width,Yi_Width,lw=2,fill=False,color="tab:blue"))

ax.set_title("Floor Pattern")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
fig.tight_layout()

plt.imsave(
    f'{BASEPATH}/Surface_Patterns/Stripe.png', 
    Intensity(X), 
    vmin=0, vmax=255, cmap=cm.gray,
    origin='upper',
)

plt.show()
