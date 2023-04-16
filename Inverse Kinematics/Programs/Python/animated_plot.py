# -*- coding: utf-8 -*-
"""
Created on Sun Feb 19 10:28:38 2023

https://www.askpython.com/python-modules/matplotlib/animated-plots

@author: Philipp Schulz
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random
from itertools import count
from IPython import display

x=[]
y=[]
i = count()
def animate(j):
    x.append(next(i))
    y.append(random.randint(0, 10))
    plt.plot(x,y)

# 1.
#"""
animation_1 = animation.FuncAnimation(plt.gcf(),animate,interval=1000)
plt.show()
#"""

# 2.
"""
video_1 = animation_1.to_html5_video()
html_code_1 = display.HTML(video_1)
display.display(html_code_1)
plt.tight_layout()
plt.show()
#"""

# 3. 

"""
x1=[]
y1=[]
i1 = count()
def animate1(j):
    t=next(i1)
    x1.append(2*t)
    y1.append(np.sin(t))
    plt.cla()
    plt.plot(x1,y1)
animation_2 = animation.FuncAnimation(plt.gcf(),animate1,interval=50)
video_2 = animation_2.to_html5_video()
html_code_2 = display.HTML(video_2)
display.display(html_code_2)
plt.tight_layout()
plt.show()
#"""

# 4. 
"""
lt.style.use('dark_background')
x=[]
y_sin=[]
y_cos=[]
i_n = count()
def animate_n(j):
    t=2*next(i)
    x.append(t)
    y_sin.append(np.sin(t))
    y_cos.append(np.cos(t))
    plt.cla()
    plt.plot(x,y_sin,label="Sine wave",color="red")
    plt.plot(x,y_cos,label="Cosine wave",color="green")
animation_n = animation.FuncAnimation(plt.gcf(),animate_n,interval=500)
video_n = animation_n.to_html5_video()
html_code_n = display.HTML(video_n)
display.display(html_code_n)
plt.tight_layout()
plt.show()
#"""