#!/usr/bin/python

from Tkinter import *
import argparse
from PIL import ImageTk, Image
import matplotlib.pyplot as plt
import numpy as np
from skimage import draw
import cv2
'''
Draw on images to get max and min color values use -i to select the image
'''

ap =argparse.ArgumentParser()
ap.add_argument('-i','--image', required= True, help='path to image')
ap.add_argument('-c','--color', required= False, help='marker color')

args= vars(ap.parse_args())
im = plt.imread(args['image'])

if im.dtype != np.uint8 and np.max(im) <= 1.0 and np.min(im) >= 0:
    img = np.uint8(255 * im)
elif im.dtype == np.uint8 and np.max(im) <= 255 and np.min(im) >= 0:
    img = im.copy()
else:
    raise ValueError("Input image must be either dtype uint8 with range [0, 255] or float with range [0, 1] but got im.dtype = {}".format(im.dtype))

colors={'red','yellow','blue','green','magenta','orange', 'black', 'white'}
color="black"
if args['color'] is not None:
    if args['color'] in colors:
        color=args['color']
    else:
        raise ValueError("Must choose a valid color") 
root = Tk()

root.title("HSV Range Finder")

root.resizable(0,0)
lastx = 0
lasty = 0

val=1
anno=np.zeros(img.shape[:-1])


def mouse_down(event):
    global lastx, lasty
    h, w = anno.shape
    lastx = min(max(event.x, 0), w - 1)
    lasty = min(max(event.y, 0), h - 1)
    x = min(max(event.x - 1, 0), w - 1)
    y = min(max(event.y - 1, 0), h - 1)
    c.create_line(lastx, lasty, x, y, fill=color)
    rr, cc = draw.line(lastx, lasty, x , y)
    anno[cc, rr] = val

def mouse_drag(event):
    global lastx, lasty
    h, w = anno.shape
    x = min(max(event.x, 0), w - 1)
    y = min(max(event.y, 0), h - 1)
    c.create_line(lastx, lasty, x, y, fill=color)
    rr, cc = draw.line(lastx, lasty, x , y)
    anno[cc, rr] = val
    lastx = x
    lasty = y

c = Canvas(root, width=img.shape[1], height= img.shape[0])

c.configure(cursor="crosshair")
photo = ImageTk.PhotoImage(Image.fromarray(np.asarray(img)))
c.create_image(img.shape[1]//2, img.shape[0]//2, image=photo)
c.pack()

c.bind("<Button-1>", mouse_down)

c.bind("<B1-Motion>", mouse_drag)


root.mainloop()
points=np.where(anno==1)
hsv_img=cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
hsv_samples=hsv_img[points]
hsv_max=np.amin(hsv_samples,axis=0)
hsv_min=np.amax(hsv_samples,axis=0)

print(hsv_max)
print(hsv_min)