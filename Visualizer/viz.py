from email.mime import image
import tkinter as tk
from tkinter import *
from PIL import ImageTk, Image


angle = 50
pos_x = 100
pos_y = 200
robot_image = Image.open("assets/robot.png")
# robot_image = robot_image.resize((50, 50), Image.ANTIALIAS)

window = tk.Tk()
window.title("Robot & Path Visualizer")
window.geometry("800x600")

canvas = tk.Canvas(window, width=800, height=600)
canvas.pack()

img = ImageTk.PhotoImage(robot_image.rotate(angle))  
canvas.create_image(pos_x, pos_y, anchor=tk.CENTER, image=img) 

window.mainloop()